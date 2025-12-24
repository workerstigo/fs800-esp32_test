import socket
import base64
import cv2
import numpy as np
import time
import binascii

HOST = '0.0.0.0'
PORT = 5000
IMAGE_TIMEOUT = 10  # 秒

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(5)
    print(f"Server listening on {HOST}:{PORT}")

    while True:
        conn, addr = s.accept()
        print(f"Connected by {addr}")
        conn.settimeout(1.0)

        data_buffer = b""
        last_image_time = time.monotonic()

        with conn:
            while True:
                # watchdog
                if time.monotonic() - last_image_time > IMAGE_TIMEOUT:
                    print("Image timeout, restarting connection")
                    break

                try:
                    data = conn.recv(4096)
                    if not data:
                        print(f"Connection closed by {addr}")
                        break
                    data_buffer += data
                except socket.timeout:
                    continue

                while b"IMAGE_START" in data_buffer and b"IMAGE_END" in data_buffer:
                    start = data_buffer.find(b"IMAGE_START") + len(b"IMAGE_START\n")
                    end = data_buffer.find(b"IMAGE_END")

                    image_data_b64 = data_buffer[start:end].replace(b"\n", b"")

                    # 先清 buffer，避免壞資料殘留
                    data_buffer = data_buffer[end + len(b"IMAGE_END\n"):]

                    # Base64 解碼
                    try:
                        image_bytes = base64.b64decode(image_data_b64, validate=True)
                    except (binascii.Error, ValueError):
                        print("Base64 decode failed, drop frame")
                        continue

                    # ⭐ 關鍵防呆 1：空資料直接丟
                    if not image_bytes:
                        print("Empty image bytes, drop frame")
                        continue

                    nparr = np.frombuffer(image_bytes, np.uint8)

                    # ⭐ 關鍵防呆 2：空 buffer 不能給 imdecode
                    if nparr.size == 0:
                        print("Empty numpy buffer, drop frame")
                        continue

                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                    if img is None:
                        print("Image decode failed, drop frame")
                        continue

                    last_image_time = time.monotonic()

                    cv2.namedWindow("ESP32 Image", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("ESP32 Image", 640, 480)
                    cv2.imshow("ESP32 Image", img)
                    cv2.waitKey(1)

                    try:
                        conn.sendall(b"OK\n")
                    except:
                        break
