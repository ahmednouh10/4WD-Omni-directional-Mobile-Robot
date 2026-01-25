import socket
import time
import json

ESP32_IP = "172.30.101.210"
PORT = 5000

def connect_to_esp():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((ESP32_IP, PORT))
        print(f"Connected to ESP32 at {ESP32_IP}:{PORT}")
        return s
    except Exception as e:
        print(f"Connection Failed: {e}")
        return None

sock = connect_to_esp()

while True:
    if sock is None:
        print("Retrying connection...")
        time.sleep(2)
        sock = connect_to_esp()
        continue

    try:
        data_list = [0, 0, 0, -255]
        json_str = json.dumps(data_list)
        sock.sendall((json_str + '\n').encode())
        print(f"Sent: {json_str}")
        time.sleep(2)

    except (BrokenPipeError, ConnectionResetError):
        print("Connection lost. Reconnecting...")
        sock.close()
        sock = None
    except KeyboardInterrupt:
        print("Closing connection")
        sock.close()
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)
