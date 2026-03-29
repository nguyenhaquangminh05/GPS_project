import socket
import serial   

SERVER_IP = "192.168.4.1"
SERVER_PORT = 2101

def main():
    sock =socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((SERVER_IP, SERVER_PORT))
    print(f"Connected to server:{SERVER_IP}:{SERVER_PORT}")
    request = "GET /KINDHELM\r\n"
    sock.sendall(request.encode())
    print("Sent request to server, awaiting response...")

    try:
        while True:
            data = sock.recv(4096)
            if not data:
                print("Server closed the connection.")
                break
            print(f"\nRTCM ({len(data)} bytes):\n{data.hex()[:3000]}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        sock.close()
        print("Connection closed.")
if __name__ == "__main__":
    main()