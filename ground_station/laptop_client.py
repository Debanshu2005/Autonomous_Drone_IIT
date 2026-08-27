import socket
import threading
import sys
import time

def receive_telemetry(sock):
    while True:
        try:
            data = sock.recv(1024)
            if not data:
                break
            print(f"\r{data.decode('utf-8').strip()}\n[CMD] > ", end="", flush=True)
        except Exception as e:
            print(f"\nConnection lost: {e}")
            break

def main():
    host = "192.168.4.1"  # Default Pi 5 hotspot IP
    port = 5000

    if len(sys.argv) > 1:
        host = sys.argv[1]

    print(f"Connecting to Edge Brain at {host}:{port}...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((host, port))
        print("Connected!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        sys.exit(1)

    # Start a background thread to listen for telemetry/status from the edge
    threading.Thread(target=receive_telemetry, args=(sock,), daemon=True).start()

    try:
        while True:
            cmd = input("[CMD] > ")
            if cmd.lower() in ['quit', 'exit']:
                break
            if cmd.strip():
                sock.sendall(cmd.encode('utf-8'))
                time.sleep(0.1) # Brief pause to allow any immediate response to print
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print("Disconnected.")

if __name__ == "__main__":
    main()
