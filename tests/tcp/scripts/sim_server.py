#!/usr/bin/env python3
import socket
import time

HOST = "192.0.2.2"  # Standard loopback interface address (localhost)
PORT = 4000  # Port to listen on (non-privileged ports are > 1023)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_TCP, 18, 100)

sock.bind((HOST, PORT))
sock.listen()

print(f'listening on {HOST}:{PORT}')

conn, addr = sock.accept()

print(f"Connection from {addr}")
t_sim = 0
dt = 0.1
while True:
    try:
        data = conn.recv(1024)
        if not data:
            print('attempting to reconnect')
            conn.close()
            t_sim = 0
            conn, addr = sock.accept()
            continue
        print("received data: {:s}".format(data.decode('utf-8')))
        #conn.sendall(data)
        conn.sendall("{:d}".format(int(t_sim*1e3)).encode('utf-8'))
        time.sleep(dt)
        t_sim += dt
    except KeyboardInterrupt as e:
        print('closing connection')
        conn.close()
        break
    except TimeoutError as e:
        print('timeout, attempting to reconnect')
        conn.close()
        conn, addr = sock.accept()
        t_sim = 0
        continue