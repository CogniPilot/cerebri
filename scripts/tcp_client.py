#!/usr/bin/env python3
import socket

HOST = '192.0.2.1'
PORT = 4242

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b'Hello, world')
    data = s.recv(1024)

print('Echoing: ', repr(data))
