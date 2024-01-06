#!/usr/bin/env python3
import socket
import time

UDP_IP = "192.0.2.1"
UDP_PORT = 4242
MESSAGE = b"test"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

for i in range(1000):
    time.sleep(0.1)
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    print("message: %s" % MESSAGE)
