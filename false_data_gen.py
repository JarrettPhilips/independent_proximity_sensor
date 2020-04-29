'''
	false_data_gen.py

	File for generating fake data to test the functionality of the ips monitor
'''

import json
import socket


TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024
MESSAGE = '1,1,1'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
s.send(MESSAGE.encode())
s.close()

print("sent:", MESSAGE)