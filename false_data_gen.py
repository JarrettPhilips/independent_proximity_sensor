'''
	false_data_gen.py

	File for generating fake data to test the functionality of the ips monitor
'''

import random
import json
import socket
import time

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024
#MESSAGE = '-1,-10,1'

while 1 :
	x = random.randint(-100, 100)
	y = random.randint(-100, 100)
	z = random.randint(-100, 100)
	MESSAGE = str(x) + ',' + str(y) + ',' + str(z)

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	s.send(MESSAGE.encode())
	s.close()

	print("sent:", MESSAGE)
	time.sleep(2)