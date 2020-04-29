'''
	ips_monitor.py


'''

import logger.py

import time
import socket
import json

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation

#Setup for TCP server
TCP_IP = '25.118.16.61'
TCP_PORT = 5005
BUFFER_SIZE = 20  #Default is 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))

#Setup for data logger
path = "/home/jarrettphilips/Desktop/ips_logs"
log = Logger(path)

#Setup for visual plot
plt.style.use('dark_background')
fig = plt.figure(num='IPS Monitor')

ax = fig.add_subplot(111, projection='3d')
ax.autoscale(enable=True, axis='both', tight=True)
ax.w_xaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
ax.w_yaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
ax.w_zaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))

#####################################
def wait_for_data():
	s.listen(1)
	conn, addr = s.accept()
	print('Connection address:', addr)

	while 1:
		data_string = conn.recv(BUFFER_SIZE)
		if not data_string: break
		print("received data:", data_string)

	return data_string

def parse_data(data_string):
	x = []
	y = []
	z = []
	

	return (x, y, z)

def log_point(x, y, z):
	json_point = json.dumps([x, y, z])
	log.update_log(json_point + '\n')

def plot_point(x, y, z):
	global graph
	graph = ax.scatter([x], [y], [z], color='red')
	plt.show(block=False)

#####################################
def main():
	while 1: 
		print("main")
		data_string = wait_for_data()
		(x, y, z) = parse_data(data_string)

		for i in range(len(x))
			log_point(x[i], y[i], z[i])
			plot_point(x[i], y[i], z[i])

main()
conn.close()
log.close_log()
plt.show(block=True)

