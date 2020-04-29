'''
	ips_monitor.py


'''

import logger

import time
import socket
import json
import threading

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation

#Setup for TCP server
TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 20  #Default is 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))

#Global variable for multithreading thing
data_string_thread_out = None
data_string_thread_out_last = None

#Setup for data logger
path = "/Users/jarrettphilips/Desktop/ips_logs"
log = logger.Logger(path)

#Setup for visual plot
plt.style.use('dark_background')
fig = plt.figure(num='IPS Monitor')

ax = fig.add_subplot(111, projection='3d')
ax.autoscale(enable=True, axis='both', tight=True)
ax.w_xaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
ax.w_yaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
ax.w_zaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
graph = ax.scatter([0], [0], [0], color='yellow')
plt.show(block=False)
plt.pause(0.0001)
graph = ax.scatter([3], [2], [-1], color='blue')
plt.show(block=False)
plt.pause(0.0001)

#####################################
def wait_for_data():
	while 1 :
		s.listen(1)
		conn, addr = s.accept()
		print('Connection address:', addr)

		while 1:
			data_string = conn.recv(BUFFER_SIZE)
			if not data_string: break
			print("received data:", data_string)

			global data_string_thread_out
			data_string_thread_out = data_string.decode()
			#return data_string.decode()
	conn.close()

def parse_data(data_string):
	data_string_split = data_string.split(',')
	return (float(data_string_split[0]), float(data_string_split[1]), float(data_string_split[2]))

def log_point(x, y, z):
	json_point = json.dumps([x, y, z])
	log.update_log(json_point + '\n')

def plot_point(x, y, z):
	global graph
	graph = ax.scatter([x], [y], [z], color='red')
	plt.show(block=False)
	#plt.pause(0.0001)
	fig.canvas.start_event_loop(0.001)

#####################################
def main():
	t = threading.Thread(target=wait_for_data, args=())
	t.start()
	while 1: 
		#data_string = wait_for_data()
		#(x, y, z) = parse_data(data_string)

		#plt.pause(.1)
		fig.canvas.start_event_loop(0.001)
		global data_string_thread_out_last
		if (data_string_thread_out != None) and (data_string_thread_out != data_string_thread_out_last):
			data_string_thread_out_last = data_string_thread_out
			(x, y, z) = parse_data(data_string_thread_out)

			log_point(x, y, z)
			plot_point(x, y, z)

main()
log.close_log()
plt.show(block=True)

