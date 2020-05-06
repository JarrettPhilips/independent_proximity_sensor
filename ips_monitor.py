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

#Setup for UDP client
msgFromClient       = "Hello UDP Server"
bytesToSend         = str.encode(msgFromClient)
serverAddressPort   = ("192.168.1.118", 2390)
bufferSize          = 1024
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPClientSocket.sendto(bytesToSend, serverAddressPort)

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

#####################################
def wait_for_data_udp():
	while 1: 
		msgFromServer = UDPClientSocket.recvfrom(bufferSize)
		global data_string_thread_out
		data_string_thread_out = "{}".format(msgFromServer[0].decode())
		print(data_string_thread_out)

def parse_data(data_string):
	data_string_split = data_string.split(',')
	return (float(data_string_split[0]), float(data_string_split[1]), float(data_string_split[2]), float(data_string_split[3]), int(data_string_split[4]))

def log_point(x, y, z):
	json_point = json.dumps([x, y, z])
	log.update_log(json_point + '\n')

def plot_point(x, y, z):
	global graph
	graph = ax.scatter([x], [y], [z], color='red')
	plt.show(block=False)
	fig.canvas.draw()
	#plt.pause(0.0001)
	fig.canvas.start_event_loop(0.001)

def tait_bryan_to_quaternion(psi, theta, phi):
	qw = np.cos(phi/2)*np.cos(theta/2)*np.cos(psi/2)+np.sin(phi/2)*np.sin(theta/2)*np.sin(psi/2)
	qx = np.sin(phi/2)*np.cos(theta/2)*np.cos(psi/2)-np.cos(phi/2)*np.sin(theta/2)*np.sin(psi/2)
	qy = np.cos(phi/2)*np.sin(theta/2)*np.cos(psi/2)+np.sin(phi/2)*np.cos(theta/2)*np.sin(psi/2)
	qz = np.cos(phi/2)*np.cos(theta/2)*np.sin(psi/2)-np.sin(phi/2)*np.sin(theta/2)*np.cos(psi/2)

	return [qw, qx, qy, qz]

def multiply_quaternions(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def get_quaternion_prime(q):
	return [q[0], -1*q[1], -1*q[2], -1*q[3]]

def rotate_vector(v, q):
	q_prime = get_quaternion_prime(q)
	v_prime = multiply_quaternions(multiply_quaternions(q, v), q_prime)
	return v_prime

#####################################
def main():
	t = threading.Thread(target=wait_for_data_udp, args=())
	t.start()
	while 1: 
		fig.canvas.start_event_loop(0.1)
		global data_string_thread_out_last
		if (data_string_thread_out != None) and (data_string_thread_out != data_string_thread_out_last):
			data_string_thread_out_last = data_string_thread_out
			(roll, pitch, yaw, distance, sensor_id) = parse_data(data_string_thread_out)
			vector = [0, 0, 0, 0] #forward vector
			if sensor_id == 1:
				vector = [0, distance, 0, 0]
			elif sensor_id == 2:
				vector = [0, 0, distance, 0]
			elif sensor_id == 3:
				vector = [0, 0, -distance, 0]
			elif sensor_id == 4:
				vector = [0, 0, 0, distance]
			print("vector:", vector)
			quaternion = tait_bryan_to_quaternion(np.radians(roll), np.radians(pitch), np.radians(yaw))
			print("quaternion:", quaternion)
			rotated_vector = rotate_vector(vector, quaternion)
			print("rotated_vector:", rotated_vector)

			log_point(rotated_vector[0], rotated_vector[1], rotated_vector[2])
			plot_point(rotated_vector[0], rotated_vector[1], rotated_vector[2])

main()
log.close_log()
plt.show(block=True)

