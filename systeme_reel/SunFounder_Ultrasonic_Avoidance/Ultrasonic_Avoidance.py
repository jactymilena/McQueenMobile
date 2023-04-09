#!/usr/bin/env python
'''
**********************************************************************
* Filename    : Ultrasonic_Avoidance.py
* Description : A module for SunFounder Ultrasonic Avoidance
* Author      : Cavon
* Brand       : SunFounder
* E-mail      : service@sunfounder.com
* Website     : www.sunfounder.com
* Update      : Cavon    2016-09-26    New release
**********************************************************************
'''
import time
import RPi.GPIO as GPIO
import constants as const

class Ultrasonic_Avoidance(object):
	timeout = 0.05

	def __init__(self, channel):
		self.channel = channel
		GPIO.setmode(GPIO.BCM)

		self.last_measures = []
		self.measures_index = 0

	def distance(self):
		pulse_end = 0
		pulse_start = 0
		GPIO.setup(self.channel,GPIO.OUT)
		GPIO.output(self.channel, False)
		time.sleep(0.01)
		GPIO.output(self.channel, True)
		time.sleep(0.00001)
		GPIO.output(self.channel, False)
		GPIO.setup(self.channel,GPIO.IN)

		timeout_start = time.time()
		while GPIO.input(self.channel)==0:
			pulse_start = time.time()
			if pulse_start - timeout_start > self.timeout:
				return -1
		while GPIO.input(self.channel)==1:
			pulse_end = time.time()
			if pulse_start - timeout_start > self.timeout:
				return -1

		if pulse_start != 0 and pulse_end != 0:
			pulse_duration = pulse_end - pulse_start
			distance = pulse_duration * 100 * 343.0 /2
			distance = int(distance)
			#print('start = %s'%pulse_start,)
			#print('end = %s'%pulse_end)
			if distance >= 0:
				return distance
			else:
				return -1
		else :
			#print('start = %s'%pulse_start,)
			#print('end = %s'%pulse_end)
			return -1


	def get_distance(self, mount = 5):
		sum = 0
		for i in range(mount):
			a = self.distance()
			#print('    %s' % a)
			sum += a
		return int(sum/mount)


	def less_than(self, alarm_gate):
		dis = self.get_distance()
		status = 0
		if dis >=0 and dis <= alarm_gate:
			status = 1
		elif dis > alarm_gate:
			status = 0
		else:
			status = -1
		#print('distance =',dis)
		#print('status =',status)
		return status
	

	def clear_measures(self):
		self.last_measures[:] = []


	# def calibrate(self):
	# 	for i in range(30):
	# 		t = self.get_distance()
	# 		time.sleep(0.2)


	def detect_obstacle(self):
		measure = self.get_distance()

		self.measures_index += 1


		# if measure + 10 < measures_mean || measure - 10 > measures_mean


		if len(self.last_measures) >= 5:
			index = (self.measures_index) % 5
			print("index " + str(index) + " measures_index " +  str(self.measures_index))
			self.last_measures[index] = measure
		else:
			self.last_measures.append(measure)
			return False

			# if len(self.last_measures) < 5: 
			# # self.last_measures.append(measure)
			# 	return False


		measures_mean = sum(self.last_measures) / len(self.last_measures)

		print(self.last_measures)

		print("mean : " + str(measures_mean))
		
		
		# print("distance " + str(measures_mean)  + " sensor_close_range " + str(const.SENSOR_CLOSE_RANGE)  + " " + str(int(measures_mean) < int(const.SENSOR_CLOSE_RANGE)) )

		# for
		# distances = [self.get_distance() for i in range(30)]
		# for i in range(15):
		# 	distances.append(self.get_distance())
		# print(distances)

		return (measures_mean <= const.SENSOR_CLOSE_RANGE) #and not (test <= 2)


def test():
	UA = Ultrasonic_Avoidance(17)
	threshold = 10
	while True:
		distance = UA.get_distance()
		status = UA.less_than(threshold)
		print('distance', distance, 'cm')
		if distance != -1:
			print('distance', distance, 'cm')
			time.sleep(0.2)
		else:
			print(False)
		if status == 1:
			print("Less than %d" % threshold)
		elif status == 0:
			print("Over %d" % threshold)
		else:
			print("Read distance error.")


if __name__ == '__main__':
	test()