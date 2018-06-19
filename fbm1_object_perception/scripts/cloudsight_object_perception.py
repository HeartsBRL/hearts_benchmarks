#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Image
#from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState
#from geometry_msgs.msg import Pose2D
#import std_srvs.srv

import re
import cloudsight 

#import urllib3
#urllib3.disable_warnings()

class Comms():

	def __init__(self):
		rospy.Subscriber('/camera/rgb/image_raw/crop', String, self.CropImageFileCallback)

		self.pubGoal = rospy.Publisher('/hearts/or/item', String, queue_size=1)
		self.pubStatus = rospy.Publisher('/hearts/cloudsight/status', String, queue_size=1)

		self.auth = cloudsight.OAuth('t-xw3Mn06dkObZwsi8qpcA', 'laJ83zYRxAVpls1Aera8Mg')
		self.api = cloudsight.API(self.auth)
	
		self.image_attempts = 0
		self.max_attempts = 2

        def recognise(self, filename):
                rospy.loginfo("Opening File: " + filename)
		
		with open(filename, 'rb') as f:
		    response = self.api.image_request(f, filename, {
		        'image_request[locale]': 'en-US',
		    })

		status = self.api.image_response(response['token'])
		while status['status'] == cloudsight.STATUS_NOT_COMPLETED:
			time.sleep(5)
			status = self.api.image_response(response['token'])
			# Done!
			rospy.loginfo('waiitng....')

			pass

		#status = self.api.wait(response['token'], timeout=30)
		parse_string = str(status)
		#rospy.loginfo(parse_string)		
		#print status


		#parse_string = "{u'url': u'http://assets.cloudsight.ai/uploads/image_request/image/303/303733/303733899/Boxes_Yellow_box1.jpg', u'status': u'completed', u'token': u'o1L9gKETlGdcwkOLzdjr3w', u'name': u'fluffy yellow plastic container', u'ttl': 54.0}"
		#parse_string.lower()

		matched_string = "unknown"
		match=re.compile("name': u'(.*?)',")
		try:
			matched_string= re.search(match, parse_string).group(1)
		except:
			self.image_attempts = self.image_attempts + 1
			#rospy.loginfo('Cloud could not identify image. Try again?')
			return 1
		#matched_string = matched_string.lower()
		rospy.loginfo("Cloud String: " + matched_string)

		split_string = matched_string.split()

		classes = {}
		objects = {}
		# these should be unique identifiers for the class
		classes['mugs'] = ['ceramic', 'mug', 'cup', 'coffee', 'mucup', 'bucket', 'vase', 'teacup']
		classes['forks and knives'] = ['knife', 'fork', 'spoon', 'slicer', 'cake', 'butter', 'toy', 'line', 'pie']
		classes['boxes'] = ['box', 'container', 'storage', 'case', 'cube', 'chest']
		classes['picture frame'] = ['frame', 'photo', 'fram', 'device', 'electronic', 'gadget'] 

		# these should be unique to the object
		objects['grey mug'] = ['grey', 'gray', 'teal', 'blue']
		objects['white mug with yellow lines'] = [ 'stripe', 'orange', 'striped', 'yellow']
		objects['white mug with yellow dots'] = ['polka', 'white']
		objects['black mug'] = ['black' ]
		objects['knife'] = ['purple', 'knife', 'butter', 'slicer', 'server']
		objects['fork'] = ['blue', 'fork', 'teal']
		objects['spoon'] = ['red', 'pink']
		objects['yellow box'] = ['yellow']
		objects['pink box'] = ['pink', 'red']
		objects['frame with SPARC logo'] = ['sparc']
		objects['frame with ERL logo'] = ['erl']

		mugs = {}
		forks_knives = {}
		boxes = {}
		frames = {}
		mugs['grey mug'] = objects['grey mug']
		mugs['white mug with yellow lines'] = objects['white mug with yellow lines']
		mugs['white mug with yellow dots'] = objects['white mug with yellow dots']
		mugs['black mug'] = objects['black mug']
		forks_knives['knife'] = objects['knife']
		forks_knives['fork'] = objects['fork']
		forks_knives['spoon'] = objects['spoon']
		boxes['yellow box'] = objects['yellow box']
		boxes['pink box'] = objects['pink box']
		frames['frame with SPARC logo'] = objects['frame with SPARC logo']
		frames['frame with ERL logo'] = objects['frame with ERL logo']

		class_name = 'unknown'
		object_name = 'unknown'
		possible_objects = []


		for keyword in split_string:
			for key, val in classes.iteritems():
				if keyword in val:
					class_name = key
					break;	


		if class_name == 'mugs' :
			for keyword in split_string:
				for key, val in mugs.iteritems():
					if keyword in val:
						object_name = key
	
		elif class_name == 'forks and knives' :
			for keyword in split_string:
				for key, val in forks_knives.iteritems():
					if keyword in val:
						object_name = key
						#possible_objects.append(key)
	
		elif class_name == 'boxes' :
			for keyword in split_string:
				for key, val in boxes.iteritems():
					if keyword in val:
						object_name = key
						#possible_objects.append(key)

		elif class_name == 'frames' :
			for keyword in split_string:
				for key, val in frames.iteritems():
					if keyword in val:
						object_name = key
						#possible_objects.append(key)

		elif class_name == 'unknown':
			#search everything to find SOMETHING
			for keyword in split_string:
				for key, val in objects.iteritems():
					if keyword in val:
						object_name = key
						#possible_objects.append(key)

#		if len(possible_objects) < 0:
#			object_name = max(set(possible_objects), key=possible_objects.count)
#			print object_name

		lu_classes = {
			'mugs' : 'a',
			'forks and knives' : 'b' ,
			'boxes' : 'c',
			'picture frame' : 'd',
			'unknown' : 'unknown'
		}
		lu_objects = {
			'grey mug' : 'a1',
			'white mug with yellow lines' : 'a2',
			'white mug with yellow dots' : 'a3',
			'black mug' : 'a4',
			'knife' : 'b1',
			'fork' : 'b2',
			'spoon' : 'b3',
			'yellow box' : 'c1',
			'pink box' : 'c2',
			'frame with SPARC logo' : 'd1',
			'frame with ERL logo' : 'd2',
			'unknown' : 'unknown'
		}
		
                return class_name, lu_classes[class_name], object_name, lu_objects[object_name]

	def CropImageFileCallback(self, data):
		
		#filename = "/home/kaya/hearts/src/hearts/object_perception_2/data/Vaseline_1.jpg"
		file_paths = data.data.split(',')

		for i in range(0, len(file_paths)):
			#for j in range(0, 1): # self.max_attempts
			file_path = file_paths[i]
			rospy.loginfo("Trying image " + str(i))
			rospy.loginfo("Filepath: " + file_path)
			cloud_return = self.recognise(file_path)
			if cloud_return == 1:
				rospy.loginfo('Failed to identify object (cloud error)')
				continue		
			class_name, pub_class_name, object_name, pub_object_name = cloud_return 
			if pub_class_name == 'unknown' and pub_object_name == 'unknown':
				rospy.loginfo('Failed to identify object (no match to item dictionary)')
                                #self.pubStatus.publish("bad")
				continue
			elif pub_object_name == 'unknown':
				pub_object_name = pub_class_name + '1'
			elif pub_class_name == 'unknown':
				pub_class_name = result = ''.join([i for i in pub_object_name if not i.isdigit()])
			rospy.loginfo('identified object - Class: ' + class_name + ' (' + pub_class_name + ') , Object: ' + object_name  + ' (' + pub_object_name + ')')
			self.pubGoal.publish(pub_class_name + ',' + pub_object_name)
			rospy.loginfo(':) Another Object Please')
			self.image_attempts = 0
			return 1

		if self.image_attempts < 2:
			rospy.loginfo('Can\'t identify image. Take another one please!')
			self.pubStatus.publish('try again')
			self.image_attempts = self.image_attempts + 1
		else:
			rospy.loginfo('Tried ' + str(self.max_attempts) + ' times. Can\'t identify. Try another object.')
			self.pubGoal.publish('unknown,unknown') 
			self.image_attempts = 0

if __name__ == '__main__':
	rospy.init_node('cloudsight_objectperception', anonymous=True)
	rospy.loginfo("CLOUDSIGHT: Started")
	c = Comms()
	rospy.spin()

