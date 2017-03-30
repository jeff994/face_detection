#!/usr/bin/env python
import rospy
import string
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
import cv2
import sys

from time import sleep

#--------------------------PARAMETERS TO UPDATE--------------------------#
cascPath 		= '/home/aaron/nova_ws/src/face_detection/scripts/haarcascade_frontalface_default.xml' 	#we can change this

camera_path 		= 0 		#laptop, webcam = 0, USB cam = 1
#------------------------------------------------------------------------#


faceCascade  		= cv2.CascadeClassifier(cascPath)
cam  			= cv2.VideoCapture(camera_path)

rospy.init_node('camera_node', anonymous=True)
face_detect_pub  	= rospy.Publisher('face_detect', Bool, queue_size = 10)
face_location_pub	= rospy.Publisher('face_location', String, queue_size = 10)

def camera_handler():
	while not rospy.is_shutdown():
		if not cam.isOpened():
			rospy.loginfo('Unable to load camera.')
			sleep(5)
			pass
		ret, frame 	= cam.read()
		
		#process image here
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		faces = faceCascade.detectMultiScale(
        		gray,
        		scaleFactor=1.1,
        		minNeighbors=5,
        		minSize=(30, 30)
    		)
		
		#fine nearest/largest face
		face_detected_flag 	= 0
		face_area_largest 	= 0
		face_index		= 0
		closest_face 		= 0
		closest_face_x 		= 0
		closest_face_y 		= 0
		closest_face_w 		= 0
 		closest_face_h 		= 0
		for (x, y, w, h) in faces:
			face_detected_flag 		= 1
			face_area 	 		= w * h
			if (face_area > face_area_largest):
				face_area_largest 	= face_area
				closest_face 		= face_index
				closest_face_x 		= x
				closest_face_y 		= y
				closest_face_w 		= w
				closest_face_h 		= h
			face_index 	= face_index + 1

		#publishing
		face_detect_pub.publish(face_detected_flag) #0 means no face, 1 means yes
		face_location_string 	= "%d,%d,%d,%d" % (closest_face_x, closest_face_y, closest_face_w, closest_face_h)
		face_location_pub.publish(face_location_string)
		###################

if __name__ == '__main__':
	try:
		camera_handler()
	except rospy.ROSInterruptException:
		cam.release()
		pass

