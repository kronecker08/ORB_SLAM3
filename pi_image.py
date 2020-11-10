#!/usr/bin/env python
#Jai Shri Shyam
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
from imutils.video import VideoStream
import imutils

bridge = CvBridge()
frameSize = (640,480)
if __name__ == '__main__':

	vs = VideoStream(src=0, usePiCamera=usingPiCamera, resolution=frameSize,
		framerate=32).start()
	rospy.init_node('Cam', anonymous=True)
	rate = rospy.Rate(rospy.get_param('~hz', 1000))
	rospy.sleep(1)
	imgpub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
	
	while True:
	    # Capture frame-by-frame
	    frame = vs.read()

	    if frame.shape[0]>640:
	        frame = cv2.resize(frame, (640, 480))
	    image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
	    imgpub.publish(image_message)

	    # Display the resulting frame
	    cv2.imshow('Frame',frame)
	    # Press ESC on keyboard to exit

	# When everything done, release the video capture object
	cap.release()

	# Closes all the frames
	cv2.destroyAllWindows()


