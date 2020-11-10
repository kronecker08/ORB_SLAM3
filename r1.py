#!/usr/bin/env python
#Jai Shri Shyam
import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
from imutils.video import VideoStream
import imutils
usingPiCamera = True
bridge = CvBridge()
frameSize = (320,240)
if __name__ == '__main__':
		timeout=60
		time_now = time.time()
		vs = VideoStream(src=0, usePiCamera=usingPiCamera, resolution=frameSize,
				framerate=32).start()
		#Allow camera to warm up.
		time.sleep(2.0)
		rospy.init_node('Cam', anonymous=True)
		rate = rospy.Rate(20)
		rate.sleep()
		imgpub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)

		while True:
			time_check = time.time()
			# Capture frame-by-frame
			if (time_now - time_check) > timeout:
				break
			frame = vs.read()

			if frame.shape[0]>320:
				frame = cv2.resize(frame, (320, 240))
			image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
			imgpub.publish(image_message)

			# Display the resulting frame

			cv2.imshow('Frame',frame)
			
			# Press ESC on keyboard to exit
			key = cv2.waitKey(1) & 0xFF
		# When everything done, release the video capture object
			if key == ord("q"):
				break
			

		#cap.release()

		# Closes all the frames
		cv2.destroyAllWindows()
		rospy.shutdown()
		vs.stop()


