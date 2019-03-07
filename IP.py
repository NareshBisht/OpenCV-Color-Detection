#!/usr/bin/env python

###########################################################################
# Author List : Naresh bisht
# Functions : image_callback, FindContours, FindContourColour 
############################################################################

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32

class Colour_Detection:
	
	def __init__(self):

		#A rosnode named 'ros_bridge' is initilised
		rospy.init_node('ros_bridge')
		
		# Create a ROS Bridge
		self.ros_bridge = cv_bridge.CvBridge()

		# Subscribing to /usb_cam/image_rect_color topic to get the image from calibrated usb camera.
		self.image_sub = rospy.Subscriber('whycon/image_out', Image, self.image_callback)

		self.Red_Count = 0   #the number of red plant pollinated
		self.Green_Count = 0  #the number of green plant pollinated
		self.Blue_Count = 0  #the number of blue plant pollinated
	
##########################################################################################################
	# Function Name: image_callback
	# Input: msg
	# Output: None
	# Logic: To get the image from calibrated usb camera and calling necessery function to draw contour around led and led color detection
	
	# Example Call: it is a callback function of /usb_cam/image_rect_color (image_callback(msg))
##########################################################################################################

	def image_callback(self,msg):

		# converting msg from Image type to cv2 (matrix) type, so that we can perform openCV operations on image for proccessing
		self.img = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

		# cv2.imshow('Contour_Led', self.img)		#Displaying image having contour drawn around LEDs with Same color as that of leds
		cv2.waitKey(1)

		#Finding all LED(contours) in image 
		Found_contours = self.FindContours()

		#by Found_contour methord we found all the LEDs(contours) now we have to detect the colour of LEDs, we do that by passing found_countour in FindContourColour methord

		#Finding LEDs color and drawing contour of same colour around LEDs  	
		self.FindContoursColour(Found_contours)

		


##########################################################################################################
	# Function Name: FindContours
	# Input: None
	# Output: contours -> (list of numpy)

	# Aim : To find LEDs(contour) in recived image.

	# Logic: 1) To find LEDs(contour) we first mask image so that all unnecessary object/geometry are removed and only very bright
	#           or nearly white pixels remain in masked image as the led when glow because of high intensity appear almost white in image so if we mask for white 
	#           pixel we will be able to detect led.

  #		      2) since flex sheet is also white and reflective the masked image will contain some noise(unwanted small white pixels) so we erode the image to    
  #		      to remove these noise.

	#         3) after erosion we find contour so the we get only LEDs(led contour) and then we return them using return function

	# Example Call: it is a callback function of /usb_cam/image_rect_color (image_callback(msg))
##########################################################################################################

	def FindContours(self):
		
		#Masking recieved image to remove unncessery object/geometry from image. 
		lowest_pixel_value_for_mask = np.array([232,232,232])    #setting lowest pixel value for masking
		higest_pixel_value_for_mask = np.array([255,255,255])	 #setting highets pixel value for masking
		masked_image  = cv2.inRange(self.img, lowest_pixel_value_for_mask, higest_pixel_value_for_mask)  #masking the image

		#since flex sheet is white and reflective the masked image will contain some noise(unwanted small white pixels) so we erode the image to    
		#to remove these noise.

		#eroding the image to remove small nosie from masked image
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))  ########perform erosion after masking
		eroded_image = cv2.erode(masked_image,kernel,iterations = 1)

		# #Finding contour in eroded image so that only LEDs are detected in  image
		abc, contours, hierarchy = cv2.findContours(eroded_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)	

		return contours


##########################################################################################################
	# Function Name: FindContoursColour
	# Input: (countours) list of numpy.
	# Output: None

	# Aim: To draw contours of same colour as that of LEDs around LEDs

	# Logic : 1) To find the diffrent colours with which the diffrent contours must be drawn we have to check the pixels BGR value around detected contours(LEDs)  
	#            to find the colour of LEDs hence the colour with which that contour must be drawn
	# 
	# Procedure : 1)Traverse the list of contours one by one using for loop
	#             2)for each contour in the list of contours find the centroid using moments function.
	#             3)traverse all pixels around centroid using nested for loops. see below written nested foor loop for better understanding
	#			  4)while traversing each pixel around the centroid, find the number of pixels that has higher value of B than G and R in BGR and store it in a
	#               variable say BlueFrequency do similarly find pixels which has higher value of G than B and R, similarly find pixels with R greater than B and G
	#				and store them in variable say GreenFrequency and RedFrequency repectively
	#             5)now find the largest of these 3 varables if GreenFrequency is largest that means most of the pixels around centroid of the contour
	#				are green hence the led is green and contour has to be drawn with green colour. 
	#			  6)similary draw rectangle around centorid using cv2.rectangle function.  
	#
	# Example Call: it is a callback function of /usb_cam/image_rect_color (image_callback(msg))
##########################################################################################################
	
	def FindContoursColour(self,Found_contours):

		self.Red_Count = 0
		self.Green_Count = 0
		self.Blue_Count = 0
		
		#Traversing all contours in list of found contours
		for contour in Found_contours:
			
			#finding centroid of the contour
			Moment = cv2.moments(contour)
			if Moment["m00"] != 0:
				cx = int(Moment['m10']/Moment['m00']) #the x coordiante of the centroid 
				cy = int(Moment['m01']/Moment['m00']) #the y coordiante of the centroid 
			else:
				return

			Pixels_with_higher_B_value = 0 # It stores the Number of pixels (around the centeroid of a contour) which have higher value of B part than G, R 
			Pixels_with_higher_G_value = 0 # It stores the Number of pixels (around the centeroid of a contour) which have higher value of G part than B, R 
			Pixels_with_higher_R_value = 0 # It stores the Number of pixels (around the centeroid of a contour) which have higher value of R part than B, G 

			#In order to find the light of led we traverse all the pixels around the centoriod of the LED(contour)
			#for traversing around LED  we traverse all values between (cy-25,cy + 25) and (cx -25, cx + 25) using nested loop

			Range = 8  #this much pixel length and width to traverse left/right, below/above centroid

			for i in range(cy - Range, cy + Range):			
				for j in range(cx - Range,cx + Range):

					

					B = self.img[i,j][0]    #B is the Blue value of the pixel that is currently traversed.
					G = self.img[i,j][1]	#G is the Green value of the pixel that is currently traversed.
					R = self.img[i,j][2]	#R is the Red value of the pixel that is currently traversed.

					if (B >= G) and (B >= R):  #if B value is larger then G and R value then we increment the number of pixels having larger B value by 1
					   Pixels_with_higher_B_value += 1			#incrementing pixels with larger Blue value by 1
					elif (G >= B) and (G >= R):#if G value is larger then B and R value then we increment the number of pixels having larger G value by 1 
					   Pixels_with_higher_G_value += 1			#incrementing pixels with larger Green value by 1
					else:					   #if R value is larger then B and G value then we increment the number of pixels having larger H value by 1 
					   Pixels_with_higher_R_value += 1	   		#incrementing pixels with larger Red value by 1

			
			#if most of the pixels around centroid have larger blue value the led is blue and we draw conntour with blue colour
			if (Pixels_with_higher_B_value >= Pixels_with_higher_G_value) and (Pixels_with_higher_B_value >= Pixels_with_higher_R_value):  
			   
			   self.Blue_Count = self.Blue_Count + 1			
			   cv2.rectangle(self.img,(cx-15,cy+15),(cx + 15,cy -15),(255,0,0),3)
			
			elif (Pixels_with_higher_G_value >= Pixels_with_higher_B_value) and (Pixels_with_higher_G_value >= Pixels_with_higher_R_value): #if most of the pixels around centroid have larger Green value the led is Green and we draw conntour with Green colour			   
			   
			   self.Green_Count = self.Green_Count + 1
			   cv2.rectangle(self.img,(cx-15,cy+15),(cx + 15,cy -15),(0,255,0),3)
			
			else:   			
			   
			   self.Red_Count = self.Red_Count + 1
			   cv2.rectangle(self.img,(cx-15,cy+15),(cx + 15,cy -15),(0,0,255),3)
	
			print "str(self.Red_Count) + " Red " + str(self.Green_Count) + " Green " + str(self.Blue_Count) + " Blue." 
   			self.Detect_only_Once = 0
		
		cv2.imshow('Contour_Led', self.img)		#Displaying image having contour drawn around LEDs with Same color as that of leds


		
if __name__ == '__main__':
	Colour_Detection_obj = Colour_Detection()  	#creating object of class Colour_Detection and calling its constructor
	rospy.spin()


#Star this repository if u like this script.
