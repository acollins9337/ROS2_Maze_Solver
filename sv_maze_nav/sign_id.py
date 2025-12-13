import rclpy
import cv2
import numpy as np
import statistics as st
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from skimage.feature import hog


class SignID(Node):
	'''
	Returns classifications for signs as seen by the Turtlebot's Pycam
	'''
	def __init__(self):
		super().__init__('sign_id')

		#Set up QoS Profiles for passing images over WiFi
		image_qos_profile = QoSProfile(depth=5)
		image_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
		image_qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
		image_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 

		self._video_subscriber = self.create_subscription(
			CompressedImage,
			'/image_raw/compressed',
			self._image_callback,
			image_qos_profile)

		self._result_publisher = self.create_publisher(
			Int32,
			"/sign_result",
			10
		)

		self.model_path = "PATH TO MODEL"
		self.model_ready = False

		self.img = None
		self.read_busy = False
		self.latest = None

		self.timer = self.create_timer(0.1, self._loading)

	def _loading(self):
		'''
		Create a callback that continuously checks if the model is loading.
		If the model is loading, this callback will pass
		'''
		if self.model_ready == True:
			return
		else:
			# Load the model from the expected path
			self.model = cv2.ml.KNearest_load(self.model_path)
			self.get_logger().info("\n Model Loaded Successfully!")
			self.model_ready = True

	def _image_callback(self, img:CompressedImage):
		'''
		Processes Pycam images, and publishes predictions

		Args:
			img (CompressedImage): Pycam image data as compressed image for processing
		'''
		# Publish a wait message if the model is loading and pass
		if self.model_ready == False:
			msg = Int32()
			msg.data = -1
			self._result_publisher.publish(msg)
			return
		# Make a prediction
		else:
			np_arr = np.frombuffer(img.data, np.uint8)
			image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

			# Run prediction algorithm five times, and return the most common prediction
			# This acts as a simplification of Bayesian Probability, and only works for high accuracy models
			prediction_list = []
			self.read_busy = True
			for i in range(5):
				p = self._make_prediction(image)
				prediction_list.append(int(p))
				if i == 4:
					self.read_busy = False

			predict_msg = Int32()
			prediction = st.mode(prediction_list)
			predict_msg.data = prediction
			self.latest = prediction

			self._result_publisher.publish(predict_msg)

	def _make_prediction(self, img):
		'''
		Use a Histogram of Oriented Gradients and a preloaded KNN classifier to predict the class of a single image

		Args:
			img (np_arr): A color image as an np_arr
		Returns:
			prediction (int): The predicted class of the image
		'''
		img_height = 64
		img_width = 64

		image = cv2.resize(img,(img_width,img_height))

		# Initial Image processing
		kblur=1
		blur=cv2.GaussianBlur(image,(kblur,kblur),0)
		gray=cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY)
		hsv=cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)

		### Implement some variables for colors to make masking easier ###
		#   Red range
		lower_red1 = np.array([0, 85, 75])
		upper_red1 = np.array([5, 255, 255])
		lower_red2 = np.array([170, 90, 80])
		upper_red2 = np.array([180, 255, 255])
		#   Green range
		lower_green = np.array([30, 15, 60])
		upper_green = np.array([100, 255, 255])
		#   Blue range
		lower_blue = np.array([101, 50, 50])
		upper_blue = np.array([130, 255, 255])

		# Create masks
		mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
		mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
		mask_green = cv2.inRange(hsv, lower_green, upper_green)
		mask_blue  = cv2.inRange(hsv, lower_blue, upper_blue)

		# Apply masks and image processing techniques
		full_mask = cv2.bitwise_or(cv2.bitwise_or(mask_red1,mask_red2),cv2.bitwise_or(mask_green,mask_blue))
		colored_objects = cv2.bitwise_and(gray,gray,mask=full_mask)
		ret,thresh=cv2.threshold(colored_objects,55,255,cv2.THRESH_BINARY)
		contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		m=5 # Margin in pixels so crop does not go directly to edges of object
		if len(contours) > 0:
			min_area=0.03*img_height*img_width
			large_contours = [
				c for c in contours 
				if cv2.contourArea(c) > min_area 
			]
			if len(large_contours) == 1:
				largest_contour=max(large_contours,key=cv2.contourArea)
				x, y , w, h = cv2.boundingRect(largest_contour)
				if x-m <=0 or y-m <=0:
					cropped = cv2.resize(gray[y:y+h, x:x+w], (img_width,img_height))
				else:
					cropped = cv2.resize(gray[y-m:y+h+m, x-m:x+w+m], (img_width,img_height))
			else:
				cropped=thresh
		else:
			cropped = thresh

		# Apply HOG Methods
		gray_float=cropped.astype(np.float32)
		features, hog_image = hog(
			gray_float,
			orientations=12,
			pixels_per_cell=(10, 10),
			cells_per_block=(4, 4),
			block_norm="L2-Hys", # Improved a little bit
			visualize=True
		)
		processed=features.flatten().reshape(1,-1).astype(np.float32)
		ret, results, neighbours, dist = self.model.findNearest(processed, k=5)
		prediction = int(ret)

		return prediction
	
def main():
	rclpy.init()
	rclpy.spin(SignID())
	rclpy.shutdown()

if __name__ == '__main__':
	main()