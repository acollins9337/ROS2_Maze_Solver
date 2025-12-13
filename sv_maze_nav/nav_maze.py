import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, Int32

class NavMaze(Node):
	'''
	Maze navigation node. This node uses information from the other nodes to make decisions. 
	Since the robot will only ever encounter a small set of scenarios, basic conditional logic was used
	rather than a more complicated decision tree or state machine.
	'''
	
	def __init__(self):
		super().__init__('navigator')

		self.skirt = .55 # Sensor skirt to determine appropriate distance for classification
		self.close = .2 # Skirt to determine when a wall is too close
		self.grid = .94 # Distance between waypoints, approximates the length of a wall segment

		# Preallocate global variables
		self.min_dist = 10.0
		self.min_angle = 0.0
		self.debug = 0.0
		self.align = math.radians(12.5)
		self.latest_prediction = None

		# Create relevant publishers and subscribers
		self._dist_sub = self.create_subscription(
			Pose2D,
			'/object_dist',
			self._dist_callback,
			10)
		
		self._wp_pub = self.create_publisher(
			Pose2D,
			'/maze_wp',
			10
		)

		self._sign_sub = self.create_subscription(
			Int32,
			'/sign_result',
			self._result_callback,
			10
		)

		self._done_sub = self.create_subscription(
			Bool,
			'/wp_done',
			self._done_callback,
			10
		)

		self.timer = self.create_timer(.1, self._state_callback)
		
	def _dist_callback(self, d:Pose2D):
		'''
		Store values for distance to closest object, and angle to the ray that points to that object

		Args:
			d (Pose2D): .x field contains closest distance obtained from sensors.py
						.theta field contains angle to closest point
						.y contains a variable used for debug
		'''
		self.min_dist = d.x
		self.min_angle = d.theta
		self.debug = d.y

	def _result_callback(self, ret:Int32):
		'''
		Obtain the predictions from the KNN model then store them as the latest prediction
		
		Args:
			ret (Int32): .data field contains the label of the prediction of the KNN model
		'''
		if ret.data == -1:
			self.latest_prediction = None
		else:
			self.latest_prediction = ret.data

	def _done_callback(self, msg:Bool):
		'''
		Print messages and set a goal status when a goal is finished. Used primarily for debugging

		Args:
			msg (Bool): A true or false variable message showing whether a goal has completed
		'''
		if msg.data:
			self.get_logger().info("Nav2 Done -- Resuming State Machine")
			self.waiting_for_goal = False

	def _state_callback(self):
		'''
		Primary decision making node. Uses data from other nodes then determines which course of action to take. Roughly:
			No wall in front of robot -> Travel to forward waypoint
			Wall in front of robot -> Access image classification -> Turn based on classified sign
			Wall obstructs path but too far to classify -> Move closer until a better classification is available
			Wall in appropriate range but classification still poor -> Turn left
		'''
		
		wp = Pose2D()
		
		# Dont take any action if the model or transform buffer are still loading
		if self.latest_prediction is None:
			self.get_logger().info("No Prediction Yet - Bypassing")
			return
		# Otherwise, if the wall is within range, make a decision
		elif self.min_dist <= self.skirt:
			# If alignment is poor, attempt realignment to get a better classification
			if abs(self.min_angle) > self.align:
				self.get_logger().info("Aligning to Wall")
				wp.x = 0.0
				wp.y = 0.0
				wp.theta = 3*self.min_angle/4
				self._wp_pub.publish(wp)
				return
			
			prediction = self.latest_prediction

			# Make a simple decision based on the prediction
			if prediction == 1:
				self.get_logger().info(f"[WALL SEEN] Prediction = {prediction} --> Turning Left")

				wp.x = 0.0
				wp.y = 0.0
				wp.theta = math.pi/2 - math.pi/55
			elif prediction == 2:
				self.get_logger().info(f"[WALL SEEN] Prediction = {prediction} --> Turning Right")

				wp.x = 0.0
				wp.y = 0.0
				wp.theta = -math.pi/2 + math.pi/58
			elif prediction == 3 or prediction == 4:
				self.get_logger().info(f"[WALL SEEN] Prediction = {prediction} --> U-Turning")
				
				wp.x = 0.0 #
				wp.y = 0.0
				wp.theta = math.pi - math.pi/55
			elif prediction == 0:
				self.get_logger().info(f"[WALL SEEN] Prediction = {prediction} --> Turning")

				wp.x = 0.0
				wp.y = 0.0
				wp.theta = math.pi/2 - math.pi/55
			elif prediction == 5:
				self.get_logger().info(f"[WALL SEEN] Prediction = {prediction} --> Goal Reached Yeahhh!")

				wp.x = 0.0
				wp.y = 0.0
				wp.theta = 0.0
		# Get closer to the wall if farther away than expected
		elif self.min_dist > self.skirt and self.min_dist <= self.grid:
			wp.x = self.grid/4
			wp.y = 0.0
			wp.theta = 0.0	
		# If no other condition is met, go forward			
		else:
			wp.x = self.grid
			wp.y = 0.0
			wp.theta = 0.0

		self._wp_pub.publish(wp)

def main():
	rclpy.init()
	rclpy.spin(NavMaze())
	rclpy.shutdown()

if __name__ == '__main__':
	main()
