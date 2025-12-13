import math
from rclpy.node import Node
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Quaternion, Pose2D
import tf2_ros


def quat_to_yaw(q):
    '''
    Convert quaternion to a yaw angle
    
    Args:
        q (Quaternion): Input quaternion to convert to a yaw
    '''
    return math.atan2(2.0 * (q.w * q.z),
                      1.0 - 2.0 * (q.z * q.z))

class MoveRobotClient(Node):
    '''
    This node controls the locomotion of the Turtlebot. It takes commands as waypoints from the NavMaze node, 
    and then sends those waypoints to ROS2 NAV2. This keeps the NAV2 interactions seperate from the state machine,
    improving readibility.
    '''
    def __init__(self):
        super().__init__('mover')
        self.get_logger().info('NavStack Started')
        
        self.start = True

        # TF buffer/listener shared for all goals
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._wp_sub = self.create_subscription(
            Pose2D,
            '/maze_wp',
            self._waypoint_callback,
            10
        )

        self._done_pub = self.create_publisher(
            Bool,
            '/wp_done',
            10
        )

    def _waypoint_callback(self, p: Pose2D):
        '''
        Listens for waypoint commands and stores the latest in a global variable
        
        Args:
            p (Pose2D): Goal waypoint on the local map
        '''
        self.local_wpx = p.x
        self.local_wpy = p.y
        self.local_wptheta = p.theta

        if self.start:
            self.start = False
            self.send_goal()

    
    def send_goal(self):
        '''
        Send a goal to ROS2 NAV2 for completion. 
        '''

        # Improved timekeeping with a timer, prevents robot from entering a dead state while waiting
        if hasattr(self, 'timer'):
            self.timer.cancel()
            del self.timer

        self.get_logger().info("GOAL SENT")
        if self.local_wpx > 0.50:
            self.get_logger().info(f"Going Forward 1 Grid Cell")
        elif self.local_wpx < 0.50 and self.local_wpx > 0.05:
            self.get_logger().info(f"Going Forward a little bit")
        else:
            pass

        # Variables for local waypoint
        x = self.local_wpx
        y = self.local_wpy
        theta = self.local_wptheta

        # Create variable to pass to transform buffer
        goal_waypoint = PoseStamped()
        goal_waypoint.header.frame_id = 'base_link'
        goal_waypoint.header.stamp = Time().to_msg()
        goal_waypoint.pose.position.x = float(x)
        goal_waypoint.pose.position.y = float(y)

        q_goal = Quaternion()
        q_goal.z = math.sin(theta / 2.0)
        q_goal.w = math.cos(theta / 2.0)
        goal_waypoint.pose.orientation = q_goal

        # Attempt to transform the waypoint in the local system to the waypoint in the global system
        try:
            global_pose = self.tf_buffer.transform(goal_waypoint, 'map', timeout=Duration(seconds=1.0))
        except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as ex:

            self.get_logger().warn(
            f"[TF] transform {goal_waypoint.header.frame_id} -> map failed: {ex}")
            self.start = True
            return

        # Create a goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = global_pose

        # Send goal to NAV2
        self._action_client.wait_for_server(timeout_sec=2.0)
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future: rclpy.task.Future):
        '''
        Defines actions to take when the sent goal is accepted
        
        Args:
            future (rclpy.task.Future): The future goal sent to NAV2 waiting on a response
        '''

        # If goal is rejected, give an error
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 Goal Rejected")
            return
        # Otherwise say goal was accepted
        self.get_logger().info("Nav2 Goal Accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_response_callback)

    def _result_response_callback(self):
        '''
        Defines actions to take when a future goal is completed
        '''

        # Send a log message, and wait five seconds to prevent new goals from sending too quickly
        self.get_logger().info("Nav2 Goal Reached")
        self.timer = self.create_timer(5.0, self.send_goal)

        msg = Bool()
        msg.data = True
        self._done_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(MoveRobotClient())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
