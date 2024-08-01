import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from beswarm_action_interfaces.action import Beswarm

#mod
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

class BeswarmActionClient(Node):
    def __init__(self):
        super().__init__('beswarm_action_client')
        self._action_client = ActionClient(self, Beswarm, 'beswarm')
   	 
    	#mod
  	#  self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

	#mod
   # def odom_callback(self, msg):
	#	self.x_act = msg.pose.pose.position.x
 	#   self.y_act = msg.pose.pose.position.y
	#	print ("Message received: ", self.x_act)
    
    def send_goal(self, client_id, evo):
    	goal_msg = Beswarm.Goal()
    	goal_msg.client_id = client_id  #send 'client_id' variable to server
    	goal_msg.evo = evo

    	self._action_client.wait_for_server()

    	self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    	self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
    	goal_handle = future.result()
    	if not goal_handle.accepted:
        	self.get_logger().info('Goal rejected :(')
        	return

    	self.get_logger().info('Goal accepted :)')

    	self._get_result_future = goal_handle.get_result_async()
    	self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
    	result = future.result().result
    	self.get_logger().info('End result: X:{0} Y:{1} Z:{2}'.format(result.x,result.y,result.z)) #final answer from server
    	rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
    	feedback = feedback_msg.feedback
    	self.get_logger().info('Partial received feedback: X:{0} Y:{1} Z:{2} Fit:{3}'.format(feedback.px,feedback.py,feedback.pz,feedback.pfit))


def main(argv):
    rclpy.init(args=None)
    if len(argv) < 2:
        print('beswarm_action_client <NUM_PARTICLES> <EVO>')
        print('EVO: 1-Sphere function; 2-Rastrigin; 3-Ackley; 4-Adjiman')
   	 
        return

	#for i in argv:
	#	print(i)

    NUM_PARTICLES = int(argv[0])
    EVO = int(argv[1])
   	 
    action_client = BeswarmActionClient()

    action_client.send_goal(NUM_PARTICLES,EVO)

    rclpy.spin(action_client)


if __name__ == '__main__':
	main(sys.argv[1:])
