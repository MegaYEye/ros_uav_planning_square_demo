import os
import subprocess
import rospy
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Vector3Stamped, Pose
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header
from utils import *
# from geometry_msgs.msg import Twist, Vector3Stamped, Pose
# from mavros_msgs.srv import CommandBool, SetMode
# from environment import Env
# from QLearningAgent import QLearningAgent
"""

gazebo----ENV: -----------learning
			|s,r               |
		  statemachine-------s,r,a--


"""

class ROSNode:
	"""StateMachine: flight routinue control"""
	def __init__(self):

		self.rate=20
		self.rateobj = None
		self.set_arm=None
		self.set_mode=None
		self.L=1
		self.H=2	
		self.local_pos_sub = None
		
		self.set_arm= None
		self.set_mode = None
		self.pos_setpoint_pub = None

	def ros_delay(self,sec):

		for i in range(int(sec*self.rate+0.5)):
			# rospy.loginfo("delaying" )
			self.rateobj.sleep()
	def ros_interact_step(self):
		self.rateobj.sleep()

		
	def start_sim(self):
		self.err_cmd=0

		mavros = subprocess.Popen([r'roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" > /dev/null'],shell=True)
		self.ros_delay(1)
		gazebo = subprocess.Popen([r"cd ~/src/Firmware/ && HEADLESS=1 make posix_sitl_default gazebo > /dev/null"],shell=True)
		# offb = subprocess.Popen([r'roslaunch offb offb_local.launch'],shell=True)
		# rviz = subprocess.Popen([r'rosrun rviz rviz'],shell=True)
		rospy.loginfo("start sim!")

	def stop_sim(self):
		subprocess.call([r'pkill mavros'],shell=True)
		subprocess.call([r'pkill px4'],shell=True)
		subprocess.call([r'pkill rviz'],shell=True)
		subprocess.call([r'pkill gzserver'],shell=True)	

	def reset_sim(self):
		rospy.loginfo("============reset simulation===============")
		self.stop_sim()
		self.start_sim()

	def moveto(self,x,y,z):
		pos = PoseStamped()
		pos.pose.position.x = x
		pos.pose.position.y = y
		pos.pose.position.z = z
		pos.header = Header()
		pos.header.frame_id = "map"
		pos.header.stamp = rospy.Time.now()
		# rospy.loginfo(pos)
		self.pos_setpoint_pub.publish(pos)

	def local_position_callback(self,data):
		self.local_position = data	

	def get_uav_pos(self):
		rx=self.local_position.pose.position.x
		ry=self.local_position.pose.position.y
		rz=self.local_position.pose.position.z
		return rx,ry,rz	

	def hover(self, sec):
		for i in range(int(sec/self.rate)):
			self.moveto(0,0,self.H)
			ros_interact_step()

	def flight_init(self):
		#https://github.com/ethz-asl/rotors_simulator/blob/master/rqt_rotors/src/rqt_rotors/hil_plugin.py
		#https://github.com/PX4/Firmware/blob/master/integrationtests/python_src/px4_it/mavros/mavros_offboard_posctl_test.py
		#https://github.com/PX4/Firmware/blob/master/integrationtests/python_src/px4_it/mavros/mavros_test_common.py
		rospy.loginfo("============flight_init===============")
		for i in range(5):

			try:
		
				for i in range(10):
					self.moveto(0,0,self.H)
	
				self.ros_delay(0.1)
				res = self.set_arm(True)
				if not res.success:
					raise rospy.ServiceException("failed to send ARM command")
					continue
				res = self.set_mode(0,"OFFBOARD")
				if not res.mode_sent:
					raise rospy.ServiceException("failed to send OFFBOARD command")
					continue

			except rospy.ServiceException as e:
				rospy.logerr(e)
				continue
			return True

		
			
		return False




	def start(self):
		
		#references: 
		rospy.init_node('ROSNode')
		rospy.loginfo("ROSNode started!")
		self.rateobj=rospy.Rate(self.rate)
		self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', 
												PoseStamped,
												self.local_position_callback)
		
		self.set_arm=rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
		self.local_position=None


		state = 0

		while not rospy.is_shutdown():

			if state==0:
				self.reset_sim()
				self.ros_delay(5)
				res = self.flight_init()
				if res:
					rospy.loginfo("flight init ok!")
					state = 1
				else:
					state=0 #do nothing
					# self.reset_sim()
					# self.ros_delay(5)
			if state==1:
				# self.Env.moveto(0,0,self.H)

				cnt=0
				print("taking off....")
				while True:
					try:
						rx,ry,rz=self.get_uav_pos()
					except:
						cnt+=1
						continue
						
					if get_dis(rx,ry,rz,0,0,self.H)<0.1:
						state=99
						break
					else:
						cnt+=1
					# print("taking off....",[rx,ry,rz])
					self.moveto(0,0,self.H)
					self.ros_delay(0.1)

					if cnt>=200:
						state=0
						break

			if state==99:
				rospy.loginfo("============take off ok!============")
				return True
	

			self.rateobj.sleep()

 

	def test(self):
		self.reset_sim()
		self.ros_delay(3)
		self.stop_sim()













# if __name__ == '__main__':
# 	s=ROSNode()
# 	s.start()
	
