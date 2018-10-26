import rospy
from random import randint
from utils import *
import numpy as np
from ROSNode import ROSNode



class Env:
	def __init__(self,length,height,Nstep):
		self.ROSNode=None
		self.L=length
		self.H=height
		
		self.local_position = None
		self.actions=[(0,-1),(0,1),(-1,0),(1,0)]
		self.Nstep=Nstep
		self.gridscale=1.0*length/Nstep
		self.limitbox=[-length/2.0-self.gridscale/2.0-self.gridscale*(int(1.5/self.gridscale)),
						length/2.0+self.gridscale/2.0+self.gridscale*(int(1.5/self.gridscale)),
						-length/2.0-self.gridscale/2.0-self.gridscale*(int(1.5/self.gridscale)),
						length/2.0+self.gridscale/2.0+self.gridscale*(int(1.5/self.gridscale)),
						self.H-1,self.H+1]
		print(self.limitbox)
		
	def wait(self):
		self.ROSNode.hover(1)

		
	def reset(self):
		self.ROSNode=ROSNode()
		self.ROSNode.start()
		self.wait()
		_,s=self.get_state()
		return s

	def get_state_xy(self,x,y):

		x_st=int((x-self.limitbox[0])/self.gridscale)
		y_st=int((y-self.limitbox[2])/self.gridscale)

		return [x_st,y_st]	

	def get_state(self):
		done=False
		x,y,z=self.ROSNode.get_uav_pos()
		# print(x,y,z,self.limitbox)
		if x<self.limitbox[0] or x>self.limitbox[1] or y<self.limitbox[2] or y>self.limitbox[3] or z<self.limitbox[4] or z>self.limitbox[5]:
			done=True
			return done,[-1,-1]


		return done, self.get_state_xy(x,y)

	def calculate_reward(self,s_old,s_new):
		length=self.L
		square_xdown_order=int((-length/2.0-self.limitbox[0])/self.gridscale)
		square_ydown_order=int((-length/2.0-self.limitbox[2])/self.gridscale)
		square_xup_order=int((length/2.0-self.limitbox[0])/self.gridscale)
		square_yup_order=int((length/2.0-self.limitbox[2])/self.gridscale)
		#x walking on x-axis
		#give reward only when ccw
		dx=s_new[0]-s_old[0]
		dy=s_new[1]-s_old[1]
		reward=0

		# print(square_xdown_order,square_xup_order,square_ydown_order,square_yup_order)

		if square_ydown_order<=s_old[1] and square_yup_order>=s_old[1]:
			#y:large->small x:same
			if square_xdown_order == s_old[0]:
				if dx==0 and dy<0:
					reward=1.01
				else:
					reward=-1.5
			#y:small->large x:same
			if square_xup_order == s_old[0]:
				if dx==0 and dy>0:
					reward=1.02
				else:
					reward=-1.5

		if square_xdown_order<=s_old[0] and square_xup_order>=s_old[0]:
			#x:small->large y:same
			if square_ydown_order == s_old[1]:
				if dx>0 and dy==0:
					reward=1.03
				else:
					reward=-1.5
			#x:large->small y:same
			if square_yup_order == s_old[1]:
				if dx<0 and dy==0:
					reward=1.04
				else:
					reward=-1.5
		if reward<1e-6 and reward>-1e-6:

			if square_ydown_order<=s_new[1] and square_yup_order>=s_new[1]:
				#x move
				if square_xdown_order == s_new[0]:
					if dx!=0:
						reward=1.05

				#x move
				if square_xup_order == s_new[0]:
					if dx!=0 :
						reward=1.06


			if square_xdown_order<=s_new[0] and square_xup_order>=s_new[0]:
				#y move
				if square_ydown_order == s_new[1]:
					if dy!=0:
						reward=1.07

				#y move
				if square_yup_order == s_new[1]:
					if dy!=0:
						reward=1.08

		if not (reward<1e-6 and reward>-1e-6):
			print('============reward get!================',reward)
		else:
			reward=-3

		return reward

	def step(self,act):

		done,state1= self.get_state()

		state2=state1

		if not done:
			px1=state1[0]*self.gridscale+self.gridscale/2.0+self.limitbox[0]
			py1=state1[1]*self.gridscale+self.gridscale/2.0+self.limitbox[2]
			tx=px1+self.actions[act][0]*self.gridscale
			ty=py1+self.actions[act][1]*self.gridscale

			# print(self.ROSNode.get_uav_pos(),tx,ty)

			self.ROSNode.moveto(tx,ty,self.H)
			self.ROSNode.ros_interact_step()

			done, state2=self.get_state()
			# reward=self.calculate_reward(state1,state2)
			# if done:
			# 	reward=-10
			# return state2, reward, done
			return state2, done
		else:
			print("error! game over!")
			# return state1, -10, done
			return state1, done
		



