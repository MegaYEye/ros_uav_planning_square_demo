from environment import Env
from QLearningAgent_Approx import QLearningAgent_Approx
from replay_buffer import ReplayBuffer
import numpy as np
import time
class LearningController:
	def __init__(self):
		self.env=None
		self.agent=None

	def one_episode(self,t_max=1000,replay=None,replay_batch_size=32):
		# print("episode start!")
		total_reward = 0.0

		s = self.env.reset()



		watcher=-1
		last_t=0
		a=-1
    
		for t in range(t_max):
	        # get agent to pick action given state s.
			
			
			
			if a<0:
				a = self.agent.get_action(np.array([s]))
			next_s, done  = self.env.step(a)
			start = time.time()
	        # train (update) agent for state s
	#         <YOUR CODE HERE>
			#done may sometimes cause error
			if done:
				self.agent.update(np.array([s]),np.array([a]),np.array([-50]),np.array([s]))
			if (not done) and (str(next_s)!=str(s)):
				r=self.env.calculate_reward(s,next_s)
	
				self.agent.update(np.array([s]),np.array([a]),np.array([r]),np.array([next_s]))

				
				# print("update")
				last_t=t
				total_reward +=r
				qt=[]
				qt.append(self.agent.get_all_qvalue(np.array([s])))
				qtn=[]
				qtn.append(self.agent.get_all_qvalue(np.array([next_s])))				
###########################replay buffer############################
				if replay is not None:

					replay.add(s, a, r, next_s, done)
		            
		  #           # sample replay_batch_size random transitions from replay, 
		  #           # then update agent on each of them in a loop
					s_batch, a_batch, r_batch, next_s_batch, done_batch = replay.sample(replay_batch_size)

					
					self.agent.update(np.array(s_batch), np.array(a_batch), np.array(r_batch), np.array(next_s_batch))
				end=time.time()

				print("old_st:",s,"new_st:",next_s,"act:",a,"reward:",r,"time:",time.time()-start)
				print("q_table_s:",qt)
				print("q_table_sn:",qtn)
				print()
###########################replay buffer############################
				a = self.agent.get_action(np.array([next_s]))
			s = next_s
			
			# if t%5==0:
			# 	print(s,next_s,a,total_reward,done)
			watcher=t

			if done: break
		print("episode end at step: ",watcher)
	        
		return total_reward		

	def start(self):
		self.env=Env(length=10,height=2,Nstep=10)

		self.agent = QLearningAgent_Approx(alpha=0.5, epsilon=0.2, discount=0.9, 
			get_legal_actions = lambda s: range(len(self.env.actions)),state_dim=(2,),action_dim=len(self.env.actions))

		self.agent.load_param("q_approx_model_503232.ckpt")
		# print("================Qtable:================")
		# for m in self.agent._qvalues:
		# 	print(m,self.agent._qvalues[m])

		rewards = []
		replay = ReplayBuffer(1000)
		for i in range(500):
			
			rewards.append(self.one_episode(2000,replay=replay))   
			
			#OPTIONAL YOUR CODE: adjust epsilon
			self.agent.epsilon *= 0.9999
			print("+++++++++++++++++++++++++++++++++++++++++++++++")
			print("episode = ",i,'eps =', self.agent.epsilon, 'mean reward =', np.mean(rewards[-10:]))
			print("+++++++++++++++++++++++++++++++++++++++++++++++")
			# if i %2 ==0:

			self.agent.save_param("q_approx_model_503232.ckpt")
			self.agent.save_param("q_approx_bak.ckpt")

if __name__ == '__main__':
	ctrl = LearningController()
	ctrl.start()