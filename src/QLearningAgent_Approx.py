
from collections import defaultdict
import random, math
import numpy as np
import pickle
from functools import partial
import h5py
import tensorflow as tf
import keras
import keras.layers as L
from keras.models import load_model
class QLearningAgent_Approx:
    def __init__(self, alpha, epsilon, discount, get_legal_actions,state_dim,action_dim):
        self.get_legal_actions = get_legal_actions
        #self._qvalues = defaultdict(lambda: defaultdict(lambda: 0))
        # self._qvalues = defaultdict(partial(defaultdict,int))
        self.alpha = alpha
        self.epsilon = epsilon
        self.discount = discount
        self.state_dim=state_dim
        self.action_dim=action_dim

        tf.reset_default_graph()
        self.sess = tf.InteractiveSession()
        keras.backend.set_session(self.sess)
        self.network=self.create_network()


      


    def create_network(self):
        self.network = keras.models.Sequential()
        self.network.add(L.InputLayer(self.state_dim))

        # let's create a network for approximate q-learning following guidelines above
        # <YOUR CODE: stack more layers!!!1 >
        self.network.add(L.Dense(50,activation='relu'))
        self.network.add(L.Dense(70,activation='relu'))
        self.network.add(L.Dense(70,activation='relu'))
        self.network.add(L.Dense(100,activation='relu'))
        self.network.add(L.Dense(self.action_dim,activation='linear'))

        self.states_ph = tf.placeholder('float32', shape=(None,) + self.state_dim,name="st")
        self.actions_ph = tf.placeholder('int32', shape=[None],name="ac")
        self.rewards_ph = tf.placeholder('float32', shape=[None],name="r")
        self.next_states_ph = tf.placeholder('float32', shape=(None,) + self.state_dim,name="nst")
        self.is_done_ph = tf.placeholder('bool', shape=[None],name="done")

        gamma = self.discount

    #         <YOUR CODE HERE>
            # Q= (1-learning_rate) * self.get_qvalue(state,action) + learning_rate * (reward+gamma*self.get_value(next_state))
            
            
            # self.set_qvalue(state, action, Q)
            #get q-values for all actions in current states
            # print(self.states_ph)
            # print(self.states_ph[None])
        predicted_qvalues = self.network(self.states_ph)


            #select q-values for chosen actions
        predicted_qvalues_for_actions = tf.reduce_sum(predicted_qvalues * tf.one_hot(self.actions_ph, self.action_dim), axis=1)
            # compute q-values for all actions in next states
            #predicted_next_qvalues = <YOUR CODE - apply network to get q-values for next_states_ph>
        predicted_next_qvalues = self.network(self.next_states_ph)
            # compute V*(next_states) using predicted next q-values
            # next_state_values = <YOUR CODE>
        next_state_values = tf.reduce_max(predicted_next_qvalues, axis=1)
            # compute "target q-values" for loss - it's what's inside square parentheses in the above formula.
            # target_qvalues_for_actions = <YOUR CODE>
        target_qvalues_for_actions = self.rewards_ph+gamma*next_state_values
            # at the last state we shall use simplified formula: Q(s,a) = r(s,a) since s' doesn't exist
            # target_qvalues_for_actions = tf.where(self.is_done_ph, self.rewards_ph, target_qvalues_for_actions)

        # target_q_array=tf.where(tf.one_hot(self.actions_ph, self.action_dim,on_value=True,off_value=False),
        #     target_qvalues_for_actions*tf.one_hot(self.actions_ph, self.action_dim),
        #     predicted_qvalues)

        # loss=tf.reduce_sum((tf.stop_gradient(target_q_array)-predicted_qvalues)**2)

        loss = (predicted_qvalues_for_actions - tf.stop_gradient(target_qvalues_for_actions)) ** 2
        loss = tf.reduce_mean(loss)

            # training function that resembles agent.update(state, action, reward, next_state) from tabular agent
        self.train_step = tf.train.AdamOptimizer(2e-4).minimize(loss) 
        self.watch=   target_qvalues_for_actions

        return self.network

    def get_all_qvalue(self, state):
        """ Returns Q(state,action) """
        # print(self.network.predict(state))
        # print(state,action)
        # print(self.network.predict(state))
        # print(state)
        # print(self.network.predict(state)[0])
        # print(self.network.predict(state)[0][action])
        # print("====")
        return self.network.predict(state)[0]

    def get_qvalue(self, state, action):
        """ Returns Q(state,action) """
        # print(self.network.predict(state))
        # print(state,action)
        # print(self.network.predict(state))
        # print(state)
        # print(self.network.predict(state)[0])
        # print(self.network.predict(state)[0][action])
        # print("====")
        return self.network.predict(state)[0][action]

    def set_qvalue(self,state,action,value):
        """ Sets the Qvalue for [state,action] to the given value """
        print("not implemented for approximate version!!!!")

    #---------------------START OF YOUR CODE---------------------#



    def update(self, state, action, reward, next_state):
        """
        You should do your Q-Value update here:
           Q(s,a) := (1 - alpha) * Q(s,a) + alpha * (r + gamma * V(s'))
        """

        #agent parameters
        # print(state.shape[0])

        self.sess.run([self.train_step],{
            self.states_ph: state, self.actions_ph: action, self.rewards_ph: reward, 
            self.next_states_ph: next_state, self.is_done_ph: np.array([False ]* state.shape[0])
        })

    def get_best_action(self, state):
        """
        Compute the best action to take in a state (using current q-values). 
        """
        possible_actions = self.get_legal_actions(state)

        #If there are no legal actions, return None
        if len(possible_actions) == 0:
            return None

#         <YOUR CODE HERE>
        value=None
        best_action=None
#         print(possible_actions)
        # for action in possible_actions:
        #     # print(state,np.array([action]))
        #     val=self.get_qvalue(state,np.array([action]))[0]

        #     if value is None:
        #         value=val
        #         best_action=action
        #     if value<val:
        #         value=val
        #         best_action=action
        best_action=np.argmax(self.get_all_qvalue(state))

        return best_action

    def get_action(self, state):
        """
        Compute the action to take in the current state, including exploration.  
        With probability self.epsilon, we should take a random action.
            otherwise - the best policy action (self.getPolicy).
        
        Note: To pick randomly from a list, use random.choice(list). 
              To pick True or False with a given probablity, generate uniform number in [0, 1]
              and compare it with your probability
        """

        # Pick Action

        
        ###YOUR CODE
        if random.random()<self.epsilon:
            chosen_action=np.random.choice(self.action_dim)
        else:
            chosen_action=self.get_best_action(state)
        return chosen_action

    def save_param(self, path):
        # self.network.save(path)
        try:
            saver = tf.train.Saver()
            saver.save(self.sess, "./"+path)
        except:
            print("save failure")
            
    def load_param(self, path):
        try:
            # self.network=self.create_network()
            saver = tf.train.Saver()
            saver.restore(self.sess, "./"+path)
        except:
     
            print("load param failure! let's start from a empty q table!")
            #self._qvalues = defaultdict(lambda: defaultdict(lambda: 0))
            self.network = self.create_network()

    def test(self):
        print(agent.get_qvalue(np.array([[1,2]]),np.array([0])))
        for i in range(20):
            print(self.get_action(np.array([[i,i]])))
        agent.update(np.array([[i,i]]),np.array([1]),np.array([10]),np.array([[i+1,i+1]]))
        agent.update(np.array([[i,i],[i,i]]),np.array([1,1]),np.array([10,10]),np.array([[i+1,i+1],[i+1,i+1]]))
        agent.save_param("aaa.ckpt")
        agent.load_param("aaa.ckpt")
        for i in range(10):
            print(agent.get_qvalue(np.array([[0.1,0.1]]),np.array([1])))
            agent.update(np.array([[0.1,0.1]]),np.array([1]),np.array([-10]),np.array([[0.2,0.2]]))

if __name__ == '__main__':
    agent=QLearningAgent_Approx(alpha=0.5,epsilon=0.25, discount=0.99, get_legal_actions=lambda s: range(4),state_dim=(2,),action_dim=4)
    agent.test()