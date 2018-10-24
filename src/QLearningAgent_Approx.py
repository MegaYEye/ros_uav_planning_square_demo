
from collections import defaultdict
import random, math
import numpy as np
import pickle
from functools import partial

import tensorflow as tf
import keras
import keras.layers as L

class QLearningAgent_Approx:
    def __init__(self, alpha, epsilon, discount, get_legal_actions,state_dim,action_dim):
        tf.reset_default_graph()
        self.sess = tf.InteractiveSession()
        keras.backend.set_session(sess)
        self.network = keras.models.Sequential()
        self.network.add(L.InputLayer(state_dim))

        # let's create a network for approximate q-learning following guidelines above
        # <YOUR CODE: stack more layers!!!1 >
        self.network.add(L.Dense(100,activation='relu'))
        self.network.add(L.Dense(100,activation='relu'))
        self.network.add(L.Dense(n_actions,activation='linear'))

        self.get_legal_actions = get_legal_actions
        #self._qvalues = defaultdict(lambda: defaultdict(lambda: 0))
        self._qvalues = defaultdict(partial(defaultdict,int))
        self.alpha = alpha
        self.epsilon = epsilon
        self.discount = discount

        # Create placeholders for the <s, a, r, s'> tuple and a special indicator for game end (is_done = True)
        self.states_ph = tf.placeholder('float32', shape=(None,) + state_dim)
        self.actions_ph = tf.placeholder('int32', shape=[None])
        self.rewards_ph = tf.placeholder('float32', shape=[None])
        self.next_states_ph = tf.placeholder('float32', shape=(None,) + state_dim)
        self.is_done_ph = tf.placeholder('bool', shape=[None])

    def get_qvalue(self, state, action):
        """ Returns Q(state,action) """
        return self._qvalues[state][action]

    def set_qvalue(self,state,action,value):
        """ Sets the Qvalue for [state,action] to the given value """
        self._qvalues[state][action] = value

    #---------------------START OF YOUR CODE---------------------#

    def get_value(self, state):
        """
        Compute your agent's estimate of V(s) using current q-values
        V(s) = max_over_action Q(state,action) over possible actions.
        Note: please take into account that q-values can be negative.
        """
        possible_actions = self.get_legal_actions(state)

        #If there are no legal actions, return 0.0
        if len(possible_actions) == 0:
            return 0.0

#         <YOUR CODE HERE>
        value=None
        for action in possible_actions:
            val=self.get_qvalue(state,action)
            if value is None:
                value=val
            if value<val:
                value=val

        return value

    def update(self, state, action, reward, next_state):
        """
        You should do your Q-Value update here:
           Q(s,a) := (1 - alpha) * Q(s,a) + alpha * (r + gamma * V(s'))
        """

        #agent parameters
        gamma = self.discount
        learning_rate = self.alpha

        states_ph=self.states_ph
        actions_ph=self.actions_ph
        next_states_ph=self.next_states_ph
        rewards_ph=self.rewards_ph
        n_actions=self.action_dim


#         <YOUR CODE HERE>
        # Q= (1-learning_rate) * self.get_qvalue(state,action) + learning_rate * (reward+gamma*self.get_value(next_state))
        
        
        # self.set_qvalue(state, action, Q)
        #get q-values for all actions in current states
        predicted_qvalues = network(states_ph)

        #select q-values for chosen actions
        predicted_qvalues_for_actions = tf.reduce_sum(predicted_qvalues * tf.one_hot(actions_ph, n_actions), axis=1)
        # compute q-values for all actions in next states
        #predicted_next_qvalues = <YOUR CODE - apply network to get q-values for next_states_ph>
        predicted_next_qvalues = network(next_states_ph)
        # compute V*(next_states) using predicted next q-values
        # next_state_values = <YOUR CODE>
        next_state_values = tf.reduce_max(predicted_next_qvalues, axis=1)
        # compute "target q-values" for loss - it's what's inside square parentheses in the above formula.
        # target_qvalues_for_actions = <YOUR CODE>
        target_qvalues_for_actions = rewards_ph+gamma*next_state_values
        # at the last state we shall use simplified formula: Q(s,a) = r(s,a) since s' doesn't exist
        target_qvalues_for_actions = tf.where(is_done_ph, rewards_ph, target_qvalues_for_actions)

        loss = (predicted_qvalues_for_actions - tf.stop_gradient(target_qvalues_for_actions)) ** 2
        loss = tf.reduce_mean(loss)

        # training function that resembles agent.update(state, action, reward, next_state) from tabular agent
        train_step = tf.train.AdamOptimizer(1e-4).minimize(loss)

            sess.run(train_step,{
                states_ph: [s], actions_ph: [a], rewards_ph: [r], 
                next_states_ph: [next_s], is_done_ph: [done]
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
        for action in possible_actions:
            val=self.get_qvalue(state,action)
            if value is None:
                value=val
                best_action=action
            if value<val:
                value=val
                best_action=action

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
        q_values = network.predict(state[None])[0]
    #     print(q_values)
        
        ###YOUR CODE
        if random.random()<epsilon:
            chosen_action=np.random.choice(len(q_values))
        else:
            chosen_action=np.argmax(q_values)   
        return chosen_action

    def save_param(self, path):
        with open(path, "wb") as p:
            pickle.dump(self._qvalues, p, protocol=pickle.HIGHEST_PROTOCOL)
            
    def load_param(self, path):
        try:
            with open(path, 'rb') as p:
                self._qvalues = pickle.load(p)
                # print(self._qvalues)
        except:
            print("load param failure! let's start from a empty q table!")
            #self._qvalues = defaultdict(lambda: defaultdict(lambda: 0))
            self._qvalues = defaultdict(partial(defaultdict,int))