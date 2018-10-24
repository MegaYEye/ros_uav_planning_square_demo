
from collections import defaultdict
import random, math
import numpy as np
import pickle
from functools import partial
class QLearningAgent:
    def __init__(self, alpha, epsilon, discount, get_legal_actions):
        """
        Q-Learning Agent
        based on http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html
        Instance variables you have access to
          - self.epsilon (exploration prob)
          - self.alpha (learning rate)
          - self.discount (discount rate aka gamma)

        Functions you should use
          - self.get_legal_actions(state) {state, hashable -> list of actions, each is hashable}
            which returns legal actions for a state
          - self.get_qvalue(state,action)
            which returns Q(state,action)
          - self.set_qvalue(state,action,value)
            which sets Q(state,action) := value

        !!!Important!!!
        Note: please avoid using self._qValues directly. 
            There's a special self.get_qvalue/set_qvalue for that.
        """

        self.get_legal_actions = get_legal_actions
        #self._qvalues = defaultdict(lambda: defaultdict(lambda: 0))
        self._qvalues = defaultdict(partial(defaultdict,int))
        self.alpha = alpha
        self.epsilon = epsilon
        self.discount = discount

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

#         <YOUR CODE HERE>
        Q= (1-learning_rate) * self.get_qvalue(state,action) + learning_rate * (reward+gamma*self.get_value(next_state))
        
        
        self.set_qvalue(state, action, Q)

    
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
        possible_actions = self.get_legal_actions(state)
        action = None

        #If there are no legal actions, return None
        if len(possible_actions) == 0:
            return None

        #agent parameters:
        epsilon = self.epsilon
        

            
#         <YOUR CODE HERE>
        if random.random()<epsilon:
            chosen_action=random.choice(possible_actions)
        else:
            chosen_action=self.get_best_action(state)        
        
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