import random
import numpy as np
class ReplayBuffer(object):
    def __init__(self, size):
        """
        Create Replay buffer.
        Parameters
        ----------
        size: int
            Max number of transitions to store in the buffer. When the buffer
            overflows the old memories are dropped.
            
        Note: for this assignment you can pick any data structure you want.
              If you want to keep it simple, you can store a list of tuples of (s, a, r, s') in self._storage
              However you may find out there are faster and/or more memory-efficient ways to do so.
        """
        self._storage = []
        self._maxsize = size
        
        # OPTIONAL: YOUR CODE
        

    def __len__(self):
        return len(self._storage)

    def add(self, state, action, reward, next_state, done):
        '''
        Make sure, _storage will not exceed _maxsize. 
        Make sure, FIFO rule is being followed: the oldest examples has to be removed earlier
        '''
        data = (state, action, reward, next_state, done)
        
        # add data to storage
        if len(self._storage)>=self._maxsize:
            self._storage.pop(0)
               
        self._storage.append(data)
        
    def sample(self, batch_size):
        """Sample a batch of experiences.
        Parameters
        ----------
        batch_size: int
            How many transitions to sample.
        Returns
        -------
        obs_batch: np.array
            batch of observations
        act_batch: np.array
            batch of actions executed given obs_batch
        rew_batch: np.array
            rewards received as results of executing act_batch
        next_obs_batch: np.array
            next set of observations seen after executing act_batch
        done_mask: np.array
            done_mask[i] = 1 if executing act_batch[i] resulted in
            the end of an episode and 0 otherwise.
        """
        #idxes = <randomly generate batch_size integers to be used as indexes of samples>
        idxes=np.random.choice(len(self._storage), batch_size)
        
        # collect <s,a,r,s',done> for each index

        data=[self._storage[i] for i in idxes]
        
        states, action, reward, next_states, is_done=zip(*list(data))
        
        
        return np.array(states), np.array(action), np.array(reward), np.array(next_states), np.array(is_done)
