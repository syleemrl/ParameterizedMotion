import numpy as np
import math
from IPython import embed
from copy import copy

class Sampler(object):
	def __init__(self, sim_env, dim):
		self.sim_env = sim_env
		self.dim = dim
		
		self.k = 5
		self.v_mean = 0

		self.er_exploit = [10.0]
		self.er_explore = [0]

		self.er_cur = 0
		self.er_ratio = 0

		self.mode = 'explore'
		self.mode_counter = 0
		self.total_iter = 0

	def random_sample(self, visited=1):
		return self.sim_env.uniformSample(visited)
		
	def adaptive_sample(self):
		t = np.random.randint(len(self.pool)) 
		goal = self.pool[t] 
		return goal

	def prob_adaptive(self, v_func, param):
		param = np.reshape(param, (-1, self.dim))
		v = v_func.get_value(param)[0]
		return math.exp(-self.k * (v - self.v_mean) / self.v_mean) + 1e-10

	def update_goal_distribution(self, v_func, m=10, N=1000):
		self.pool = []
		for i in range(m):
			x_cur = self.random_sample(1)
			for j in range(int(N/m)):
				self.pool.append(x_cur)
				x_new = self.random_sample(1)
				alpha = min(1.0, self.prob_adaptive(v_func, x_new)/self.prob_adaptive(v_func, x_cur))
			
				if np.random.rand() <= alpha:          
					x_cur = x_new

	def set_new_goal(self):
		if self.mode == 'explore' or self.mode == 'eval_explore':
			t = self.random_sample(visited=0)
		elif self.mode == 'exploit' and self.mode_counter > 3:
			t = self.adaptive_sample()		
		else:
			t = self.random_sample(visited=1)
		
		t = np.array(t, dtype=np.float32) 
		self.sim_env.setParamGoal(t)	

	def save_exploration_rate(self):
		self.er_cur += self.sim_env.getExplorationRate()

	def update_exploration_rate(self):
		print('mode: ', self.mode, ', exploration rate', self.er_cur)

		if self.mode == 'explore' or self.mode == 'eval_explore':
			self.er_explore.append(self.er_cur)
			if len(self.er_explore) > 5:
				self.er_explore = self.er_explore[-5:]
		else:
			self.er_exploit.append(self.er_cur)
			if len(self.er_exploit) > 5:
				self.er_exploit = self.er_exploit[-5:]
		self.er_cur = 0

	def transition_to_explore(self):
		p_mean = np.array(self.er_exploit).mean()
		p_mean_prev = np.array(self.er_explore).mean()
		print(self.er_exploit)
		print(p_mean, p_mean_prev)	
		if p_mean <= p_mean_prev:
			print('transition to explore')
			return True
		return False

	def transition_to_exploit(self):
		p_mean = np.array(self.er_explore).mean()
		p_mean_prev = np.array(self.er_exploit).mean()
		print(self.er_explore)
		print(p_mean, p_mean_prev)
		if p_mean <= p_mean_prev:
			print('transition to exploit')
			return True
		return False

	def update_curriculum(self, v_func, v_rollouts):

		self.total_iter += 1
		self.mode_counter += 1

		self.update_exploration_rate()

		if self.total_iter % 50 == 49:
			self.sim_env.saveParamSpace(-1)
		
		if self.total_iter % 10 == 9:
			self.sim_env.trainParamNetwork()
			self.er_ratio = self.sim_env.getVisitedRatio()

		if self.mode == 'explore':
			if self.mode_counter > 10 and self.transition_to_exploit():
				self.mode = 'exploit'
				self.er_exploit = []
				self.mode_counter  = 0
			elif self.mode_counter % 30 == 20:
				self.mode = 'eval_exploit'
				self.er_exploit = []
				self.eval_counter = 0

		elif self.mode == 'exploit':
			self.v_mean_cur = np.array(v_rollouts).mean()
			if self.v_mean == 0:
				self.v_mean = self.v_mean_cur
			else:
				self.v_mean = 0.6 * self.v_mean + 0.4 * self.v_mean_cur
			print('v mean: ', self.v_mean)
			if self.mode_counter % 5 == 4:
				self.update_goal_distribution(v_func)

			if self.er_ratio != 1 and self.mode_counter > 10 and self.transition_to_explore():
				self.mode = 'explore'
				self.er_explore = []
				self.mode_counter  = 0
			elif self.er_ratio != 1 and self.mode_counter % 30 == 20:
				self.mode = 'eval_explore'
				self.er_explore = []
				self.eval_counter = 0

		elif self.mode == 'eval_explore':
			self.eval_counter += 1
			if self.eval_counter >= 2:
				self.mode = 'exploit'

		elif self.mode == 'eval_exploit':
			self.eval_counter += 1
			if self.eval_counter >= 2:
				self.mode = 'explore'
