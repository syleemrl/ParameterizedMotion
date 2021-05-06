import simEnv
import time
import datetime
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import copy
from utils import RunningMeanStd
from IPython import embed
import os

class EnvWrapper(object):
	def __init__(self, ref, num_slave, directory, parametric, load_param, plot=False):		

		self.sim_env = simEnv.Env(num_slave, directory, ref, parametric, load_param)
		self.parametric = parametric

		self.num_state = self.sim_env.getNumState()
		self.num_action = self.sim_env.getNumAction()
		self.num_slave = num_slave
		
		self.motionlength = self.sim_env.getMotionLength()
		self.reward_label = self.sim_env.getRewardLabels()

		self.RMS = RunningMeanStd(shape=(self.num_state))	
		self.plot = plot
		self.directory = directory

		self.start_time = time.time()		

		self.num_total_iterations = 0
		self.num_total_episodes = 0
		self.num_total_transitions = 0
		self.total_rewards = []
		self.total_rewards_by_parts = np.array([[]]*len(self.reward_label))

		self.num_nan_per_iteration = 0
		self.num_episodes_per_iteration = 0
		self.num_transitions_per_iteration = 0
		self.nt_elapsed_iteration = 0
		self.rewards_per_iteration = 0
		self.rewards_by_part_per_iteration = []

		self.max_episode_length = 0
		self.transition_per_episodes_history = []

		self.terminated = [False]*self.num_slave
		self.states = [0]*self.num_slave
		self.prevframes = [0]*self.num_slave

		if self.plot:
			plt.ion()

		if self.parametric:
			self.num_param = self.sim_env.getNumParam()

	def get_states(self):
		return np.array(self.states).astype('float32') 

	def set_terminated(self, idx):
		self.terminated[idx] = True
	
	def get_terminated(self, idx):
		return self.terminated[idx]
	
	def get_all_terminated(self):
		for i in range(self.num_slave):
			if not self.terminated[i]:
				return False
		return True 
	
	def reset(self, i, b=True):
		self.sim_env.reset(i, b)
		state = np.array([self.sim_env.getState(i)])
		self.states[i] = self.RMS.apply(state)[0]
		self.terminated[i] = False
		self.prevframes[i] = 0

	def step_sim(self, actions):
		rewards = []
		dones = []
		ntimes = []
		times = []
		nan_count = 0

		self.sim_env.setActionAll(actions)
		self.sim_env.stepAll()

		for j in range(self.num_slave):
			is_terminal, nan_occur, nt_elapsed, t_elapsed = self.sim_env.getStepInfo(j)
			if not nan_occur:
				r = self.sim_env.getRewardVector(j)
				rewards.append(r)
				dones.append(is_terminal)
			else:
				rewards.append([0, 0])
				dones.append(True)
				nan_count += 1

			times.append(t_elapsed)
			ntimes.append(nt_elapsed)

		states = self.sim_env.getStateAll()
		return states, rewards, dones, times, ntimes, nan_count 

	def step(self, actions, record=True):
		self.states, rewards, dones, times, ntimes, nan_count =  self.step_sim(actions)

		if self.parametric:
			params = np.array(self.states)[:,-self.num_param:]
			phases = np.array(self.states)[:,-(self.num_param+1)]
		else:
			params = np.zeros(1)
			phases = np.array(self.states)[:,-1]

		states_updated = self.RMS.apply(self.states[~np.array(self.terminated)])
		self.states[~np.array(self.terminated)] = states_updated
		if record:
			self.num_nan_per_iteration += nan_count
			for i in range(self.num_slave):
				if not self.terminated[i] and len(rewards[i]) != 2:
					self.rewards_per_iteration += rewards[i][0]
					self.rewards_by_part_per_iteration.append(rewards[i])
					self.num_transitions_per_iteration += 1

					if dones[i]:
						self.num_episodes_per_iteration += 1
						self.nt_elapsed_iteration += ntimes[i]

						if ntimes[i] > self.max_episode_length:
							self.max_episode_length = ntimes[i]


		if self.parametric:
			rewards = [[rewards[i][0], rewards[i][1]] for i in range(len(rewards))]
		else:	
			rewards = [rewards[i][0] for i in range(len(rewards))]
				
		return rewards, dones, phases, params

	def plot_fig(self, y_list, title, num_fig=1, ylim=True, path=None):
		if self.plot:
			plt.figure(num_fig, clear=True, figsize=(5.5, 4))
		else:
			plt.figure(num_fig, figsize=(5.5, 4))
		plt.title(title)

		i = 0
		for y in y_list:
			plt.plot(y[0], label=y[1])
			i+= 1
		plt.legend(loc=2)
		if self.plot:
			plt.show()
			if ylim:
				plt.ylim([0,1])
			plt.pause(0.001)
		if path is not None:
			plt.savefig(path, format="png")

	def print_summary(self, save=True):
		r_per_e = self.rewards_per_iteration / self.num_episodes_per_iteration
		rp_per_i = np.array(self.rewards_by_part_per_iteration).sum(axis=0) / self.num_transitions_per_iteration

		if save:
			self.total_rewards.append(r_per_e)
		
			self.num_total_transitions += self.num_transitions_per_iteration
			self.num_total_episodes += self.num_episodes_per_iteration
			self.num_total_iterations += 1
			self.total_rewards_by_parts = np.insert(self.total_rewards_by_parts, self.total_rewards_by_parts.shape[1], 
				np.asarray(self.rewards_by_part_per_iteration).sum(axis=0)/self.num_episodes_per_iteration, axis=1)

			print_list = []
			print_list.append('===============================================================')
			print_list.append(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
			print_list.append("Elapsed time : {:.2f}s".format(time.time() - self.start_time))
			print_list.append('Num iter : {}'.format(self.num_total_iterations))
			print_list.append('total episode count : {}'.format(self.num_total_episodes))
			print_list.append('total transition count : {}'.format(self.num_total_transitions))

			t_per_e = 0
			if self.num_total_episodes != 0:
				t_per_e = self.num_total_transitions / self.num_total_episodes
			print_list.append('total transition per episodes : {:.2f}'.format(t_per_e))

			print_list.append('episode count : {}'.format(self.num_episodes_per_iteration))
			print_list.append('transition count : {}'.format(self.num_transitions_per_iteration))
			
			t_per_e = 0
			if self.num_episodes_per_iteration != 0:
				t_per_e = self.num_transitions_per_iteration / self.num_episodes_per_iteration
			self.transition_per_episodes_history.append(t_per_e)

			print_list.append('transition per episodes : {:.2f}'.format(t_per_e))
			print_list.append('rewards per episodes : {:.2f}'.format(self.total_rewards[-1]))
			print_list.append('max episode length : {}'.format(self.max_episode_length))

			nt_per_t  = 0
			if self.num_transitions_per_iteration != 0:
				nt_per_t = self.nt_elapsed_iteration / self.num_transitions_per_iteration;
			print_list.append('normalized time elapsed per transition : {:.2f}'.format(nt_per_t))		

			if self.num_nan_per_iteration != 0:
				print_list.append('nan count : {}'.format(self.num_nan_per_iteration))
			print_list.append('===============================================================')

			for s in print_list:
				print(s)
			
			if self.directory != None:
				out = open(self.directory+"results", "a")
				for s in print_list:
					out.write(s+'\n')
				out.close()

			if self.plot:
				y_list = [[np.asarray(self.transition_per_episodes_history), 'steps']]
				for i in range(len(self.total_rewards_by_parts)):
					y_list.append([np.asarray(self.total_rewards_by_parts[i]), self.reward_label[i]])

				self.plot_fig(y_list, "rewards" , 1, False, path=self.directory+"result.png")

				y_list = y_list[1:]
				for i in range(len(y_list)):
					y_list[i][0] = np.array(y_list[i][0])/np.array(self.transition_per_episodes_history)

				self.plot_fig(y_list, "rewards_per_step", 2, False, path=self.directory+"result_per_step.png")

		self.num_nan_per_iteration = 0
		self.num_episodes_per_iteration = 0
		self.num_transitions_per_iteration = 0
		self.rewards_per_iteration = 0
		self.rewards_by_part_per_iteration = []
		self.nt_elapsed_iteration = 0

		summary = dict()
		summary['r_per_e'] = r_per_e
		summary['rp_per_i'] = rp_per_i
		summary['t_per_e'] = t_per_e

		if not self.parametric and self.num_total_iterations > 400 and self.num_total_iterations % 100 == 0:
			w = min(1.0, (self.num_total_iterations-400) / 1000.0)
			self.sim_env.setWeightAll(w)

		return summary
