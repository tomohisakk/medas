import gym
import math
import random
import numpy as np
from enum import IntEnum

from map import MakeMap
from map import Symbols

class Actions(IntEnum):
	U = 0
	R = 1
	D = 2
	L = 3

	# diagnal move
	UR = 4
	UL = 5
	DR = 6
	DL = 7

	# change shape
	C1 = 8
	C2 = 9
	C3 = 10
	C4 = 11

class MEDAEnv(gym.Env):
	def __init__(self, w=8, h=8, dsize=1, s_modules=2, d_modules=2, test_flag=False):
		super(MEDAEnv, self).__init__()
		assert w>0 and h>0 and dsize>0
		assert 0<=s_modules and 0<=d_modules
		self.w = w
		self.h = h
		self.dsize = dsize
		self.actions = Actions
		self.action_space = len(self.actions)
		self.observation_space = (3, w, h)
		self.n_steps = 0
		self.max_step = 5*(w+h)

		self.state = (0,0)
		self.goal = (w-1, h-1)

		self.map_symbols = Symbols()
		self.mapclass = MakeMap(w=self.w,h=self.h,dsize=self.dsize,s_modules=s_modules,d_modules=d_modules)
		self.map = self.mapclass.gen_random_map()

		self.test_flag = test_flag

		self.dynamic_flag = 0
		self.dynamic_state = (0,0)

		self.is_vlong = False

	def reset(self, test_map=None):
		self.n_steps = 0
		self.state = (0, 0)

		if self.test_flag == False:
			self.map = self.mapclass.gen_random_map()
		else:
			self.map = test_map

		obs = self._get_obs()

		return obs

	def step(self, action):
		done = False
		message = None
		self.n_steps += 1

		_dist = self._get_dist(self.state, self.goal)
		self._update_position(action)

#		if self.dynamic_flag == 1:
#			dist = self._get_dist(self.dynamic_state, self.goal)
#			self.dynamic_flag = 0
#			message = "derror"
#		else:
		dist = self._get_dist(self.state, self.goal)

		if dist <= (self.dsize-1)*math.sqrt(2):
			reward = 1
			done = True
		elif self.n_steps == self.max_step:
			reward = 0
			done = True
		elif self.dynamic_flag == 1:
			reward = 0
			self.dynamic_flag = 0
			message = "derror"
#		elif dist < _dist:
#			reward = -0.1
		else:
			reward = -0.1


#		print(Actions(action))
#		print(self.map)

		obs = self._get_obs()

		return obs, reward, done, message

	def _get_dist(self, state1, state2):
		diff_x = state1[1] - state2[1]
		diff_y = state1[0] - state2[0]
		return math.sqrt(diff_x*diff_x + diff_y*diff_y)

	def _is_touching(self, state, obj):
		if self.is_vlong:
			if self.map[state[1]][state[0]] == obj or self.map[state[1]+1][state[0]] == obj:
				return True
			else:
				return False
		else:
			if self.map[state[1]][state[0]] == obj or self.map[state[1]][state[0]+1] == obj:
				return True
			else:
				return False

	def _update_position(self, action):
		state_ = list(self.state)

#		print(state_)
		if action == Actions.U:
			state_[1] -= 1
		elif action == Actions.R:
			state_[0] += 1
		elif action == Actions.D:
			state_[1] += 1
		elif action == Actions.L:
			state_[0] -= 1

		# diagnal movements
		elif action == Actions.UR:
			state_[0] += 1
			state_[1] -= 1
		elif action == Actions.UL:
			state_[1] -= 1
			state_[0] -= 1
		elif action == Actions.DR:
			state_[1] += 1
			state_[0] += 1
		elif action == Actions.DL:
			state_[1] += 1
			state_[0] -= 1

		elif action == Actioins.C1:
			if self.is_vlong:
				#L
				state_[0] -= 1
				self.is_vlong = False
			else:
				#U
				state_[1] -= 1
				self.is_vlong = True
		elif action == Actioins.C2:
			if self.is_vlong:
				#DL
				state_[1] += 1
				state_[0] += 1
				self.is_vlong = False
			else:
				#UR
				state_[0] += 1
				state_[1] -= 1
				self.is_vlong = True
		elif action == Actioins.C3:
			if self.is_vlong:
				#same
				self.is_vlong = False
			else:
				#same
				self.is_vlong = True
		elif action == Actioins.C4:
			if self.is_vlong:
				#D
				state_[1] += 1
				self.is_vlong = False
			else:
				#R
				state_[0] += 1
				self.is_vlong = True
		else:
			print("Unexpected action")
			return 0

		if self.is_vlong:
			if 0>state_[0] or 0>state_[1] or state_[0]>self.w-1 or state_[1]+1>self.h-1:
				self.is_vlong = False

			elif self._is_touching(state_, self.map_symbols.Dynamic_module):
				if self.map[self.state[1]][self.state[0]] == self.map_symbols.Dynamic_module:
					self.map[self.state[1]][self.state[0]] = self.map_symbols.Static_module
				elif self.map[self.state[1]+1][self.state[0]] == self.map_symbols.Dynamic_module:
					self.map[self.state[1]+1][self.state[0]] = self.map_symbols.Static_module
					self.is_vlong = False
			
			elif self._is_touching(state_, self.map_symbols.Static_module):
				self.is_vlong = False

			else:
				self.map[self.state[1]][self.state[0]] = self.map_symbols.Health
				self.map[self.state[1]+1][self.state[0]] = self.map_symbols.Health
				self.state = state_
				self.map[self.state[1]][self.state[0]] = self.map_symbols.State
				self.map[self.state[1]+1][self.state[0]] = self.map_symbols.State



		if (0 <= state_[1] <= self.h-self.dsize) and (0 <= state_[0] <= self.w-self.dsize) and\
		   (self._is_touching(state_, self.map_symbols.Dynamic_module) == False) and (self._is_touching(state_, self.map_symbols.Static_module) == False):
#			print("okok")
			i = 0
			while True:
				j = 0
				while True:
					self.map[self.state[1]+j][self.state[0]+i] = self.map_symbols.Health
					j += 1
					if j == self.dsize:
						break
				i += 1
				if i == self.dsize:
					break

#			print(Actions(action))
#			print(self.state)
			self.state = state_
#			print(self.state)

			# Set Droplet state
			i = 0
			while True:
				j = 0
				while True:
					self.map[self.state[1]+j][self.state[0]+i] = self.map_symbols.State
					j += 1
					if j == self.dsize:
						break
				i += 1
				if i == self.dsize:
					break

		elif (0 <= state_[1] < self.h-self.dsize+1) and (0 <= state_[0] < self.w-self.dsize+1) and\
		   (self._is_touching(state_, self.map_symbols.Dynamic_module) == True):
			self.dynamic_flag += 1
			self.dynamic_state = state_

			i = 0
			while True:
				j = 0
				while True:
					if self.map[state_[1]+j][state_[0]+i] == self.map_symbols.Dynamic_module:
						self.map[state_[1]+j][state_[0]+i] = self.map_symbols.Static_module
					j += 1
					if j == self.dsize:
						break
				i += 1
				if i == self.dsize:
					break

	def _get_obs(self):
		obs = np.zeros(shape = (3, self.w, self.h))
		for i in range(self.w):
			for j in range(self.h):
				if self.map[j][i] == self.map_symbols.State:
					obs[0][i][j] = 1
				elif self.map[j][i] == self.map_symbols.Goal:
					obs[1][i][j] = 1
				elif self.map[j][i] == self.map_symbols.Static_module:
					obs[2][i][j] = 1
#		print(obs)
		return obs

	def close(self):
		pass
