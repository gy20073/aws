from client import Client, MS, FRAMES
from rpcclient import client as rpc_client
from time import sleep

class Sled(Client):
	def __init__(self, host=None, targets=[], target_step=100*MS, reward=None, tick_fn=None, tick_step=100*MS):
		if host is None:
			try:
				c = rpc_client()
				
				if 'SupersonicSled' in c.games():
					P = [p for g,p in c.runningGames() if g=='SupersonicSled']
					if not len(P):
						print( "Starting up sled, waiting to connect... (10 sec)" )
						port = c.startGame('SupersonicSled')
						sleep(10)
						host = ('localhost', port)
					else:
						host = ('localhost', P[0])
			except (ConnectionResetError, ConnectionRefusedError):
				pass
		
		if host is None:
			host = ('localhost', 8766)
			print( "Failed to find running game instance, and failed to launch new one using RPC! Trying localhost:8766 ..." )
		super().__init__(host)
		if not self.control():
			raise Exception("Did not get exclusive control, giving up!")
		
		self.reset()
		self.fetchGameState(self._set_game_state, tick_step)
		self.state = None
		
		if len(targets):
			self.fetchTarget(targets, self._set_target, step=target_step, W=128, H=128)
		self.targets = {}
		self.last_target_update = 0
		
		self.tick_fn = tick_fn
		self.reward = reward
		self.current_key_state = {}
	
	def resetAndWait(self):
		self.reset()
		self.state = None
		# Wait for first state
		while self.state is None or not self.state['ready'] or self.state['started']:
			sleep(0.1)
		# Wait for one more state
		for it in range(1):
			ts = self.state['timestamp']
			while self.state is None or not self.state['timestamp'] > ts:
				sleep(0.1)
		
	
	def tick(self):
		if self.tick_fn is not None:
			action = self.tick_fn(self)
			if action is not None:
				self.act(action)
	

	def act(self, a):
		if isinstance(a, (list, tuple)):
			a = {'thurst':a[0], 'boost':a[1]}
		if 'thrust' in a:
			if a['thrust'] > 0.5:
				self.sendKey(b'\x26', 2) # VK_UP
			else:
				self.sendKey(b'\x26', 0) # VK_UP

			if a['thrust'] < -0.5:
				self.sendKey(b'\x28', 2) # VK_DOWN
			else:
				self.sendKey(b'\x28', 0) # VK_DOWN
		
		if 'boost' in a:
			if a['boost'] > 0.5:
				n = b'1'
				if self.state is not None:
					if self.state['booster1'] == 0:
						n = b'1'
					elif self.state['booster2'] == 0:
						n = b'2'
					elif self.state['booster3'] == 0:
						n = b'3'
					elif self.state['booster4'] == 0:
						n = b'4'
				self.sendKey(n, 0.1)
			if a['boost'] < -0.5:
				self.sendKey(b'5', 0.1)
	
	def listActions(self):
		from collections import OrderedDict
		r = OrderedDict()
		r['thrust'] = [-1,0,1]
		r['boost'] = [-1,0,1]
		return r
	
	def _set_target(self, n, fid, ts, data):
		self.targets[n] = data
		self.last_target_update = ts
	
	def _set_game_state(self, fid, ts, state):
		import numpy as np
		vrs = list(state) + [fid,ts]
		nms = self.game_state_info.np_fmt_str + [('frame_id', 'i8'), ('timestamp', 'i8')]
		if self.reward is not None:
			vrs.append( eval(self.reward, {}, {n:v for v,n in zip(nms, vrs)}) )
			nms.append(('reward','f'))
		self.state = np.array(tuple(vrs), dtype=nms)
		if not self.state['complete']:
			self.state = None
		self.tick()

	def start(self):
		self.sendKey(b'\x26', 0.2) # VK_UP

	# Possible values include : 'invalid', 'dead', 'finished', 'started', 'ready'
	@property
	def game_state(self):
		if self.state is None:
			return 'invalid'
		if not self.state['alive']:
			return 'dead'
		if self.state['finished']:
			return 'finished'
		if self.state['started']:
			return 'started'
		if self.state['ready']:
			return 'ready'
		return 'invalid'

def collectTrace(policy, step_size=100*MS, reward = None, timeout = None, state_func = lambda x: x, action_func = lambda x: x):
	from time import sleep
	state_action = []

	def tick(game, state_action):
		from copy import copy
		state = copy(state_func(game.state))
		action = policy(state)
		state_action.append((state,action,game.state['reward']))
		return action_func(action)
	try:
		s = Sled(tick_fn=lambda g: tick(g, state_action), reward = reward, tick_step=step_size)
	except:
		return None
	s.resetAndWait()
	s.start()
	t0 = s.state['timestamp']
	t1 = t0 + 1e100
	if timeout is not None:
		t1 = t0 + timeout
	while s.game_state in ['ready', 'started'] and s.state['timestamp'] < t1:
		sleep(0.1)
	
	s.disconnect()

	return state_action

SIMPLE_SPACE = (10,10)
def simpleState(s):
	from math import sqrt
	return (min(1,max(0,s['position'])), sqrt(min(1,max(0,s['speed']))))

SIMPLE_ACTION = (3,)
def simpleAction(a):
	return {'thrust': a[0]-1}

def simpleTrace(simple_policy, step_size=500*MS, reward = None, timeout = None):
	return collectTrace(simple_policy, step_size, reward, timeout, state_func=simpleState, action_func=simpleAction)

def simple_policy(state):
	if state['position'] < 0.8:
		return {'thrust': 1}
	elif state['position'] < 0.85:
		return {'thrust': 0}
	else:
		return {'thrust': -1}

if __name__ == "__main__":
	from rl.simple_q import QL
	from time import sleep

	learner = QL(SIMPLE_SPACE, SIMPLE_ACTION, lambda p: simpleTrace(p, reward="100*(alive-1) + position + speed + 10*finished"), stochastic=0)
	try:
		learner.load_traces('tracer.dat')
	except:
		pass
	for it in range(20):
		learner.updateQ()

	from pylab import *
	ion()
	figure()
	subplot(2,2,1)
	imV = imshow(learner.V, vmin=0, vmax=.2)
	axis('off')
	title('value func')
	subplot(2,2,2)
	imQ0 = imshow(learner.Q[:,:,0], vmin=0, vmax=.2)
	axis('off')
	title('Q brake')
	subplot(2,2,3)
	imQ1 = imshow(learner.Q[:,:,1], vmin=0, vmax=.2)
	axis('off')
	title('Q nothing')
	subplot(2,2,4)
	imQ2 = imshow(learner.Q[:,:,2], vmin=0, vmax=.2)
	axis('off')
	title('Q thrust')
	pause(0.0001)
	for i in range(1000):
		learner.collectTraces()
		learner.save_traces('tracer.dat')
		learner.updateQ()
		imV.set_data(learner.V)
		imQ0.set_data(learner.Q[:,:,0])
		imQ1.set_data(learner.Q[:,:,1])
		imQ2.set_data(learner.Q[:,:,2])
		draw()
		pause(0.001)

	learner.stochastic = False
	learner.collectTraces()
	
	# trace = collectTrace(simple_policy, reward = "100*(alive-1) + position + 10*finished")
	# print( trace )
