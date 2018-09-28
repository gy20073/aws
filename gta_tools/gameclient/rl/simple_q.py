# Some really simple and dumb tabular Q learning
import numpy as np

# A policy is a function that maps states to actions

# the tracer function takes a policy and returns a (state, action, reward) trace

def LERP(A, i):
	if len(i) == 1:
		i, = i
		i0 = int(i)
		i1 = i0+1
		return (i-i0)*A[i1] + (i1-i)*A[i0]
	else:
		i0 = int(i[0])
		i1 = i0+1
		return (i[0]-i0)*LERP(A[i1],i[1:]) + (i1-i[0])*LERP(A[i0],i[1:])

def LERP_ADD(A, i, delta):
	if len(i) == 1:
		i, = i
		i0 = int(i)
		i1 = i0+1
		A[i1] += (i-i0)*delta
		A[i0] += (i1-i)*delta
	else:
		i0 = int(i[0])
		i1 = i0+1
		LERP_ADD(A[i0], i[1:], (i1-i[0])*delta)
		LERP_ADD(A[i1], i[1:], (i[0]-i0)*delta)


class QL:
	def __init__(self, state_space, action_space, tracer, stochastic=0, discount=0.99):
		self.Q = np.zeros(state_space + action_space)
		self.state_space = state_space
		self.action_space = action_space
		self.tracer = tracer
		self.stochastic = stochastic
		self.traces = []
		self.trace_steps = []
		self.discount = discount
	
	def __Q(self, state):
		scaled_state = np.array(state)*np.array(self.state_space)*0.9999
		# TODO: LERP here
		return LERP(self.Q, scaled_state)
		return self.Q[scaled_state.astype(np.uint32)]

	def __updateQ(self, state, delta):
		scaled_state = np.array(state)*np.array(self.state_space)*0.9999
		# TODO: LERP here
		return LERP_ADD(self.Q, scaled_state, delta)
		self.Q[scaled_state.astype(np.uint32)] += delta

	def policy(self, state):
		actions = self.__Q(state)
		if not self.stochastic:
			if np.random.random() < 0.1:
				id = np.random.choice(actions.size)
				return np.unravel_index(id, actions.shape)
			return np.argmax( actions ),
		P = np.exp(self.stochastic*(actions-np.max(actions)))
		P /= np.sum(P)
		id = np.random.choice(P.size,p=P.ravel())
		return np.unravel_index(id, actions.shape)

	def updateQ(self, lr=0.1):
		for s0,a,s1,r in self.trace_steps:
			Q0 = self.__Q(s0)
			Q1 = self.__Q(s1)
			deltaQ = r + self.discount * np.max(Q1) - Q0[a]
			Q1[:] = 0
			Q1[a] = deltaQ
			self.__updateQ(s0, Q1)
	
	def collectTraces(self):
		T = self.tracer(self.policy)
		t0 = T[0]
		for t1 in T[1:]:
			self.trace_steps.append((t0[0],t0[1],t1[0],t1[2]))
			t0 = t1

	def save_traces(self, filename):
		from pickle import dump
		with open(filename, 'wb') as f:
			dump(self.trace_steps, f)

	def load_traces(self, filename):
		from pickle import load
		with open(filename, 'rb') as f:
			self.trace_steps = load(f)

	@property
	def V(self):
		# TODO: Fix for multi-dim axis
		return np.max(self.Q, axis=-1)
