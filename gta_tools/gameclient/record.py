from client import *

if __name__ == "__main__":
	from argparse import ArgumentParser
	import re
	import numpy as np
	from os import makedirs, path
	
	parser = ArgumentParser()
	parser.add_argument('-t', '--target', nargs='+', default=['.*'])
	parser.add_argument('output_dir')
	args = parser.parse_args()
	
	# Connect to the game
	c = Client(('localhost', 8766))
	
	# Figure out which targets we should fetch
	target_g = [re.compile(t) for t in args.target]
	target_list = []
	for t in c.listTargets():
		if np.any([g.match(t) for g in target_g]):
			target_list.append(t)
	
	
	# Fetch the targets
	try:
		makedirs(args.output_dir)
	except FileExistsError:
		pass
	
	key_state = {}
	def dump_key(key, state):
		key_state[key.decode()] = state
	
	def dump_target(name, frame_id, timestamp, data):
		if data.dtype == np.uint8 and len(data.shape)==3 and data.shape[2] >= 3:
			from skimage import io
			io.imsave(path.join(args.output_dir, '%06d_%s.bmp'%(frame_id,name)), data)
		else:
			from struct import pack
			# Write the raw file
			with open(path.join(args.output_dir, '%06d_%s.raw'%(frame_id,name)), 'wb') as f:
				type_str = ('%s%s%d'%(data.dtype.byteorder[0], data.dtype.char[0], data.dtype.itemsize)).encode()
				while len(type_str) < 4:
					type_str += b'\0'
				assert len(type_str) == 4
				S = list(data.shape)
				while len(S) < 3:
					S = S+[0]
				assert len(S) == 3
				f.write(type_str + pack('<III',*S))
				f.write(data.tobytes())
		
	def dump_state(frame_id, timestamp, state):
		with open(path.join(args.output_dir, '%06d_state.json'%frame_id), 'w') as f:
			state['key'] = key_state
			import json
			f.write(json.dumps(state))
	
	c.interceptKeys(dump_key)
	c.fetchTarget(target_list, dump_target, W=800, H=600, step=50*MS)
	c.fetchGameState(dump_state, step=1*FRAMES)
	
	sleep(10)
	
	# Finish recording
	c.interceptKeys(None)
	c.fetchTarget(target_list, None)
	c.fetchGameState(None)
	sleep(0.5)
