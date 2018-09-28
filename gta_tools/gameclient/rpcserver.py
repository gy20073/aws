from xmlrpc import server

SERVER_PORT = 8765

class RequestHandler(server.SimpleXMLRPCRequestHandler):
	rpc_paths = ('/RPC2',)

game_path = {}
# game_list.py is a list of variables (game names) with corresponding executable paths
# e.g.:
#  GTA5 = 'c:\\Games\\GTA5\\gta5.exe'
try:
	from importlib import import_module
	game_list = import_module('game_list')
	game_path = {n:getattr(game_list, n) for n in dir(game_list) if len(n)>0 and n[0] != '_' and isinstance(getattr(game_list, n), str)}
except ImportError:
	pass

def games():
	return list(game_path)

running_games = {} # port -> pair (name, process id)
last_log = {} # port -> logfile

def cleanUpGames():
	for p in list(running_games):
		n, pid = running_games[p]
		if pid.poll() is not None:
			del running_games[p]

def runningGames():
	cleanUpGames()
	return [(running_games[p][0],p) for p in running_games]

def startGame(name, port=None):
	import os, subprocess
	from datetime import datetime
	cleanUpGames()
	
	if name not in game_path:
		raise "Game '%s' not found!"%name
	path = game_path[name]
	
	USED_PORTS = set(running_games)
	if port is None:
		# Find a free port
		for p in range(SERVER_PORT+1, SERVER_PORT+1000):
			if p not in USED_PORTS:
				break
		port = p
	
	if port in USED_PORTS:
		raise "Port %d already in use!"%port
	
	dirname = os.path.dirname(path)
	try:
		os.mkdir('logs')
	except:
		pass
	
	logfile = os.path.abspath(os.path.join('logs', name+'_{:%y_%m_%d_%H_%M_%S}'.format(datetime.now())+'_%d.txt'%port))
	# Start up the game on an unused port
	CREATE_NEW_PROCESS_GROUP = 0x00000200
	DETACHED_PROCESS = 0x00000008
	pid = subprocess.Popen([path], cwd=dirname, creationflags=DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP, env={'SERVER_PORT': str(port), 'LOGFILE': logfile, **os.environ})
	running_games[port] = (name, pid)
	last_log[port] = logfile
	return port

def stopGame(port):
	cleanUpGames()
	if port in running_games:
		n, pid = running_games[port]
		pid.terminate()
		return 1
	return 0

def killGame(port):
	cleanUpGames()
	if port in running_games:
		n, pid = running_games[port]
		pid.kill()
		return 1
	return 0

def log(port):
	if port in last_log:
		return open(last_log[port], 'r').read()
	return ""

with server.SimpleXMLRPCServer(("", SERVER_PORT), requestHandler=RequestHandler) as server:
	server.register_introspection_functions()
	
	server.register_function(games)
	server.register_function(cleanUpGames)
	server.register_function(runningGames)
	server.register_function(startGame)
	server.register_function(stopGame)
	server.register_function(killGame)
	server.register_function(log)
	
	server.serve_forever()
