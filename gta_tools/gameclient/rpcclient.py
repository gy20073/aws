from time import sleep

SERVER_PORT = 8765

def client():
	from xmlrpc import client
	return client.ServerProxy('http://localhost:%d'%SERVER_PORT)

if __name__ == "__main__":
	from argparse import ArgumentParser
	parser = ArgumentParser()
	parser.add_argument('-l', '--list', action='store_true')
	parser.add_argument('-r', '--running', action='store_true')
	parser.add_argument('-s', '--start')
	parser.add_argument('-t', '--stop', type=int)
	parser.add_argument('-k', '--kill', type=int)
	parser.add_argument('--log', type=int)
	args = parser.parse_args()

	c = client()
	if args.list:
		print( 'Available games:' )
		for n in c.games():
			print( ' * ', n )
	
	if args.running:
		print( 'Running games:' )
		for n in c.runningGames():
			print( ' * ', n )
	
	if args.start is not None:
		print( c.startGame(args.start) )
	
	if args.stop is not None:
		print( c.stopGame(args.stop) )
	
	if args.kill is not None:
		print( c.killGame(args.kill) )
	
	if args.log is not None:
		print( c.log(args.log) )
