import os
import numpy
import argparse

# def thermocouple(args):
# 	print('This is working?')
# 	args1 = args*2
# 	print(args1)
# 	print('See? its working!')

# integer example
# def main():
# 	parser = argparse.ArgumentParser()
# 	parser.add_argument('integer', type=int, help=
# 		'multiply integer provided by 2')
# 	args = parser.parse_args()
# 	thermocouple(args.integer)

def thermocouple(args):
	print('This is working?')
	filepath = '/Users/andrewwinhold/Desktop/'
	filename = args
	full = os.path.join(filepath, filename)
	f = open(full, 'a')
	print(full)
	print('See? its working!')

#string example, say for filename
def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('filename', type=str, help=
		'provide filename for data collection')
	args = parser.parse_args()
	thermocouple(args.filename)