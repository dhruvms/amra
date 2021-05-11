import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from PIL import Image
import imageio

def parse_args():
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

	parser.add_argument('-N', '--envs', type=int, default=10, help='Number of environments')
	parser.add_argument('--ttv', action='store_true', help='Split into train, test, validation')
	parser.add_argument('--same', action='store_true', help='Same sized passages')
	parser.add_argument('-p', '--passages', type=int, default=3, help='Number of narrow passages')
	parser.add_argument('--xwidths', nargs='+', type=int, default=[20], help='Width of narrow passages (x-axis)')
	parser.add_argument('--ywidths', nargs='+', type=int, default=[20], help='Width of narrow passages (y-axis)')
	parser.add_argument('-x', '--x-lims', nargs='+', type=int, default=[0, 201], help='Environment x limits')
	parser.add_argument('-y', '--y-lims', nargs='+', type=int, default=[0, 201], help='Environment y limits')

	args = parser.parse_args()
	return args

def gen_map(params, savedir, save=False, display=True):

	assert args.x_lims[0] == args.y_lims[0] and args.x_lims[1] == args.y_lims[1] # environment is square
	assert args.passages > 2
	envmap = np.ones((args.x_lims[1]-args.x_lims[0], args.y_lims[1]-args.y_lims[0]))

	split = args.x_lims[1]//args.passages
	if args.same:
		assert len(args.xwidths) == len(args.ywidths)

	pts = []
	for p in range(args.passages):
		# tempmap = np.ones((args.x_lims[1]-args.x_lims[0], args.y_lims[1]-args.y_lims[0]))
		# o = np.random.randint(0,2)
		o = p % 2
		pos1_x = np.random.randint(p*split, (p+1)*split)
		pos1_y = np.random.randint(0, args.y_lims[1])

		if args.same:
			wx = np.random.randint(2, args.xwidths[0])
			wy = np.random.randint(2, args.ywidths[0])
		else:
			wx = np.random.randint(2, args.xwidths[p])
			wy = np.random.randint(2, args.ywidths[p])

		if o: # passage going right
			pos2_x = pos1_x
			pos2_y = np.random.randint(0, args.y_lims[1])
			if (wy+split) >= args.y_lims[1] or (pos1_y+wy+split) >= args.y_lims[1]:
				pos2_y = args.y_lims[1]
			else:
				while np.abs(pos2_y - pos1_y) < wy+split:
					pos2_y = np.random.randint(pos1_y, args.y_lims[1])

			x_l = pos1_x - int(wx//1.5)
			x_r = pos1_x + int(wx//1.5)
			y_d = args.y_lims[0]
			y_u = args.y_lims[1]

			envmap[:, x_l:x_r] = 0.0
			pts.append((np.maximum(args.y_lims[1]-(pos1_y+(wy//2)), args.y_lims[0]), np.minimum(args.y_lims[1]-(pos1_y-(wy//2)), args.y_lims[1]-1), x_l, x_r))
			pts.append((np.maximum(args.y_lims[1]-(pos2_y+(wy//2)), args.y_lims[0]), np.minimum(args.y_lims[1]-(pos2_y-(wy//2)), args.y_lims[1]-1), x_l, x_r))
		else: # passage going up
			pos2_y = pos1_y
			pos2_x = np.random.randint(0, args.x_lims[1])
			if (wx+split) >= args.x_lims[1] or (pos1_x+wx+split) >= args.x_lims[1]:
				pos2_x = args.x_lims[1]
			else:
				while np.abs(pos2_x - pos1_x) < wx+split:
					pos2_x = np.random.randint(pos1_x, args.x_lims[1])

			x_l = args.x_lims[0]
			x_r = args.x_lims[1]
			y_d = args.y_lims[1] - (pos1_y - int(wy//1.5))
			y_u = args.y_lims[1] - (pos1_y + int(wy//1.5))

			envmap[y_u:y_d, :] = 0.0
			pts.append((y_u, y_d, np.maximum(pos1_x-(wx//2), args.x_lims[0]), np.minimum(pos1_x+(wx//2), args.x_lims[1])))
			pts.append((y_u, y_d, np.maximum(pos2_x-(wx//2), args.x_lims[0]), np.minimum(pos2_x+(wx//2), args.x_lims[1])))


	for x in pts:
		envmap[x[0]:x[1], x[2]:x[3]] = 1.0

	envmap[:, 0] = 0.0
	envmap[:, -1] = 0.0
	envmap[0, :] = 0.0
	envmap[-1, :] = 0.0
	return envmap

def gen_envs(params, savedir, save=False, display=True):
	for e in range(params.envs):
		filename = savedir
		if params.ttv:
			if e < params.envs*0.8:
				filename += 'train/' + str(e) + '.png'
			elif e < params.envs*0.9:
				filename += 'validation/' + str(e) + '.png'
			else:
				filename += 'test/' + str(e) + '.png'
		else:
			filename += 'train/' + str(e) + '.png'

		envmap = gen_map(params, savedir, save=save, display=display)
		if display:
			plt.imshow(envmap, cmap='gray')
			plt.show()
		if save:
			envmap *= 255
			imageio.imwrite(filename, envmap.astype(np.uint8))

if __name__ == '__main__':
	args = parse_args()

	savedir = './motion_planning_datasets/narrow_passages/'
	if not os.path.exists(savedir):
		os.makedirs(savedir)
	if not os.path.exists(savedir + 'train/'):
		os.makedirs(savedir + 'train/')
	if not os.path.exists(savedir + 'validation/'):
		os.makedirs(savedir + 'validation/')
	if not os.path.exists(savedir + 'test/'):
		os.makedirs(savedir + 'test/')

	gen_envs(args, savedir, save=True, display=False)
