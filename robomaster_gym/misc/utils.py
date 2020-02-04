
import random
import roslaunch

class DemoPool:
	def __init__(self, max_size=1e6):
		self._keys = ('observations', 'actions', 'next_observations', 'rewards', 'terminals')
		self._fields = {}
		self._max_size = int(max_size)
		self._size = 0
		self._pointer = 0

	@property
	def size(self):
		return self._size


	def add_sample(self, *arrays):
		if self._size:
			self._add(arrays)
		else:
			self._init(arrays)

		self._advance()
		# print(self._size, self._pointer)

	def save(self, params, *savepath):
		savepath = os.path.join(*savepath)
		self._prune()
		save_info = [(key, self._fields[key].shape) for key in self._keys]
		print('[ DemoPool ] Saving to: {} | {}'.format(savepath, save_info))
		pickle.dump(self._fields, open(savepath, 'wb+'))

		## save params
		params_path = savepath.replace('pool', 'params')
		pickle.dump(params, open(params_path, 'wb+'))

	def _add(self, arrays):
		for key, array in zip(self._keys, arrays):
			self._fields[key][self._pointer] = array

	def _init(self, arrays):
		for key, array in zip(self._keys, arrays):
			shape = array.shape if type(array) == np.ndarray else (1,)
			dtype = array.dtype if type(array) == np.ndarray else type(array)
			self._fields[key] = np.zeros((self._max_size, shape), dtype=dtype)
			self._fields[key][self._pointer] = array
			# print(key, self._fields[key].shape, self._fields[key].dtype)

	def _advance(self):
		self._size = min(self._size + 1, self._max_size)
		self._pointer = (self._pointer + 1) % self._max_size

	def _prune(self):
		for key in self._keys:
			self._fields[key] = self._fields[key][:self._size]

	def get_samples(self):
		self._prune()
		return self._fields

chance = lambda prob: random.random() < prob

# Zone Types:
# 1: no movement debuff
# -1: no shoot debuff
# 2: team 0-1 health refill -2: team 2-3 health refill
# 3: team 0-1 bullet refill -3: team 2-3 enemy bullet refill
# due to symmetry, the second half is always negated version of first half
def generate_random_zone_config():
    permutation = [1, 2, 3]
    random.shuffle(permutation)
    permutation = [-n if chance(0.5) else n for n in permutation]
    return permutation + [-n for n in permutation]

class rosFromPython:
	__instantiated = False
	def __init__(self):
		if rosFromPython.__instantiated:
			raise Exception("ROS start up should only be run once")
		self.__instantiated = True

		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		cli_args1 = ['roborts_sim','multi_robot.launch','gui:=false']
		roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
		roslaunch_args1 = cli_args1[2:]
		self.launch = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file1,roslaunch_args1)], is_core=True)
		self.launch.start()

	
	def shutdown(self):
		self.launch.shutdown()
		