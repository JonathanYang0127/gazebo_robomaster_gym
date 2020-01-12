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
			self._fields[key] = np.zeros((self._max_size, *shape), dtype=dtype)
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

