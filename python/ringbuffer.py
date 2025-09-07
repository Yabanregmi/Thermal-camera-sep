class RingBuffer:
    def __init__(self, size):
        self.size = size
        self.data = [None] * size
        self.index = 0
        self.full = False

    def append(self, item):
        self.data[self.index] = item
        self.index = (self.index + 1) % self.size
        if self.index == 0:
            self.full = True

    def is_full(self):
        return self.full

    def get_all(self):
        if self.full:
            return self.data[self.index:] + self.data[:self.index]
        else:
            return self.data[:self.index]

    def clear(self):
        self.index = 0
        self.full = False
        self.data = [None] * self.size
