class CircularArrayList:
    def __init__(self, capacity):
        """
        Circular Array List의 초기화 메서드.
        :param capacity: Circular Array List의 최대 크기.
        """
        self.n = capacity + 1  # 실제 버퍼 크기 (capacity + 1)
        self.buf = [None] * self.n  # 버퍼 초기화
        self.head = 0  # Circular Array의 head 포인터
        self.tail = 0  # Circular Array의 tail 포인터

    def capacity(self):
        """버퍼의 최대 크기를 반환."""
        return self.n - 1

    def wrap_index(self, i):
        """인덱스를 순환형 인덱스로 변환."""
        m = i % self.n
        return m if m >= 0 else m + self.n

    def size(self):
        """Circular Array List에 저장된 요소의 개수를 반환."""
        return self.tail - self.head + (self.n if self.tail < self.head else 0)

    def get(self, i):
        """지정된 인덱스의 요소를 반환."""
        if i < 0 or i >= self.size():
            raise IndexError("Index out of bounds")
        return self.buf[self.wrap_index(self.head + i)]

    def get_latest(self):
        """마지막 요소를 반환."""
        if self.size() == 0:
            raise IndexError("CircularArrayList is empty")
        return self.buf[self.wrap_index(self.head + self.size() - 1)]

    def set(self, i, value):
        """지정된 인덱스에 값을 설정."""
        if i < 0 or i >= self.size():
            raise IndexError("Index out of bounds")
        self.buf[self.wrap_index(self.head + i)] = value

    def add(self, i, value):
        """지정된 인덱스에 값을 추가."""
        s = self.size()
        if s == self.n - 1:
            raise Exception("CircularArrayList is filled to capacity.")
        if i < 0 or i > s:
            raise IndexError("Index out of bounds")

        self.tail = self.wrap_index(self.tail + 1)
        if i < s:
            self.shift_block(i, s)
        self.set(i, value)

    def shift_block(self, start_index, end_index):
        """요소를 오른쪽으로 이동."""
        assert end_index > start_index
        for i in range(end_index - 1, start_index - 1, -1):
            self.set(i + 1, self.get(i))

    def remove(self, i):
        """지정된 인덱스의 요소를 제거."""
        s = self.size()
        if i < 0 or i >= s:
            raise IndexError("Index out of bounds")

        value = self.get(i)
        if i > 0:
            self.shift_block(0, i)
        self.head = self.wrap_index(self.head + 1)
        return value

    def remove_last(self):
        """마지막 요소를 제거."""
        if self.size() == 0:
            raise IndexError("CircularArrayList is empty")
        value = self.get(0)
        self.head = self.wrap_index(self.head + 1)
        return value

    def copy(self):
        """Circular Array List를 복사."""
        new_list = CircularArrayList(self.capacity())
        for i in range(self.size()):
            new_list.add(i, self.get(i))
        return new_list
