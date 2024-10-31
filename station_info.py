class StationInfo:
    _instance = None # 싱글톤 용도

    # 싱글톤 설정
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(StationInfo, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        self.__header = None
        self.__version = None


    @property
    def header(self):
        return self.__header

    @header.setter
    def header(self, value):
        self.__header = value

    @property
    def version(self):
        return self.__version

    @version.setter
    def version(self, value):
        self.__version = value

