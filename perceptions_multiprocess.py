from all_perceptions import Perceptions
from multiprocessing import Lock

class PerceptionsMultiprocess():
    def __init__(self, **kwargs):
        #self.perception = Perceptions(**kwargs)
        self.lock = Lock()

    def all_in_one(self, image_input):
        self.lock.acquire()
        compute = image_input
        self.lock.release()
        print(compute)
        return compute

    def destroy(self):
        self.perception.destroy()

