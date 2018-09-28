import datetime
import threading

from utils.client_save_utils import *

c = Client(('localhost', 8766))
t = threading.Thread(target=capture_gta_to_folder, args=(datetime.datetime.now().strftime('%m%d_%H%M%S'), c))
t.daemon = True
t.start()


# capture_gta_to_folder(datetime.datetime.now().strftime('%m%d_%H%M%S'))
import time
time.sleep(5)
print('time to stop!')
c.disconnect()


# close()
# t.exit()
# stop_fetching()


