import threading
import time

stop=False

def frst():
    for i in range(0,10):
        print("hi")
        time.sleep(0.5)

def count():
    # global stop
    for j in range(0,10):
        if stop:
            break
        print(j)
        time.sleep(0.5)

frst_thread= threading.Thread(target=frst,daemon=True)
count_thread=threading.Thread(target=count,daemon=True)

frst_thread.start()
count_thread.start()

input("press enter to stop counting")
stop=True

count_thread.join()
frst_thread.join()