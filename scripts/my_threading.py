import threading
import time

stop = False

def always_run():
    counter =0
    while not stop:
        time.sleep(1)
        print(counter)
        counter+=1
        
t = threading.Thread(target = always_run, daemon=True)
t.start()
input("Press Enter to stop\n")
stop = True
t.join()