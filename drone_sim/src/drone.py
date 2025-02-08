import threading as td
import time
import keyboard as k
stop=False
def run_here():
    global stop
    n = int(input("enter a no:"))
    while n>0 and not stop:
        print("Moving!")
        time.sleep(1)
        n-=1
def repeat():
    global stop
    k.wait('q')
    stop=True
    t.join()
t = td.Thread(target=run_here,daemon=True)
t1 = td.Thread(target=repeat,daemon=True)
t.start()
t1.start()
t1.join()