import threading as th
import keyboard as k
import time

stop = False  
count = int(input("Enter count: "))
def print_numbers():
    global count, stop  
    while count > 0 and not stop:
        time.sleep(1)
        print(count)
        count -= 1
    print("Stopped")
t = th.Thread(target=print_numbers)
t.start()
print("Press q to stop.")
k.wait('q') 
stop = True 
t.join() 
print("Thread finished execution.")
