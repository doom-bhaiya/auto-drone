import threading as td
import time

count=int(input("enter it 1st:"))

tym=time.perf_counter()

def run():
    global count
    while count>0:
        print(count)
        time.sleep(1)
        count-=1

def update_count():
    global count
    new_count=int(input("Enter 2nd after call:"))
    count = new_count

t = td.Thread(target=run)
t1 = td.Thread(target = update_count)
t.start()
t1.start()
t.join()

tym2=time.perf_counter()
print(f'{tym2-tym}')