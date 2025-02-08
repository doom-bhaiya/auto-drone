import threading
import time
class Mover:
    def __init__(self):
        self.distance = 0
        self.is_moving = False
        self.worker = None
    def go(self):
        if not self.is_moving: 
            self.is_moving = True
            if self.worker is None or not self.worker.is_alive():
                self.worker = threading.Thread(target=self._move)
                self.worker.start()
    def stop(self):    
        self.is_moving = False
        if self.worker is not None:
            self.worker.join()  
            self.worker = None
        print("Stopped")

    def _move(self):
        while self.is_moving:
            print("Moving forward. Press Enter again to stop")
            time.sleep(1) 
            ##self.distance += 1
        print("Movement paused .Press 'e' to quit.")
def key_listener(mover):
    print("Press 'Enter' to start moving.")
    while True:
        user_input = input()  
        if user_input == 'e':  
            mover.stop()
            print("Exit...")
            break
        elif not mover.is_moving:  
            mover.go()
        else: 
            mover.stop()
mover = Mover()
listener_thread = threading.Thread(target=key_listener, args=(mover,))
listener_thread.start()
listener_thread.join()

