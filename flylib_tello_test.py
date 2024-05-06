from FlyLib3.control.tello import FlyLib3Tello
import time

tello = FlyLib3Tello()
tello.connect()
tello.takeoff()
time.sleep(1)
tello.land()