
if True:
    from FlyLib3.control.tello import Tello
    drone = Tello(host="127.0.0.1")
else:
    from FlyLib3.control.tello import FlyLib3Tello
    drone = FlyLib3Tello()

drone.connect()
drone.takeoff()