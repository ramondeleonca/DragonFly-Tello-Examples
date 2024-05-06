
if False:
    from FlyLib3.control.unofficial_tello import Tello
    drone = Tello()
else:
    from FlyLib3.control.tello import FlyLib3Tello
    drone = FlyLib3Tello()

drone.connect()
drone.takeoff()