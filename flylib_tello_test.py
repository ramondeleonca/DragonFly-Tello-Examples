from FlyLib3.control.EXPERIMENTAL_tello import FlyLib3Tello

tello = FlyLib3Tello(log_level=0, log_to_console=True, log_to_file=True, log=True)
tello.connect()
tello.takeoff()
tello.land()