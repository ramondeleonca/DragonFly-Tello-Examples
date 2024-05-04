import time
from ursina import Vec3

class SimulatedTello:
    battery: int
    min_battery: int
    connection_delay: float
    is_flying: bool = False
    idle_discharge_rate: float
    flying_discharge_rate: float
    last_tick_time: float = None
    position: Vec3 = Vec3(0, 0, 0)
    rotation: Vec3 = Vec3(0, 0, 0)

    def __init__(
            self,
            initial_battery: int = 100,
            min_battery: int = 15,
            connection_delay: float = 1,
            idle_discharge_rate: float = 0.01,
            flying_discharge_rate: float = 0.05,
        ):
        self.battery = initial_battery
        self.min_battery = min_battery
        self.connection_delay = connection_delay
        self.idle_discharge_rate = idle_discharge_rate
        self.flying_discharge_rate = flying_discharge_rate

    def tick(self, dt: float = None):
        now_time = time.time()
        if self.last_tick_time is None:
            self.last_tick_time = now_time
        if dt is None:
            dt = now_time - self.last_tick_time

        if self.is_flying:
            self.battery -= self.flying_discharge_rate * dt
        else:
            self.battery -= self.idle_discharge_rate * dt

        if self.battery < self.min_battery:
            self.land()

        self.last_tick_time = now_time

if __name__ == "__main__":
    drone = SimulatedTello()
    drone.connect()
    drone.takeoff()
    while True:
        drone.tick()
        print(drone.get_battery())
        time.sleep(1)
        if drone.get_battery() <= 0:
            break
    drone.land()
    print("Battery depleted, landing drone.")