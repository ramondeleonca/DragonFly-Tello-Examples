import threading
from ursina import *
from ursina.prefabs.dropdown_menu import DropdownMenu, DropdownMenuButton
from FlyLib3.control.tello import Tello

engine = Ursina(title="FlyLib3 Telemetry", borderless=False)
drone = Tello()
drone_entity = Entity(model=load_model("./tello.obj"), texture=load_texture("tello-texture.png"), scale=0.1)
field = Entity(model=load_model("./TDC_BeyondTheSky.glb"))
camera = EditorCamera()

DropdownMenu("Rendering mode", buttons=(
    DropdownMenuButton("Wireframe"),
    DropdownMenuButton("Solid"),
))

def update_drone_entity_loop():
    while True:
        pitch = drone.get_pitch()
        yaw = drone.get_yaw()
        roll = drone.get_roll()
        print(pitch, yaw, roll)

        drone_entity.rotation_x = roll
        drone_entity.rotation_y = yaw
        drone_entity.rotation_z = pitch

        height = drone.get_height()

        drone_entity.position = (0, height / 100, 0)
        
        time.sleep(0.1)

# drone.connect()
# threading.Thread(target=update_drone_entity_loop, daemon=True).start()
engine.run()