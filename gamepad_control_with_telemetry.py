import pygame
from ursina import *
from FlyLib3.control.unofficial_tello import Tello

pygame.init()

drone = Tello()
joystick = pygame.joystick.Joystick(0)
app = Ursina()
drone_entity = Entity(model=load_model("./tello.obj"), texture=load_texture("tello-texture.png"), scale=1)
camera = EditorCamera()

def main():
    drone.connect()
    drone.takeoff()
    joystick.init()
    running = True
    while running:
        app.step()

        # Check for events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Get joystick values
        left_x = joystick.get_axis(0)
        left_y = -joystick.get_axis(1)
        right_x = joystick.get_axis(2)
        right_y = -joystick.get_axis(3)

        pitch = -drone.get_pitch()
        yaw = drone.get_yaw()
        roll = -drone.get_roll()
        print(pitch, yaw, roll)

        drone_entity.rotation_x = roll
        drone_entity.rotation_y = yaw
        drone_entity.rotation_z = pitch

        height = drone.get_height()

        drone_entity.position = (0, height / 100, 0)

        drone.send_rc_control(int(right_x * 100), int(right_y * 100), int(left_y * 100), int(left_x * 100))

    # Quit Pygame
    pygame.quit()

if __name__ == "__main__":
    main()