import pygame
from djitellopy import Tello

pygame.init()

drone = Tello()
joystick = pygame.joystick.Joystick(0)

def main():
    drone.connect()
    drone.takeoff()
    joystick.init()
    running = True
    while running:
        # Check for events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Get joystick values
        left_x = joystick.get_axis(0)
        left_y = -joystick.get_axis(1)
        right_x = joystick.get_axis(2)
        right_y = -joystick.get_axis(3)

        drone.send_rc_control(int(right_x * 100), int(right_y * 100), int(left_y * 100), int(left_x * 100))

    # Quit Pygame
    pygame.quit()

if __name__ == "__main__":
    main()