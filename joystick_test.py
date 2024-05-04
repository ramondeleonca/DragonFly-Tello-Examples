import pygame
import time

pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

def main():
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        val = joystick.get_axis(3)
        print(val)

        time.sleep(0.05)

    # Quit Pygame
    pygame.quit()

if __name__ == "__main__":
    main()