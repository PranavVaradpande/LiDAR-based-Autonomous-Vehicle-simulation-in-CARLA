import pygame
import os
import time

def main():
    pygame.init()
    pygame.joystick.init()

    # Check if a controller is plugged in
    if pygame.joystick.get_count() == 0:
        print("ERROR: No joystick detected. Please plug it in and try again.")
        return

    # Initialize the first detected controller
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    name = joystick.get_name()
    num_axes = joystick.get_numaxes()
    num_buttons = joystick.get_numbuttons()
    num_hats = joystick.get_numhats()

    try:
        while True:
            # Pygame requires this to update the controller's state internally
            pygame.event.pump()

            # Clear the terminal screen for a clean dashboard look (Windows: 'cls')
            os.system('cls' if os.name == 'nt' else 'clear')

            print(f"=== CONTROLLER TESTER: {name} ===")
            print(f"Total Detected -> Axes: {num_axes} | Buttons: {num_buttons} | Hats: {num_hats}")
            print("Move your sticks/triggers and press buttons. Press CTRL+C to quit.\n")

            # --- 1. Display Axes (Sticks and Triggers) ---
            print("[ AXES (Range: -1.00 to 1.00) ]")
            for i in range(num_axes):
                axis_val = joystick.get_axis(i)
                # Adding a little marker '*' if the axis is being moved
                marker = " <--" if abs(axis_val) > 0.1 else ""
                print(f"  Axis {i}: {axis_val:>6.2f}{marker}")

            # --- 2. Display Buttons ---
            print("\n[ BUTTONS (0 = Released, 1 = Pressed) ]")
            for i in range(num_buttons):
                button_val = joystick.get_button(i)
                marker = "[X]" if button_val else "[ ]"
                # Print 4 buttons per line for readability
                print(f"  Btn {i:02d}: {marker}", end="   " if (i + 1) % 4 != 0 else "\n")
            if num_buttons % 4 != 0: print() # Newline if not perfectly divisible by 4

            # --- 3. Display Hats (D-Pad) ---
            print("\n[ HATS / D-PAD (X, Y) ]")
            if num_hats == 0:
                print("  No hats detected.")
            for i in range(num_hats):
                hat_val = joystick.get_hat(i)
                print(f"  Hat {i}: {hat_val}")

            # Refresh rate (~10 times a second)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nExiting Joystick Tester...")
    finally:
        pygame.quit()

if __name__ == '__main__':
    main()