import pygame
import socket
import time

AXIS_LEFT_STICK_X = 0
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 3
AXIS_RIGHT_STICK_Y = 2
TRIGGER_RIGHT = 5
TRIGGER_LEFT = 4
# Labels for DS4 controller buttons
# # Note that there are 14 buttons (0 to 13 for pygame, 1 to 14 for Windows setup)
BUTTON_B = 1
BUTTON_Y = 3
BUTTON_A = 0
BUTTON_X = 2
GEARUP = 5
GEARDOWN = 4
BUTTON_L2 = 7
BUTTON_R2 = 8
BUTTON_SHARE = 8
BUTTON_OPTIONS = 6

BUTTON_LEFT_STICK = 10
BUTTON_RIGHT_STICK = 11

D_PAD_UP = 13
D_PAD_DOWN = 14
LEFT_ARROW = 13
RIGHT_ARROW = 14
BUTTON_PS = 5
BUTTON_PAD = 15
GEARUP_toggle = True
GEARDOWN_toggle = True
GD = 0
GU = 0
Gear = 0
A = 0
trigger = 0
resetValue = 0
# Debouncing time in milliseconds
debounce_time = 200
driveMode=0

# Initial Count

Gear = 0

# Timestamp of the last button press
last_press_time = 0


class TextPrint:

    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 30)

    def tprint(self, screen, text):
        text_bitmap = self.font.render(text, True, (192, 192, 192))
        screen.blit(text_bitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 20

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 20


def map(value, fromLow, fromHigh, toLow, toHigh):
    # Calculate the scaled value
    scaled_value = (value - fromLow) * (toHigh - toLow) / \
        (fromHigh - fromLow) + toLow
    # Return the scaled value
    return round(scaled_value)


pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("RM & Drive Controls")
pygame.joystick.init()
controller = pygame.joystick.Joystick(0)
print("Joystick Successfully sected")
controller.init()
output_string = " M{Gear}X{left_joystick_x}Y{left_joystick_y}P{right_joystick_x}Q{right_joystick_y}A{A}S{trigger}R{resetValue}D{driveMode}E"
# Set up the socket
# HOST = '192.168.137.250'  # The host IP address
HOST = "10.0.0.11"
# HOST = "127.0.0.1"
PORT = 5005      # The port to listen on
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    addr = (HOST, PORT)
    # s.connect(addr)
    # s, addr1 = s.accept()
    text_print = TextPrint()
    pygame.event.pump()

    while True:
        screen.fill((0, 0, 0))
        text_print.reset()
        text_print.tprint(screen, f"Y-Pitch Down")
        # text_print.indent()
        text_print.tprint(screen, f"A-Pitch Up")
        # text_print.indent()
        text_print.tprint(screen, f"B-Right Roll")
        # text_print.indent()
        text_print.tprint(screen, f"X-Left Roll")
        # text_print.indent()
        # text_print.tprint(screen, f"M-{Gear}")
        # text_print.indent()
        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()
        # Limit to 30 frames per second.

        print('connected by', addr)

        text_print.tprint(screen, f"Connected to {addr} ")
        # Set up a timer and interval to send data
        timer = 0
        interval = 10  # Send data every 0.1 seconds
        running = True
        pygame.key.get_focused()

        while running:
            # Check for events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    pygame.quit()
                    exit()
                elif event.type == pygame.KEYDOWN:
                     if event.key == pygame.K_SPACE:
                        if driveMode == 0:
                            driveMode=1
                        else:
                            driveMode=0
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BUTTON_Y:
                        A = 1
                        print("A has been pressed")
                    elif event.button == BUTTON_A:
                        print("B has been pressed")
                        A = 2
                    elif event.button == BUTTON_B:
                        print("X has been pressed")
                        A = 3
                    elif event.button == BUTTON_X:
                        print("Y has been pressed")
                        A = 4
                elif event.type == pygame.JOYHATMOTION:
                    if event.value[1] == 1:
                        print("D_PAD_UP has been pressed")
                        A = 5
                    elif event.value[1] == -1:
                        print("D_PAD_DOWN has been pressed")
                        A = 6
                    elif event.value[0] == 1:
                        print("D_PAD_Right has been pressed")
                        A = 7
                    elif event.value[0] == -1:
                        print("D_PAD_LEFT has been pressed")
                        resetValue = 1
                    elif event.value[1] == 0:
                        A = 0
                        resetValue = 0
                elif event.type == pygame.JOYBUTTONUP:
                    # if event.key == pygame.K_1 or event.key == pygame.K_2:
                    A = 0
            # Update the timer and send data
            timer += 1 / 60  # 1/60 seconds have passed
            if timer >= interval:
                # The interval has passed, reset the timer and send data
                timer = 0
                R1 = controller.get_button(GEARUP)
                L1 = controller.get_button(GEARDOWN)
                # Y=controller.get_button(BUTTON_X)
                # A=controller.get_button(BUTTON_Y)
                # B=controller.get_button(BUTTON_A)
                # X=controller.get_button(BUTTON_B)

                if R1:
                    # Check for debouncing
                    if pygame.time.get_ticks() - last_press_time > debounce_time:
                        if Gear < 9:
                            Gear += 1
                            last_press_time = pygame.time.get_ticks()
                if L1:
                    # Check for debouncing
                    if pygame.time.get_ticks() - last_press_time > debounce_time:
                        if Gear > 0:
                            Gear -= 1
                            last_press_time = pygame.time.get_ticks()

                pygame.event.pump()
                left_joystick_0 = controller.get_axis(AXIS_LEFT_STICK_X)
                left_joystick_x_0 = int(
                    map(left_joystick_0, -1, 1, 1023+100, -1023-100))
                left_joystick_x = str(left_joystick_x_0)
                left_joystick_y_0 = (map(controller.get_axis(
                    AXIS_LEFT_STICK_Y), -1, 1, -1023, 1023))
                left_joystick_y = str(left_joystick_y_0)
                right_joystick_x_0 = (
                    map(controller.get_axis(AXIS_RIGHT_STICK_X), -1, 1, 10, -10))
                right_joystick_x = str(right_joystick_x_0)
                right_joystick_y_0 = (
                    map(controller.get_axis(AXIS_RIGHT_STICK_Y), -1, 1, 10, -10))
                right_joystick_y = str(right_joystick_y_0)

                if int(trigger) >= 0:
                    right_trigger_axis = (
                        map(controller.get_axis(TRIGGER_RIGHT), -1, 1, 0, 10))
                    trigger = str(right_trigger_axis)
                if int(trigger) <= 0:
                    left_trigger_axis = (
                        map(controller.get_axis(TRIGGER_LEFT), -1, 1, 0, -10))
                    trigger = str(left_trigger_axis)
                data = output_string.format(
                    Gear=Gear, left_joystick_x=left_joystick_x, left_joystick_y=left_joystick_y, right_joystick_x=right_joystick_x, right_joystick_y=right_joystick_y, A=A, trigger=trigger, resetValue=resetValue, driveMode=driveMode, E=0)
                print(data)
                s.sendto(data.encode(), (addr))

# Quit pygame
pygame.quit()
# p=right joystick x-axis[-5,5], q=right joystick y-axis [-5,5]
