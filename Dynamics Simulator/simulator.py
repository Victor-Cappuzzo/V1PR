import numpy as np
import pygame
import math
import time

# SCREEN VARIABLES
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
SCREEN_BORDER_THICKNESS = 5
FPS = 120
CART_WIDTH = 80
CART_HEIGHT = 40

# COLORS
RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,0,255)
WHITE = (255,255,255)
BLACK = (0,0,0)

# PHYSICAL VARIABLES
m_cart = 0.5 # Cart mass [kg]
m_pend = 2.0 # Pendulum mass [kg]
length = 100 # Length of pendulum arm [m]
b = 100 # Drag coefficient
g = 9.81 # Gravitational constant [m/s^2]
time_step = 1/FPS
force_mag = 250 # Force applied when pressing keys [N]

# INITIAL CONDITIONS
cart_x = SCREEN_WIDTH/2 # Cart position
cart_v = 0 # Cart velocity
cart_a = 0 # Cart acceleration
pend_angle = 0 # Pendulum angle, where pi = down and 0 = up [deg]
pend_angle_vel = 0 # Pendulum angle velocity [deg/s]

# PID CONSTANTS
K_p = 1300
K_i = 600
K_d = 500
error_sum = 0 # For integration control
previous_error = 0 # For derivative control
theta_setpoint = 0 # Pendulum upright

# User PID select
PID_index = 0
PID_array = []
PID_array.append(K_p)
PID_array.append(K_i)
PID_array.append(K_d)
PID_names = ["Kp", "Ki", "Kd"]
PID_step = 100
debounce_time = 0.5


# pygame setup
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT)) # set screen width and height
clock = pygame.time.Clock() # get the game clock
running = True # determines if the game is running or not



# while the game is running
while running:
    pygame.event.set_grab(True)

    # check if user quit the game
    for event in pygame.event.get():

        # see if user clicked the X window button to quit
        if event.type == pygame.QUIT:
            running = False

        # see if the user clicked ESCAPE to quit
        if pygame.key.get_pressed()[pygame.K_ESCAPE]:
            running = False

    # Check if a key is clicked
    keys = pygame.key.get_pressed()
    disturbance = 0
    if keys[pygame.K_LEFT] or keys[pygame.K_a]:
        disturbance = -force_mag / m_cart
    if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
        disturbance = force_mag / m_cart

    # Check if user wants to change the PID values
    if keys[pygame.K_UP]:
        PID_array[PID_index] += PID_step
        print(PID_names[PID_index], "increased to:", PID_array[PID_index])
        time.sleep(debounce_time)
    if keys[pygame.K_DOWN]:
        PID_array[PID_index] -= PID_step
        print(PID_names[PID_index], "decreased to:", PID_array[PID_index])
        time.sleep(debounce_time)
    if keys[pygame.K_p]:
        PID_index = 0
        print("Controlling Kp")
        time.sleep(debounce_time)
    if keys[pygame.K_i]:
        PID_index = 1
        print("Controlling Ki")
        time.sleep(debounce_time)
    if keys[pygame.K_d]:
        PID_index = 2
        print("Controlling Kd")
        time.sleep(debounce_time)
    if keys[pygame.K_ESCAPE]:
        print("Final PID values: Kp =", K_p,"| Ki = ", K_i, "| K_d = ", K_d)
    
    # Update PID values
    K_p,K_i,K_d = PID_array

    # Check if user wants to reset the simulation
    if keys[pygame.K_r]:
        cart_x = SCREEN_WIDTH/2
        cart_v = 0
        cart_a = 0
        pend_angle = 0
        pend_angle_vel = 0
        error_sum = 0
        previous_error = 0
        theta_setpoint = 0

    # PID Control Loop
    error = theta_setpoint - pend_angle
    error_sum += error * time_step
    error_rate = (error - previous_error) / time_step

    pid_accel = K_p*error + K_i*error_sum + K_d*error_rate
    previous_error = error

    # Add in disturbance force
    cart_a = pid_accel + disturbance

    # Physics
    # Update cart motion
    cart_v += cart_a * time_step
    cart_x += cart_v * time_step

    # Pendulum dynamics (simplified)
    # Equation: theta_dot = (g/L)*sin(theta) + cart_accel*cos(theta)/L
    theta = pend_angle
    theta_dot = pend_angle_vel
    theta_ddot = (g/length)*math.sin(theta) + (cart_a/length)*math.cos(theta)

    pend_angle_vel += theta_ddot * time_step
    pend_angle += theta_dot * time_step

    # Rendering
    screen.fill(WHITE)

    # Draw cart
    pend_x = cart_x - length*math.sin(pend_angle)
    pend_y = SCREEN_HEIGHT//2 - length*math.cos(pend_angle)
    cart_rect = pygame.Rect(cart_x - CART_WIDTH//2, SCREEN_HEIGHT//2, CART_WIDTH, CART_HEIGHT)
    pygame.draw.rect(screen, BLACK, cart_rect)

    # Draw pendulum
    pygame.draw.line(screen, RED, (cart_x, SCREEN_HEIGHT//2), (pend_x, pend_y), 4)
    pygame.draw.circle(screen, BLUE, (int(pend_x), int(pend_y)), 12)

    # Draw target line (upright reference)
    pygame.draw.line(screen, (100, 100, 100), (cart_x, SCREEN_HEIGHT//2), (cart_x, SCREEN_HEIGHT//2 - length), 1)

    # Display info
    font = pygame.font.SysFont(None, 24)
    text = font.render(f"PID accel: {pid_accel:.2f} | Ext force: {disturbance:.2f}", True, (0, 0, 0))
    screen.blit(text, (20, 20))
    PID_text = font.render(f"PID Values: Kp = {K_p:.2f} | Ki = {K_i:.2f} | Kd = {K_d:.2f}", True, (0, 0, 0))
    screen.blit(PID_text, (20, 40))

    pygame.display.flip()
    clock.tick(FPS)

