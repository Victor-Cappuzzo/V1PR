import numpy as np
import pygame
import math

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
m_cart = 1.0 # Cart mass [kg]
m_pend = 2 # Pendulum mass [kg]
length = 100 # Length of pendulum arm [m]
b = 100 # Drag coefficient
g = 9.81 # Gravitational constant [m/s^2]
time_step = 1/FPS
force_mag = 200 # Force applied when pressing keys [N]

# INITIAL CONDITIONS
cart_x = SCREEN_WIDTH/2 # Cart position
cart_v = 0 # Cart velocity
cart_a = 0 # Cart acceleration
pend_angle = math.pi # Pendulum angle, where pi = down and 0 = up [deg]
pend_angle_vel = 0 # Pendulum angle velocity [deg/s]


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
    cart_a = 0
    if keys[pygame.K_LEFT] or keys[pygame.K_a]:
        cart_a = -force_mag / m_cart
    if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
        cart_a = force_mag / m_cart

    # Physics
    # Update cart motion
    cart_v += cart_a * time_step
    cart_x += cart_v * time_step

    # Pendulum dynamics (simplified)
    # Equation: theta_dot = -(g/L)*sin(theta) + cart_accel*cos(theta)/L
    theta = pend_angle
    theta_dot = pend_angle_vel
    theta_ddot = -(g/length)*math.sin(theta) - (cart_a/length)*math.cos(theta)

    pend_angle_vel += theta_ddot * time_step
    pend_angle += theta_dot * time_step

    # Rendering
    screen.fill(WHITE)

    # Draw cart
    pend_x = cart_x + length*math.sin(pend_angle)
    pend_y = SCREEN_HEIGHT//2 + length*math.cos(pend_angle)
    cart_rect = pygame.Rect(cart_x - CART_WIDTH//2, SCREEN_HEIGHT//2, CART_WIDTH, CART_HEIGHT)
    pygame.draw.rect(screen, BLACK, cart_rect)

    # Draw pendulum
    pygame.draw.line(screen, RED, (cart_x, SCREEN_HEIGHT//2), (pend_x, pend_y), 4)
    pygame.draw.circle(screen, BLUE, (int(pend_x), int(pend_y)), 12)

    pygame.display.flip()
    clock.tick(FPS)

