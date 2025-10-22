import machine
from machine import Pin, UART, PWM, I2C
import time
from time import sleep_ms
import MPU6050
import math


"""
------------------------------------------------------------------------
PIN DEFINITIONS AND GLOBAL VARIABLES
------------------------------------------------------------------------
"""

# Bluetooth board pins
BLUETOOTH_TX_PIN = Pin(0)
BLUETOOTH_RX_PIN = Pin(1)

# Define UART object on UART0 ports
uart = UART(0, baudrate=9600, tx=BLUETOOTH_TX_PIN, rx=BLUETOOTH_RX_PIN)

# Gyroscope pins
GYRO_SCL_PIN = Pin(21)
GYRO_SDA_PIN = Pin(20)

# Define mpu object with I2C 0
i2c = I2C(0, scl=GYRO_SCL_PIN, sda=GYRO_SDA_PIN)
mpu = MPU6050.MPU6050(i2c)

# Motor controller pins
PWM_MIN = 0
PWM_MAX = 10000
pwm_freq = 5000
duty_min = 0
duty_max = 65535
MOTOR_1_PWM_PIN = Pin(2)
PWM_1 = PWM(MOTOR_1_PWM_PIN, freq=pwm_freq)
MOTOR_1_IN1_PIN = Pin(3, Pin.OUT)
MOTOR_1_IN2_PIN = Pin(4, Pin.OUT)
MOTOR_2_PWM_PIN = Pin(6)
PWM_2 = PWM(MOTOR_2_PWM_PIN, freq=pwm_freq)
MOTOR_2_IN1_PIN = Pin(7, Pin.OUT)
MOTOR_2_IN2_PIN = Pin(8, Pin.OUT)

# Motor encoder pins
MOTOR_1_ENCODER_A_PIN = Pin(11)
MOTOR_1_ENCODER_B_PIN = Pin(10)
MOTOR_2_ENCODER_A_PIN = Pin(12)
MOTOR_2_ENCODER_B_PIN = Pin(13)

# PID constants
K_p = 0
K_i = 0
K_d = 0


"""
------------------------------------------------------------------------
MAIN PROGRAM
------------------------------------------------------------------------
"""
def main():
    
    # Wake up MPU
    mpu.wake()
    
    while True:
        # Read MPU data
        gyro, accel = read_mpu()
    
        # Convert accelerometer data to angle
        get_angle(accel)
    
    # Enter control loop
    

"""
------------------------------------------------------------------------
OTHER FUNCTIONS
------------------------------------------------------------------------
"""
# ----------------------------------------------------------------------
# Sets the speed of the motors to a given PWM duty cycle %
# ----------------------------------------------------------------------
def set_speed(speed):
    
    PWM_1.duty_u16(int(speed*duty_max/100))
    PWM_2.duty_u16(int(speed*duty_max/100))
    print("Speed set to:", str(speed),"%")


# ----------------------------------------------------------------------
# Rotates the motors forward
# Forward is defined as CCW looking down the left motor shaft
# Forward is defined as CW looking down the right motor shaft
#
# Left motor --> 1
# Right motor --> 2
# ----------------------------------------------------------------------
def move_forward(motor):
    
    if motor == 1:
    
        MOTOR_1_IN1_PIN.value(0)
        MOTOR_1_IN2_PIN.value(1)
    
    if motor == 2:
        
        MOTOR_2_IN1_PIN.value(1)
        MOTOR_2_IN2_PIN.value(0)
    
    print("Moving motor #" + str(motor) + " forward...")
    

# ----------------------------------------------------------------------
# Rotates the given motor backwards
# Backward is defined as CW looking down the left motor shaft
# Backward is defined as CCW looking down the right motor shaft
#
# Left motor --> 1
# Right motor --> 2
# ----------------------------------------------------------------------
def move_backward(motor):
    
    if motor == 1:
        MOTOR_1_IN1_PIN.value(1)
        MOTOR_1_IN2_PIN.value(0)
    
    if motor == 2:
        MOTOR_2_IN1_PIN.value(0)
        MOTOR_2_IN2_PIN.value(1)
    
    print("Moving motor #" + str(motor) + " backward...")


# ----------------------------------------------------------------------
# Stops the given motor
#
# Left motor --> 1
# Right motor --> 2
# ----------------------------------------------------------------------
def brake(motor):
    
    if motor == 1:
        MOTOR_1_IN1_PIN.value(0)
        MOTOR_1_IN2_PIN.value(0)
    
    if motor == 2:
        MOTOR_2_IN1_PIN.value(0)
        MOTOR_2_IN2_PIN.value(0)
    
    print("Braking motor #" + str(motor))


# ----------------------------------------------------------------------
# Floats the given motor
#
# Left motor --> 1
# Right motor --> 2
# ----------------------------------------------------------------------
def float_motor(motor):
    
    if motor == 1:
        MOTOR_1_IN1_PIN.value(1)
        MOTOR_1_IN2_PIN.value(1)
    
    if motor == 2:
        MOTOR_2_IN1_PIN.value(1)
        MOTOR_2_IN2_PIN.value(1)
    
    print("Floating motor #" + str(motor))
    

# ----------------------------------------------------------------------
# Reads and returns MP6050 gyroscope + accelerometer
# Gyroscope data --> [X, Y, Z] deg/s
# Accelerometer data --> [X, Y, Z] m/s^2
# ----------------------------------------------------------------------
def read_mpu():
    
    # Read gyroscope data
    gyro = mpu.read_gyro_data()
    
    # Read accelerometer data
    accel = mpu.read_accel_data()
    
    # Output results to console
    gyro_str = str("X: " + str(gyro[0]) + ", Y: " + str(gyro[1]) + ", Z: " + str(gyro[2]))
    accel_str = str("X: " + str(accel[0]) + ", Y: " + str(accel[1]) + ", Z: " + str(accel[2]))
    #print("Gyro: " + gyro_str + " deg/s | Accel: " + accel_str + " m/s^2")
    
    # Debounce/filter
    time.sleep(0.1)
    
    return gyro, accel


# ----------------------------------------------------------------------
# Converts MPU data to robot angle
#
# Pitch = atan2(-accel_X, sqrt(accel_Y^2 + accel_Z^2)) * 180 / pi
# Roll = atan2(-accel_Y, sqrt(accel_X^2 + accel_Z^2)) * 180 / pi
# ----------------------------------------------------------------------
def get_angle(accel):
    
    pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) * 180.0 / math.pi
    print("Pitch angle: ", pitch)



main()