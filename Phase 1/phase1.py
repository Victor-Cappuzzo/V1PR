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
#K_p = 1.1
#K_i = 0.05
#K_d = 0.7
K_p = 1.5
K_i = 0.0
K_d = 0.0
theta_setpoint = 0
theta_deadzone = 0.5 # Robot is considered "balanced" if within +/- this theta value
error_sum_max = 150
error_rate_max = 5

# Plotting variables
time_list = []
initial_time = time.time_ns()
setpoint_list = []
error_list = []
proportional_list = []
integral_list = []
derivative_list = []
control_effort_list = []

"""
t_prev = time.ticks_ms()
min_dt = 1e9
max_dt = 0
count = 0
sum_dt = 0
"""

"""
------------------------------------------------------------------------
MAIN PROGRAM
------------------------------------------------------------------------
"""
def main():
    
    # Wake up MPU
    mpu.wake()
    
    # Set motor speeds to 0 initially
    command_motor("left", "float", 0)
    command_motor("right", "float", 0)
    
    # Error variables
    error = 0
    error_sum = 0
    previous_error = 0
    error_rate = 0
    
    while True:
        
        """
        global t_prev
        global min_dt
        global max_dt
        global count
        global sum_dt
        
        t = time.ticks_ms()
        dt = (t - t_prev) / 1000.0
        t_prev = t
        min_dt = min(min_dt, dt)
        max_dt = max(max_dt, dt)
        sum_dt += dt
        count += 1
        if count % 200 == 0:
            print("dt avg/min/max = {:.4f}/{:.4f}/{:.4f} s".format(sum_dt/count, min_dt, max_dt))
        
        """
        
        start_time = time.time_ns()
        
        # Read MPU data
        gyro, accel = read_mpu()
    
        # Convert accelerometer data to angle
        new_theta = get_angle(accel)
        
        # Get new error, error sum, and error rate
        error = new_theta - theta_setpoint
        
        # If the robot is not at the setpoint (within some tolerance), calculate control effort
        if abs(error) >= theta_deadzone:
            
            # Calculate error sum, and limit to a range of values
            error_sum += error
            if abs(error_sum) > error_sum_max:
                error_sum = math.copysign(error_sum_max, error_sum)
                
            # Calculate error rate and limit to a range of values
            elapsed_time = float((time.time_ns() - start_time) / (10**9))
            error_rate = (error - previous_error) / elapsed_time
            if abs(error_rate) > error_rate_max:
                error_rate = math.copysign(error_rate_max, error_sum)
            
            #print("Error:", error, "| Error sum:", error_sum, "| Error rate:", error_rate)
        
            # Enter control loop
            control_effort = K_p * error + K_i * error_sum + K_d * error_rate
            
            # Convert control effort to a new speed
            new_speed = control_effort
            
            # Clip new speed off at 100%
            if abs(new_speed) > 100:
                
                new_speed = 100 * math.copysign(1.0, new_speed)
            
            if new_speed >= 0:
                command_motor("left", "forward", new_speed)
                command_motor("right", "forward", new_speed)
                
            else:
                command_motor("left", "backward", abs(new_speed))
                command_motor("right", "backward", abs(new_speed))
        
        else:
            command_motor("left", "float", 0)
            command_motor("right", "float", 0)
            
        """
        # Add datapoints to plot lists
        time_list.append(start_time - initial_time)
        error_list.append(error)
        setpoint_list.append(theta_setpoint)
        proportional_list.append(K_p * error)
        integral_list.append(K_i * error_sum)
        derivative_list.append(K_d * error_rate)
        control_effort_list.append(K_p * error + K_i * error_sum + K_d * error_rate)
        """
        #print("Error", error, "Setpoint", theta_setpoint, "Proportional", K_p*error, "Integral", K_i*error_sum, "Derivative", K_d*error_rate, "Control Effort", K_p * error + K_i * error_sum + K_d * error_rate)
        print("Error", error, "Setpoint", theta_setpoint, "Proportional", K_p*error)

"""
------------------------------------------------------------------------
OTHER FUNCTIONS
------------------------------------------------------------------------
"""
# ----------------------------------------------------------------------
# Controls a specific motor given a command and a speed
# ----------------------------------------------------------------------
def command_motor(motor, command, speed):
    
    # Set the motor speed
    set_speed(motor, speed)
    
    # If the command is forward
    if command == "forward":
        
        move_forward(motor)
    
    # If the command is backward
    elif command == "backward":
        
        move_backward(motor)
    
    # If the command is brake
    elif command == "brake":
        
        brake(motor)
    
    # If the command is float
    elif command == "float":
        
        float_motor(motor)
    
    # If the command is unknown
    else:
        
        print("Unknown command...")


# ----------------------------------------------------------------------
# Sets the speed of a given motor to a given PWM duty cycle %
# ----------------------------------------------------------------------
def set_speed(motor, speed):
    
    if motor == "left":
        PWM_1.duty_u16(int(speed*duty_max/100))
        
    if motor == "right":
        PWM_2.duty_u16(int(speed*duty_max/100))
    
    #print("Motor", motor, "speed set to:", str(speed),"%")


# ----------------------------------------------------------------------
# Rotates the motors forward
# Forward is defined as CCW looking down the left motor shaft
# Forward is defined as CW looking down the right motor shaft
# ----------------------------------------------------------------------
def move_forward(motor):
    
    if motor == "left":
    
        MOTOR_1_IN1_PIN.value(0)
        MOTOR_1_IN2_PIN.value(1)
    
    if motor == "right":
        
        MOTOR_2_IN1_PIN.value(1)
        MOTOR_2_IN2_PIN.value(0)
    
    #print("Moving", motor, "motor forward")
    

# ----------------------------------------------------------------------
# Rotates the given motor backwards
# Backward is defined as CW looking down the left motor shaft
# Backward is defined as CCW looking down the right motor shaft
# ----------------------------------------------------------------------
def move_backward(motor):
    
    if motor == "left":
        MOTOR_1_IN1_PIN.value(1)
        MOTOR_1_IN2_PIN.value(0)
    
    if motor == "right":
        MOTOR_2_IN1_PIN.value(0)
        MOTOR_2_IN2_PIN.value(1)
    
    #print("Moving", motor, "motor backward")


# ----------------------------------------------------------------------
# Stops the given motor
# ----------------------------------------------------------------------
def brake(motor):
    
    if motor == "left":
        MOTOR_1_IN1_PIN.value(0)
        MOTOR_1_IN2_PIN.value(0)
    
    if motor == "right":
        MOTOR_2_IN1_PIN.value(0)
        MOTOR_2_IN2_PIN.value(0)
    
    #print("Braking", motor, "motor")


# ----------------------------------------------------------------------
# Floats the given motor
# ----------------------------------------------------------------------
def float_motor(motor):
    
    if motor == "left":
        MOTOR_1_IN1_PIN.value(1)
        MOTOR_1_IN2_PIN.value(1)
    
    if motor == "right":
        MOTOR_2_IN1_PIN.value(1)
        MOTOR_2_IN2_PIN.value(1)
    
    #print("Floating", motor, "motor")
    

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
    #time.sleep(0.1)
    
    return gyro, accel


# ----------------------------------------------------------------------
# Converts MPU data to robot angle
#
# Pitch = atan2(-accel_X, sqrt(accel_Y^2 + accel_Z^2)) * 180 / pi
# Roll = atan2(-accel_Y, sqrt(accel_X^2 + accel_Z^2)) * 180 / pi
# ----------------------------------------------------------------------
def get_angle(accel):
    
    pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) * 180.0 / math.pi
    #print("Pitch angle: ", pitch)
    
    return pitch



main()
