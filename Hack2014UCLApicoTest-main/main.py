from connections import connect_mqtt, connect_internet
from time import sleep
from constants import ssid, wlanpass, mqtt_server, mqtt_user, mqtt_pass
from machine import Pin
from dht import DHT11
import ujson
from hcsr04 import HCSR04
from machine import Pin, PWM
import utime

sensor_pin = Pin(15, Pin.IN, Pin.PULL_UP)
sensor = DHT11(sensor_pin)
USsensor = HCSR04(trigger_pin=17, echo_pin=16, echo_timeout_us=1000000)
# Define the pins connected to the servos
servo1_pin = 18
servo2_pin = 19

servo1 = PWM(Pin(servo1_pin), freq=50)  # Initialize PWM on the servo1 pin with a frequency of 50Hz
servo2 = PWM(Pin(servo2_pin), freq=50)  # Initialize PWM on the servo2 pin with a frequency of 50Hz

# Servo limits and pulse width parameters
MIN_ANGLE = -90
MAX_ANGLE = 135
ANGLE_STEP = 15
PULSE_WIDTH_MIN = 1000  # Pulse width for 0 degrees
PULSE_WIDTH_MAX = 2000  # Pulse width for 90 degrees
PULSE_WIDTH_RANGE = PULSE_WIDTH_MAX - PULSE_WIDTH_MIN
ANGLE_RANGE = MAX_ANGLE - MIN_ANGLE

# Initialize the current angles to 0 degrees
current_angle_servo1 = 0
current_angle_servo2 = 0

pwm_freq = 1000  # PWM frequency
speed = 60000
# Defining motor pins
# Motor Controller 1
# OUT1  and OUT2
# In1 = Pin(7, Pin.OUT)
# In2 = Pin(6, Pin.OUT)
# 
# # OUT3  and OUT4
# In3 = Pin(5, Pin.OUT)
# In4 = Pin(4, Pin.OUT)
# 
# # Motor Controller 2
# # OUT1  and OUT2
# In5 = Pin(11, Pin.OUT)
# In6 = Pin(10, Pin.OUT)
# 
# # OUT3  and OUT4
# In7 = Pin(9, Pin.OUT)
# In8 = Pin(8, Pin.OUT)

# Motor Controller 1
In1 = PWM(Pin(7), freq=pwm_freq)
In2 = PWM(Pin(6), freq=pwm_freq)
In3 = PWM(Pin(5), freq=pwm_freq)
In4 = PWM(Pin(4), freq=pwm_freq)

# Motor Controller 2
In5 = PWM(Pin(11), freq=pwm_freq)
In6 = PWM(Pin(10), freq=pwm_freq)
In7 = PWM(Pin(9), freq=pwm_freq)
In8 = PWM(Pin(8), freq=pwm_freq)

def move_forward():
    In1.duty_u16(speed)
    In2.duty_u16(0)
    In3.duty_u16(speed)
    In4.duty_u16(0)
    In5.duty_u16(0)
    In6.duty_u16(speed)
    In7.duty_u16(0)
    In8.duty_u16(speed)

def move_backward():
    In1.duty_u16(0)
    In2.duty_u16(speed)
    In3.duty_u16(0)
    In4.duty_u16(speed)
    In5.duty_u16(speed)
    In6.duty_u16(0)
    In7.duty_u16(speed)
    In8.duty_u16(0)

def turn_right():
    In1.duty_u16(speed)
    In2.duty_u16(0)
    In3.duty_u16(speed)
    In4.duty_u16(0)
    In5.duty_u16(speed)
    In6.duty_u16(0)
    In7.duty_u16(speed)
    In8.duty_u16(0)

def turn_left():
    In1.duty_u16(0)
    In2.duty_u16(speed)
    In3.duty_u16(0)
    In4.duty_u16(speed)
    In5.duty_u16(0)
    In6.duty_u16(speed)
    In7.duty_u16(0)
    In8.duty_u16(speed)

def stop():
    In1.duty_u16(0)
    In2.duty_u16(0)
    In3.duty_u16(0)
    In4.duty_u16(0)
    In5.duty_u16(0)
    In6.duty_u16(0)
    In7.duty_u16(0)
    In8.duty_u16(0)
    
# def move_forward():
#     In1.high()
#     In2.low()
#     In3.high()
#     In4.low()
#     In5.low()
#     In6.high()
#     In7.low()
#     In8.high()
# 
# def move_backward():
#     In1.low()
#     In2.high()
#     In3.low()
#     In4.high()
#     In5.high()
#     In6.low()
#     In7.high()
#     In8.low()
# 
# def turn_right():
#     In1.high()
#     In2.low()
#     In3.high()
#     In4.low()
#     In5.high()
#     In6.low()
#     In7.high()
#     In8.low()
# 
# def turn_left():
#     In1.low()
#     In2.high()
#     In3.low()
#     In4.high()
#     In5.low()
#     In6.high()
#     In7.low()
#     In8.high()
# 
# def stop():
#     In1.low()
#     In2.low()
#     In3.low()
#     In4.low()
#     In5.low()
#     In6.low()
#     In7.low()
#     In8.low()


def angle_to_pulse_width(angle):
    """Convert angle to pulse width in microseconds."""
    return PULSE_WIDTH_MIN + (angle / ANGLE_RANGE) * PULSE_WIDTH_RANGE

def set_angle(servo, angle):
    """Set the servo motor to the specified angle within limits."""
    if angle < MIN_ANGLE:
        angle = MIN_ANGLE
    elif angle > MAX_ANGLE:
        angle = MAX_ANGLE
    pulse_width = angle_to_pulse_width(angle)
    servo.duty_u16(int(pulse_width * 65535 / 20000))  # Convert pulse width to duty cycle

def rotate_clockwise_servo1():
    """Rotate the servo1 motor 15 degrees clockwise."""
    global current_angle_servo1
    new_angle = current_angle_servo1 + ANGLE_STEP
    set_angle(servo1, new_angle)
    current_angle_servo1 = new_angle
    print("Servo1 rotated clockwise to", new_angle, "degrees")
    utime.sleep(1)  # Wait for the servo to reach the position

def rotate_counterclockwise_servo1():
    """Rotate the servo1 motor 15 degrees counterclockwise."""
    global current_angle_servo1
    new_angle = current_angle_servo1 - ANGLE_STEP
    set_angle(servo1, new_angle)
    current_angle_servo1 = new_angle
    print("Servo1 rotated counterclockwise to", new_angle, "degrees")
    utime.sleep(1)  # Wait for the servo to reach the position

def rotate_clockwise_servo2():
    """Rotate the servo2 motor 15 degrees clockwise."""
    global current_angle_servo2
    new_angle = current_angle_servo2 + ANGLE_STEP
    set_angle(servo2, new_angle)
    current_angle_servo2 = new_angle
    print("Servo2 rotated clockwise to", new_angle, "degrees")
    utime.sleep(1)  # Wait for the servo to reach the position

def rotate_counterclockwise_servo2():
    """Rotate the servo2 motor 15 degrees counterclockwise."""
    global current_angle_servo2
    new_angle = current_angle_servo2 - ANGLE_STEP
    set_angle(servo2, new_angle)
    current_angle_servo2 = new_angle
    print("Servo2 rotated counterclockwise to", new_angle, "degrees")
    utime.sleep(1)  # Wait for the servo to reach the position

# Function to handle an incoming message
def cb(topic, msg):
    # print(f"Topic: {topic}, Message: {msg}")
    if topic == b"direction":
        if msg == b"forward":
            print("Message from direction", msg)
            move_forward()
        elif msg == b"backward":
            print("Message from direction", msg)
            move_backward()
        elif msg == b"left":
            print("Message from direction", msg)
            turn_left()
        elif msg == b"right":
            print("Message from direction", msg)
            turn_right()
        elif msg == b"stop":
            print("Message from direction", msg)
            stop()
            
    elif topic == b"arm":
        if msg == b"armup":
            print("Message from arm",msg)
            rotate_counterclockwise_servo2()
        elif msg == b"armdown":
            print("Message from arm",msg)
            rotate_clockwise_servo2()
    
    elif topic == b"pinch":
        if msg == b"handopen":
            print("Message from Pinch",msg)
            rotate_counterclockwise_servo1()
        elif msg == b"handclose":
            print("Message from Pinch",msg)
            rotate_clockwise_servo1()

def main():
    try:
        connect_internet(ssid, wlanpass)
        client = connect_mqtt(mqtt_server, mqtt_user, mqtt_pass)

        client.set_callback(cb)
        client.subscribe("direction")
        client.subscribe("arm")
        client.subscribe("pinch")
        cycles = 0

        while True:
            client.check_msg()
            
            # Data payload
            
            if cycles == 200:
                sensor.measure()
                temp = sensor.temperature()
                humidity = sensor.humidity()
                distance = USsensor.distance_cm()
                
                data = {
                    "temp": temp,
                    "humidity": humidity
                }
                print("temp:", temp)
                print("humidity", humidity)
                print(f'Distance: {distance:.2f} cm')
                message = ujson.dumps(data)
            
                client.publish("temp&&humidity", message)
                client.publish("ultrasonic", str(distance))
                cycles = 0
            
            cycles += 1
            sleep(.01)

    except KeyboardInterrupt:
        print('keyboard interrupt')
        

if __name__ == "__main__":
    main()



