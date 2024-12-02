import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time

broker = "10.0.0.15"
port = 1883
topic = "esp32/norm_acceleration_x_without_gravity" 

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)
servo1.start(0)

def set_angle(angle):
    duty_cycle = 2 + (angle / 18)
    servo1.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)


def calculate_angle_from_acceleration(accX):
    max_acc = 10
    min_acc = -10

    normalized_angle = (accX - min_acc) * 180 / (max_acc - min_acc)

    if normalized_angle < 0:
        normalized_angle = 0
    elif normalized_angle > 180:
        normalized_angle = 180

    return normalized_angle


def on_message(client, userdata, msg):
    try:

        data = msg.payload.decode().strip() 
        accX = float(data) 
        print(f"Received acceleration X: {accX} m/s²")


        angle = calculate_angle_from_acceleration(accX)
        print(f"Calculated angle: {angle} degrees")


        set_angle(angle)
    except ValueError:
        print("Invalid data received!")


client = mqtt.Client()
client.on_message = on_message


client.connect(broker, port, 60)
client.subscribe(topic)

try:
    client.loop_forever()
except KeyboardInterrupt:
    servo1.stop()
    GPIO.cleanup()
    client.disconnect()
