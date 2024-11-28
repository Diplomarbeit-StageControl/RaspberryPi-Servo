import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time

broker = "10.0.0.13"
port = 1883
topic = "servo/data" 

GPIO.setmode(GPIO.BCM)
GPIO.setup(11, GPIO.OUT)
pwm = GPIO.PWM(11, 50)
pwm.start(0)

def set_angle(angle):
    duty_cycle = angle / 18 + 2
    GPIO.output(11, True)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    GPIO.output(11, False)
    pwm.ChangeDutyCycle(0)

def on_message(client, userdata, msg):
    try:
        angle = float(msg.payload.decode())
        print(f"Received angle: {angle}")
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
    pwm.stop()
    GPIO.cleanup()
    client.disconnect()
