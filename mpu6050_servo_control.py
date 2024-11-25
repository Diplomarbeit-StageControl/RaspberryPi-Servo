import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time

broker = "10.0.0.1"
port = 1883
topic = "esp32/movements"

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)  # Hier GPIO18 für PWM verwenden
servo1 = GPIO.PWM(18, 50)  # 50Hz PWM für den Servo
servo1.start(0)  # Starten mit 0% Duty Cycle (Servo in Ruheposition)
GPIO.setwarnings(False)

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    if rc == 0:
        client.subscribe(topic)
    else:
        print("Connection failed")

def on_message(client, userdata, msg):
    message = msg.payload.decode()
    print(f"Received message: {message}")

    try:
        data = message.split(',')
        if len(data) != 3:
            print("Error: Incorrect message format. Expected 3 values.")
            return

        x = float(data[0])
        y = float(data[1])
        z = float(data[2])

        # Berechne den Servo-Winkel
        angle = (x + 8) * 10
        angle = max(0, min(180, angle))

        print(f"Calculated angle: {angle}")

        # Berechne den Duty Cycle
        duty_cycle = (angle / 18) + 2.5  # Skaliere den Duty Cycle
        print(f"Calculated duty_cycle: {duty_cycle}")

        # Ändere den Duty Cycle des Servos
        servo1.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5)
        servo1.ChangeDutyCycle(0)

    except Exception as e:
        print(f"Error processing message: {e}")

# MQTT Client Setup
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Verbinde zum Broker und starte die Loop
client.connect(broker, port, 20)
client.enable_logger()  # Aktiviere Logging für Debugging
client.loop_forever()
