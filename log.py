import matplotlib.pyplot as plt
import numpy as np
import paho.mqtt.client as paho
import serial
import time

serdev = '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600)
mqttc = paho.Client()
log = open("log.txt", 'a')
# Settings for connection
host = "localhost"
topic= "Mbed"
port = 1883

# Callbacks
def on_connect(self, mosq, obj, rc):
    print("Connected rc: " + str(rc))

def on_message(mosq, obj, msg):
    print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
    print("Unsubscribed OK")

# Set callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe    

# Connect and subscribe
print("Connecting to " + host + "/" + topic)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic, 0)
# log file setting
current = time.ctime()+"\n"
log.write(current)
while 1:
    line = s.readline()
    print(line)
    line_de = line.decode()
    log.write(line_de)

    if line_de == "80661\r\n":
        print("identification data:")
        line = int(s.readline())
        print(line)
        log.write(str(line))
        mqttc.publish(topic, "identification data: "+str(line))
    elif line_de == "80662\r\n":
        print("matrix data:")
        line = s.readline().decode()
        print(line)
        log.write(line)
        mqttc.publish(topic, "matrix data: "+line)
    if line_de == "80663\r\n":
        print("ping data check result:")
        line = s.readline().decode()
        print(line)
        log.write(line)
        mqttc.publish(topic, "ping data: "+line)
    
    