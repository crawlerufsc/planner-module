from flask import Flask
import paho.mqtt.publish as publish
from paho.mqtt.client import Client

app = Flask(__name__)

def setup():
    client = Client("crawler-mqtt-api") # client ID "mqtt-test"
    #client.username_pw_set("crawler", "435FKDVpp48ddf")
    client.connect('127.0.0.1', 1883)
    return client

@app.route("/")
def crawler_api_welcome():
    return "Crawler manual control API"

@app.route("/speed/inc")
def requestIncreaseSpeed():
    client = setup()
    client.publish(topic="/crawler/cmd", payload="1", retain=True)
    return "OK"
    

@app.route("/speed/dec")
def requestDecreaseSpeed():
    client = setup()
    client.publish(topic="/crawler/cmd", payload="2", retain=True)
    return "OK"

@app.route("/turn/right")
def requestIncreaseTurnRight():
    client = setup()
    client.publish(topic="/crawler/cmd", payload="3", retain=True)
    return "OK"

@app.route("/turn/left")
def requestIncreaseTurnLeft():
    client = setup()
    client.publish(topic="/crawler/cmd", payload="4", retain=True)
    return "OK"

