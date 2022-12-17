from flask import Flask, request
import paho.mqtt.publish as publish
from paho.mqtt.client import Client
import io
from json import dumps

plannerIP = "127.0.0.1"
plannerSdpOriginalReqPort = 7990
plannerSdpSegmentedReqPort = 7991

app = Flask(__name__)

def setup():
    client = Client("crawler-mqtt-api") # client ID "mqtt-test"
    #client.username_pw_set("crawler", "435FKDVpp48ddf")
    client.connect('127.0.0.1', 1883)
    return client

@app.route("/")
def crawler_api_welcome():
    return "Crawler manual control API"

@app.route("/stop")
def requestStop():
    client = setup()
    client.publish(topic="/crawler/cmd", payload="5", retain=False)
    return "{ 'result': 'true' }"

@app.route("/rst")
def requestReset():
    client = setup()
    client.publish(topic="/crawler/cmd", payload="6", retain=False)
    return "{ 'result': 'true' }"

@app.route("/speed/inc")
def requestIncreaseSpeed():
    client = setup()
    client.publish(topic="/crawler/cmd", payload="1", retain=False)
    return "{ 'result': 'true' }"

@app.route("/speed/dec")
def requestDecreaseSpeed():
    client = setup()
    client.publish(topic="/crawler/cmd", payload="2", retain=False)
    return "{ 'result': 'true' }"

@app.route("/speed/forward/<int:val>")
def requestSetSpeedForward(val: int):
    client = setup()
    p = chr(val)
    client.publish(topic="/crawler/cmd", payload=f"7{p}", retain=False)
    return "{ 'result': 'true' }"

@app.route("/speed/backward/<int:val>")
def requestSetSpeedBackward(val: int):
    client = setup()
    p = chr(val)
    client.publish(topic="/crawler/cmd", payload=f"8{p}", retain=False)
    return "{ 'result': 'true' }"


@app.route("/turn/right")
def requestIncreaseTurnRight():
    client = setup()
    client.publish(topic="/crawler/cmd", payload="3", retain=False)
    return "{ 'result': 'true' }"

@app.route("/turn/left")
def requestIncreaseTurnLeft():
    client = setup()
    client.publish(topic="/crawler/cmd", payload="4", retain=True)
    return "{ 'result': 'true' }"

@app.route("/turn/right/<int:val>")
def requestSetSteeringRight(val: int):
    client = setup()
    p = chr(val)
    client.publish(topic="/crawler/cmd", payload=f"9{p}", retain=False)
    return "{ 'result': 'true' }"

@app.route("/turn/left/<int:val>")
def requestSetSteeringLeft(val: int):
    client = setup()
    p = chr(val)
    client.publish(topic="/crawler/cmd", payload=f"A{p}", retain=False)
    return "{ 'result': 'true' }"


@app.route("/stream/original/sdp", methods = ['GET', 'POST'])
def originalVisionServiceSdp():
    if request.method == 'POST':
        payload = request.json
        client = setup()
        client.publish(topic="/stream/original/client-sdp", payload=str(payload['sdp']), retain=False)
        return "{ 'result': 'true' }"
    else:
        resp = {'sdp': None, 'type':'offer'}
        with open('/tmp/crawler_sdp_original.dat') as f:
            resp['sdp'] = f.read()
        return dumps(resp)

@app.route("/stream/segmented/sdp", methods = ['POST', 'GET'])
def segmentedVisionServiceSdp():
    if request.method == 'POST':
        payload = request.json
        client = setup()
        client.publish(topic="/stream/segmented/client-sdp", payload=str(payload['sdp']), retain=False)
        return "{ 'result': 'true' }"
    else:
        resp = {'sdp': None, 'type':'offer'}
        with open('/tmp/crawler_sdp_segmented.dat') as f:
            resp['sdp'] = f.read()
        return dumps(resp)

@app.route("/logging/vision/original", methods = ['GET'])
def originalVisionLogToFileStart():
    client = setup()
    client.publish(topic="/stream/original/log", payload=str("start"), retain=False)
    return "{ 'result': 'true' }"

@app.route("/logging/vision/original", methods = ['DEL', 'DELETE'])
def originalVisionLogToFileStop():
    client = setup()
    client.publish(topic="/stream/original/log", payload=str("stop"), retain=False)
    return "{ 'result': 'true' }"

@app.route("/logging/vision/segmented", methods = ['GET'])
def segmentedVisionLogToFileStart():
    client = setup()
    client.publish(topic="/stream/segmented/log", payload=str("start"), retain=False)
    return "{ 'result': 'true' }"

@app.route("/logging/vision/segmented", methods = ['DEL', 'DELETE'])
def segmentedVisionLogToFileStop():
    client = setup()
    client.publish(topic="/stream/segmented/log", payload=str("stop"), retain=False)
    return "{ 'result': 'true' }"

@app.route("/logging/vision/og", methods = ['GET'])
def ogVisionLogToFileStart():
    client = setup()
    client.publish(topic="/stream/og/log", payload=str("start"), retain=False)
    return "{ 'result': 'true' }"

@app.route("/logging/vision/og", methods = ['DEL', 'DELETE'])
def oglVisionLogToFileStop():
    client = setup()
    client.publish(topic="/stream/og/log", payload=str("stop"), retain=False)
    return "{ 'result': 'true' }"    