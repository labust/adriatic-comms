#! /usr/bin/python3
from http.client import HTTPConnection
import json
import cgi
import rospy
from std_msgs.msg import String
from labust_msgs.msg import NanomodemRequest, NanomodemPayload

_gesture_registry = {
    "d!" : "HOLD",
    "e!" : "LEFT",
    "f!" : "RIGHT",
    "10" : "FRONT",
    "11" : "BACK"
    "7!" : "MONITOR"
    "8!" : "SCOOTER_MODE"
    "9!" : "STANDBY"
    "b!" : "START_RECORDING"
    "c!" : "STOP_RECORDING"
    "a!" : "TAKE_PICTURE"
}

def send_gesture(gesture, args):
    destination = args.domain if args.domain else args.ip
    port = args.port

    if not destination:
        raise Exception("Server address not provided. Abort...")

    conn = HTTPConnection(destination, port)
    print("Open connection to ${destination}:${port}")

    conn.request("GET", "/?gesture=${gesture}")
    response = conn.getresponse()
    as_json = json.loads(response.msg)
    if response.status == "200 OK":
        print("Received response", response.msg)
    else:
        print("Failed response")


def nanomodem_callback(payload, args):
    decoded = None
    if payload:
        decoded = "".join([chr(item) for item in payload.msg])
        print("Received message: ", decoded)
        # return 
    else:
        print("No payload received")
        return
    
    gesture = _gesture_registry[decoded]
    send_gesture(gesture, args)



def run(args):
    rospy.init_node("nanomodem_listener")
    rospy.Subscriber(args.topic, NanomodemPayload, nanomodem_callback, callback_args=args)
    print("Node initialized")
    rospy.spin()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', dest="ip", type=str, default="127.0.0.1")
    parser.add_argument('--port', "-p", dest="port", type=int, default="12345")
    parser.add_argument("--domain", "-d", dest="domain", type=str, default="labust.ddnsfree.com")
    parser.add_argument("--topic", dest="topic", type=str, default="nanomodem_payload")
    parser.add_argument("--mode", dest="mode", type=str, default="topic")

    args = parser.parse_args()
    run(args)
