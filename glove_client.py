from http.client import HTTPConnection
#from urllib.parse import parse_qs
import json
import cgi
import rospy
from std_msgs.msg import String
from labust_msgs.msg import NanomodemRequest, NanomodemPayload

_gesture_registry = {
    "id_1" : "LEFT",
    "id_2" : "RIGHT",
    "id_3" : "UP",
    "id_4" : "DOWN",
    "id_5" : "HOLD"
}


def nanomodem_callback(payload):
    destination = args.ip if args.ip else args.domain
    port = args.port

    if payload:
        print("payload type: ", payload.msg_type)
        print("payload msg: ", payload.msg)
        decoded = "".join([chr(item) for item in payload.msg])
        print("payload msg decoded: ", decoded)
    else:
        print("no payload received")
        return

    if not destination:
        raise Exception("Server address not provided. Abort...")

    conn = HTTPConnection(destination, port)
    print("Open connection to ${destination}:${port}")

    gesture = _gesture_registry["id_1"]
    amount = 1
    conn.request("GET", "/?gesture=${gesture}&amount=${amount}")
    response = conn.getresponse()
    as_json = json.loads(response.msg)
    if response.status == "200 OK":
        print("Received response", response.msg)
    else:
        print("Failed response")



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
    parser.add_argument("--domain", "-d", dest="domain", type=str, default="")
    parser.add_argument("--topic", dest="topic", type=str, default="nanomodem_in1")

    args = parser.parse_args()
    run(args)