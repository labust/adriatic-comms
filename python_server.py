#!/usr/bin/env python3

from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs
import json
import cgi
import rospy
from std_msgs.msg import String
from labust_msgs.msg import NanomodemRequest

pub = None

_gesture_registry = {
    "HOLD" : "3",
    "LEFT" : "7",
    "RIGHT" : "6",
    "FRONT" : "4",
    "BACK" : "5"
   #"MONITOR" : "7!"
   #"SCOOTER_MODE" : "8!"
   #"STANDBY" : "9!"
   #"START_RECORDING" : "b!"
   #"STOP_RECORDING" : "c!"
   #"TAKE_PICTURE" : "a!"
}

class Server(BaseHTTPRequestHandler):
    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        
    def _publish_to_ros(self, msg):
        global pub
        if not pub:
            print("Publisher not set!")
            return False

        gesture = _gesture_registry.get(msg, "3")
        request = NanomodemRequest()
        request.id = 11
        request.req_type = 2
        request.msg = bytearray(gesture)
        pub.publish(request)

    def do_GET(self):
        self._set_headers()        
        params = parse_qs(self.path[2:])
        print(params)
        gesture = params["gesture"][0]
        if not gesture:
            print("No gesture received")
            return
        # CHECK PARAMS
        print("received gesture: ", gesture)
        if self._publish_to_ros(gesture):
            self.wfile.write(str.encode(
                json.dumps({'success': 1, 'received_command': gesture }))
            )
        else:
            return json.dumps({'success': 0})
        
    # def do_POST(self):
    #     ctype, _ = cgi.parse_header(self.headers['content-type'])
        
    #     # refuse to receive non-json content
    #     if ctype != 'application/json':
    #         self.send_response(400)
    #         self.end_headers()
    #         return
            
    #     # read the message and convert it into a python dictionary
    #     length = int(self.headers['content-length'])
    #     message = json.loads(self.rfile.read(length))
        
    #     # add a property to the object, just to mess with data
    #     message['received'] = 'ok'
        
    #     # send the message back
    #     self._set_headers()
    #     self.wfile.write(str.encode(json.dumps(message)))

def setup_ros(topic_name="nanomodem_request"):
    global pub
    pub = rospy.Publisher(topic_name, NanomodemRequest, queue_size=10)
    rospy.init_node('adriatic_server', anonymous=False)

def run(port=12345):
    setup_ros()
    server_address = ('', port)
    httpd = HTTPServer(server_address, Server)
    print ('Starting httpd on port %d...' % port)
    httpd.serve_forever()
    
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', "-p", dest="port", type=int, default="12345")
    args = parser.parse_args()
    run(port=args.port)
