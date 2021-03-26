#!/usr/bin/env python2

from http.server import BaseHTTPRequestHandler, HTTPServer
#from urllib.parse import parse_qs
import json
import cgi
import rospy
from std_msgs.msg import String
from labust_msgs.msg import NanomodemRequest

pub = None

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

        msg = NanomodemRequest()
        msg.id = 11
        msg.req_type = 3
        pub.publish(msg)
        return True


    def do_GET(self):
        self._set_headers()        
        #params = parse_qs(self.path[2:])
        # CHECK PARAMS
        params = { "test" : "nista"}
        print(params)
        if self._publish_to_ros(params):
            self.wfile.write(str.encode(
                json.dumps({'success': 1, 'test2': 'TEST2'}))
            )
        else:
            return json.dumps({'success': 0})
        
    def do_POST(self):
        ctype, _ = cgi.parse_header(self.headers['content-type'])
        
        # refuse to receive non-json content
        if ctype != 'application/json':
            self.send_response(400)
            self.end_headers()
            return
            
        # read the message and convert it into a python dictionary
        length = int(self.headers['content-length'])
        message = json.loads(self.rfile.read(length))
        
        # add a property to the object, just to mess with data
        message['received'] = 'ok'
        
        # send the message back
        self._set_headers()
        self.wfile.write(str.encode(json.dumps(message)))

def setup_ros(topic_name="nanomodem1_in"):
    global pub
    pub = rospy.Publisher(topic_name, NanomodemRequest, queue_size=10)
    rospy.init_node('adriatic-comms', anonymous=True)

def run(port=12345):
    setup_ros()
    server_address = ('', port)
    httpd = HTTPServer(server_address, Server)
    print ('Starting httpd on port %d...' % port)
    httpd.serve_forever()
    
if __name__ == "__main__":
    from sys import argv
    
    if len(argv) == 2:
        run(port=int(argv[1]))
    else:
        run()
