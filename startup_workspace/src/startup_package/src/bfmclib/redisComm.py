import cv2
import numpy as np
import sys
import base64
#sys.path.insert(0, "/home/papafotit/.local/lib/python3.6/site-packages")
import redis


class RedisComm:

    def __init__(self):
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.det = self.r.pubsub()
        self.det.subscribe('detection-channel')

    def getMessage(self, channel):
        message = self.det.get_message()
        if message is not None:
            message = message['data']
            return message

    def publish(self, channel, image):
        retval, buffer = cv2.imencode('.jpg', image)
        image_as_text = base64.b64encode(buffer)
        self.r.publish(channel, image_as_text)

    def translate(self, message):
        text_image = message
        if type(text_image) is long:
            return 0
        else:
            img_binary = base64.b64decode(text_image)
            img_np = np.frombuffer(img_binary, dtype=np.uint8)
            img = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
            return img
