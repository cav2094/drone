import rclpy
from rclpy.node import Node
import socket
import cv2
import numpy as np
import struct
import time
from geometry_msgs.msg import Point

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.publisher = self.create_publisher(Point, 'face_position', 10)
        
        self.ip = '192.168.4.1'
        self.port = 5000
        
        # We must use TCP (SOCK_STREAM), not UDP!
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.get_logger().info(f"Connecting to AI-deck at {self.ip}:{self.port}...")
        try:
            self.sock.connect((self.ip, self.port))
            self.get_logger().info("Socket connected!")
            self.receive_stream()
        except Exception as e:
             self.get_logger().error(f"Failed to connect: {e}")

    def rx_bytes(self, size):
        """ Helper to read an exact amount of bytes from the TCP socket """
        data = bytearray()
        while len(data) < size:
            data.extend(self.sock.recv(size - len(data)))
        return data

    def receive_stream(self):
        while rclpy.ok():
            try:
                # 1. Read the Packet Header (4 bytes)
                packetInfoRaw = self.rx_bytes(4)
                length, routing, function = struct.unpack('<HBB', packetInfoRaw)

                # 2. Read the Image Header
                imgHeader = self.rx_bytes(length - 2)
                magic, width, height, depth, fmt, size = struct.unpack('<BHHBBI', imgHeader)

                if magic == 0xBC:
                    # 3. Read the actual raw image chunks as they come in
                    imgStream = bytearray()
                    while len(imgStream) < size:
                        packetInfoRaw = self.rx_bytes(4)
                        chunk_length, dst, src = struct.unpack('<HBB', packetInfoRaw)
                        chunk = self.rx_bytes(chunk_length - 2)
                        imgStream.extend(chunk)

                    # The streaming format is either 0 (RAW Bayer) or something else (JPEG)
                    if fmt == 0:
                        # 4A. Decode RAW Bayer Image
                        bayer_img = np.frombuffer(imgStream, dtype=np.uint8)
                        # AI Deck camera resolution is 244x324
                        bayer_img.shape = (244, 324)
                        image = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGR)
                    else:
                        # 4B. Decode JPEG Image
                        nparr = np.frombuffer(imgStream, np.uint8)
                        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                    if image is not None:
                        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.05, minNeighbors=2, minSize=(20, 20))
                        
                        for (x, y, w, h) in faces:
                            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                            
                            # 1. Calculate the center of the face
                            center_x = float(x + (w / 2))
                            center_y = float(y + (h / 2))
                            
                            # 2. Use 'Z' to hold the area of the box (width * height)
                            area = float(w * h)
                            
                            # 3. Create the Point message
                            msg = Point()
                            msg.x = center_x
                            msg.y = center_y
                            msg.z = area
                            
                            # 4. Broadcast the message to the Controller!
                            self.publisher.publish(msg)
                            self.get_logger().info(f"Published target: X={center_x}, Y={center_y}, Area={area}")
                            
                        # The AI Deck camera is tiny (324x244). Let's scale it up 3x!
                        h, w = image.shape[:2]
                        display_image = cv2.resize(image, (w*3, h*3), interpolation=cv2.INTER_NEAREST)
                        cv2.imshow("Drone Video Stream", display_image)
                        cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f"Stream error: {e}")
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
