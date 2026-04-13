import rclpy
from rclpy.node import Node
import socket
import cv2
import numpy as np
import struct
import time
from geometry_msgs.msg import Point
from ultralytics import YOLO

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.model = YOLO('yolov8n.pt')  # Auto-downloads pretrained COCO weights

        # COCO classes we care about
        self.target_classes = [0, 2, 3, 5, 7, 4]  # person, car, motorcycle, bus, truck, airplane(drone)
        self.class_labels = {
            0: 'Person',
            2: 'Car',
            3: 'Motorcycle',
            5: 'Bus',
            7: 'Truck',
            4: 'Drone',
        }
        self.class_colors = {
            0: (0, 255, 0),    # Person    - green
            2: (0, 0, 255),    # Car       - red
            3: (255, 165, 0),  # Motorcycle - orange
            5: (255, 0, 255),  # Bus       - magenta
            7: (0, 255, 255),  # Truck     - yellow
            4: (255, 0, 0),    # Drone     - blue
        }

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
                        # Run YOLOv8 inference on all target classes
                        results = self.model(image, classes=self.target_classes, verbose=False)

                        for result in results:
                            for box in result.boxes:
                                x1, y1, x2, y2 = map(int, box.xyxy[0])
                                conf = float(box.conf[0])
                                cls = int(box.cls[0])

                                label = self.class_labels.get(cls, 'Unknown')
                                color = self.class_colors.get(cls, (255, 255, 255))

                                # Draw bounding box and label
                                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                                cv2.putText(image, f'{label} {conf:.2f}', (x1, y1 - 5),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

                                # Calculate center and area of the bounding box
                                center_x = float(x1 + (x2 - x1) / 2)
                                center_y = float(y1 + (y2 - y1) / 2)
                                area = float((x2 - x1) * (y2 - y1))

                                # Publish target position to controller
                                msg = Point()
                                msg.x = center_x
                                msg.y = center_y
                                msg.z = area
                                self.publisher.publish(msg)
                                self.get_logger().info(f"{label} detected: X={center_x:.1f}, Y={center_y:.1f}, Conf={conf:.2f}")

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
