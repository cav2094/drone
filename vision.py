import rclpy
from rclpy.node import Node
import socket
import cv2
import numpy as np
import struct
import time
from geometry_msgs.msg import Point
from ultralytics import YOLO


class KalmanTracker:
    """
    Tracks a single target using a Kalman filter with a constant velocity model.

    State vector:    [cx, cy, area, vcx, vcy, varea]  (position + velocity)
    Measurement:     [cx, cy, area]                   (from YOLO bounding box)

    When a detection is available  -> correct() then predict()
    When no detection (occlusion)  -> predict() only, up to MAX_MISSING_FRAMES
    """

    MAX_MISSING_FRAMES = 10  # how long to coast before giving up

    def __init__(self):
        # 6 state variables, 3 measured variables
        self.kf = cv2.KalmanFilter(6, 3)

        # State transition matrix — constant velocity model
        # x_new = x + vx,  y_new = y + vy,  area_new = area + varea
        self.kf.transitionMatrix = np.array([
            [1, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 1],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ], dtype=np.float32)

        # Measurement matrix — we observe position/area, not velocity
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ], dtype=np.float32)

        # Process noise — how much we trust the motion model
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 0.03

        # Measurement noise — how much we trust YOLO's detections
        self.kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1.0

        # Initial error covariance
        self.kf.errorCovPost = np.eye(6, dtype=np.float32)

        self.initialized = False
        self.frames_since_seen = 0

    def update(self, cx, cy, area):
        """ Call this when YOLO has a detection. Returns smoothed [cx, cy, area]. """
        measurement = np.array([[cx], [cy], [area]], dtype=np.float32)

        if not self.initialized:
            # Seed the filter with the first detection
            self.kf.statePost = np.array(
                [[cx], [cy], [area], [0.0], [0.0], [0.0]], dtype=np.float32
            )
            self.initialized = True

        self.kf.correct(measurement)
        self.frames_since_seen = 0
        predicted = self.kf.predict()
        return predicted[:3].flatten()

    def predict(self):
        """ Call this when there is NO detection. Returns predicted [cx, cy, area]. """
        self.frames_since_seen += 1
        predicted = self.kf.predict()
        return predicted[:3].flatten()

    @property
    def is_active(self):
        """ True while we still have a usable prediction. """
        return self.initialized and self.frames_since_seen < self.MAX_MISSING_FRAMES


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.model = YOLO('yolov8n.pt')  # Auto-downloads pretrained COCO weights

        # COCO classes we care about
        self.target_classes = [0, 2, 3, 4, 5, 7]  # person, car, motorcycle, airplane(drone), bus, truck
        self.class_labels = {
            0: 'Person',
            2: 'Car',
            3: 'Motorcycle',
            4: 'Drone',
            5: 'Bus',
            7: 'Truck',
        }
        self.class_colors = {
            0: (0, 255, 0),    # Person     - green
            2: (0, 0, 255),    # Car        - red
            3: (255, 165, 0),  # Motorcycle - orange
            4: (255, 0, 0),    # Drone      - blue
            5: (255, 0, 255),  # Bus        - magenta
            7: (0, 255, 255),  # Truck      - yellow
        }

        # Priority order for which class to track when multiple are detected
        self.track_priority = [0, 4, 2, 7, 5, 3]  # person first, then drone, then vehicles

        # One Kalman tracker for the primary target
        self.tracker = KalmanTracker()

        self.publisher = self.create_publisher(Point, 'target_position', 10)

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

    def pick_primary_target(self, boxes):
        """
        From all detections, pick one to track.
        Priority: person > drone > car > truck > bus > motorcycle.
        Within the same class, pick highest confidence.
        """
        best = None
        best_priority = len(self.track_priority) + 1
        best_conf = -1.0

        for box in boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            try:
                priority = self.track_priority.index(cls)
            except ValueError:
                priority = len(self.track_priority)

            if priority < best_priority or (priority == best_priority and conf > best_conf):
                best = box
                best_priority = priority
                best_conf = conf

        return best

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
                        all_boxes = results[0].boxes if results else []

                        # Draw ALL detections on the frame
                        for box in all_boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            conf = float(box.conf[0])
                            cls = int(box.cls[0])
                            label = self.class_labels.get(cls, 'Unknown')
                            color = self.class_colors.get(cls, (255, 255, 255))
                            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(image, f'{label} {conf:.2f}', (x1, y1 - 5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

                        # Pick the primary target and run it through the Kalman filter
                        primary = self.pick_primary_target(all_boxes)

                        if primary is not None:
                            x1, y1, x2, y2 = map(int, primary.xyxy[0])
                            raw_cx = float(x1 + (x2 - x1) / 2)
                            raw_cy = float(y1 + (y2 - y1) / 2)
                            raw_area = float((x2 - x1) * (y2 - y1))
                            cx, cy, area = self.tracker.update(raw_cx, raw_cy, raw_area)
                            cls = int(primary.cls[0])
                            self.get_logger().info(
                                f"Tracking {self.class_labels.get(cls, '?')}: "
                                f"X={cx:.1f}, Y={cy:.1f}, Area={area:.0f}"
                            )
                        elif self.tracker.is_active:
                            # No detection this frame — coast on Kalman prediction
                            cx, cy, area = self.tracker.predict()
                            self.get_logger().info(
                                f"Coasting (frames missing: {self.tracker.frames_since_seen}): "
                                f"X={cx:.1f}, Y={cy:.1f}"
                            )
                        else:
                            cx, cy, area = None, None, None

                        # Publish smoothed target position if we have one
                        if cx is not None:
                            # Draw the Kalman-filtered position as a crosshair
                            cv2.drawMarker(image, (int(cx), int(cy)), (255, 255, 255),
                                           cv2.MARKER_CROSS, 10, 1)
                            msg = Point()
                            msg.x = float(cx)
                            msg.y = float(cy)
                            msg.z = float(area)
                            self.publisher.publish(msg)

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
