import rclpy
from rclpy.node import Node
from mavros_msgs.msg import CameraImageCaptured

import cv2
import numpy as np

class ImageCapturedSubscriber(Node):

    def __init__(self):
        super().__init__('image_captured_subscriber')
        self.subscription = self.create_subscription(
            CameraImageCaptured,
            '/mavros/camera/image_captured',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Initialize data for displaying
        self.data = {
            'time_usec': 0,
            'latitude': 0.0,
            'longitude': 0.0,
            'altitude': 0.0,
            'orientation': (0.0, 0.0, 0.0)
        }

        # Initialize OpenCV window
        self.cv_window_name = "Image Captured Info"
        cv2.namedWindow(self.cv_window_name, cv2.WINDOW_NORMAL)

    def listener_callback(self, msg):
        # Update the data
        self.data['time_usec'] = msg.time_usec
        self.data['latitude'] = msg.latitude
        self.data['longitude'] = msg.longitude
        self.data['altitude'] = msg.altitude
        self.data['orientation'] = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])

        # Display the updated information
        self.update_visual()

    def update_visual(self):
        # Create a blank image
        img = np.zeros((500, 800, 3), dtype=np.uint8)

        # Add text information on the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_color = (255, 255, 255)

        cv2.putText(img, 'Image Captured Info:', (30, 50), font, 1, text_color, 2, cv2.LINE_AA)
        cv2.putText(img, f'Timestamp (usec): {self.data["time_usec"]}', (30, 100), font, 0.7, text_color, 1, cv2.LINE_AA)
        cv2.putText(img, f'Latitude: {self.data["latitude"]}', (30, 150), font, 0.7, text_color, 1, cv2.LINE_AA)
        cv2.putText(img, f'Longitude: {self.data["longitude"]}', (30, 200), font, 0.7, text_color, 1, cv2.LINE_AA)
        cv2.putText(img, f'Altitude: {self.data["altitude"]}', (30, 250), font, 0.7, text_color, 1, cv2.LINE_AA)
        cv2.putText(img, f'Orientation: {self.data["orientation"]}', (30, 300), font, 0.7, text_color, 1, cv2.LINE_AA)

        # Display the image in the OpenCV window
        cv2.imshow(self.cv_window_name, img)
        cv2.waitKey(1)  # Use a short wait to update the display

def main(args=None):
    rclpy.init(args=args)
    node = ImageCapturedSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
