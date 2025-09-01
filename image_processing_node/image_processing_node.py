import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class ImageProcessingNode(Node):
    """
    ROS 2 node for image processing operations including distortion removal and ROI cropping.

    Subscribes to:
        - image_raw topic (sensor_msgs/Image)
        - camera_info topic (sensor_msgs/CameraInfo)

    Publishes to:
        - image_clean topic (sensor_msgs/Image)
    """

    def __init__(self):
        super().__init__("image_processing_node")

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_subscription = self.create_subscription(
            Image, "image_raw", self.image_callback, 10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            "camera_info",
            self.camera_info_callback,
            10,
        )

        # Camera calibration parameters
        self.camera_matrix = None
        self.distortion_coefficients = None

        # Region of Interest (ROI) parameters
        self.image_roi_top = None
        self.image_roi_bottom = None
        self.image_roi_left = None
        self.image_roi_right = None

        # Undistortion maps for optimization
        self.map1 = None
        self.map2 = None

        # Publishers
        self.image_publisher = self.create_publisher(Image, "image_clean", 10)

    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback for camera info messages.
        Extracts camera matrix and distortion coefficients.
        """
        if not self.map1 or not self.map2:
            # Extract camera matrix
            self.camera_matrix = np.array(msg.k).reshape(3, 3)

            # Extract distortion coefficients
            self.distortion_coefficients = np.array(msg.d)

            self.camera_info_received = True

            self.image_height = msg.height
            self.image_width = msg.width

            self.image_roi_top = msg.roi.y_offset - msg.roi.height // 2
            self.image_roi_bottom = msg.roi.y_offset + msg.roi.height // 2
            self.image_roi_left = msg.roi.x_offset - msg.roi.width // 2
            self.image_roi_right = msg.roi.x_offset + msg.roi.width // 2

            self.get_logger().info("Camera calibration parameters received")
            self.get_logger().debug(f"Camera matrix: {self.camera_matrix}")
            self.get_logger().debug(
                f"Distortion coefficients: {self.distortion_coefficients}"
            )
            self.get_logger().debug(
                f"ROI: {self.image_roi_left}, {self.image_roi_top}, {self.image_roi_right}, {self.image_roi_bottom}"
            )

            # Get optimal new camera matrix
            new_camera_matrix, self.undistortion_roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix,
                self.distortion_coefficients,
                (self.image_width, self.image_height),
                1,
                (self.image_width, self.image_height),
            )

            # Initialize undistortion and rectification transformation maps
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.camera_matrix,
                self.distortion_coefficients,
                None,
                new_camera_matrix,
                (self.image_width, self.image_height),
                cv2.CV_16SC2,
            )

            self.get_logger().info("Undistortion maps initialized")

    def remove_distortion(self, image: np.ndarray) -> np.ndarray:
        """
        Remove lens distortion from the image.

        Args:
            image: Input image as numpy array

        Returns:
            Undistorted image
        """
        if not self.map1 or not self.map2:
            self.get_logger().warn_once(
                "Camera info not received yet, skipping distortion removal"
            )
            return image

        # Use pre-computed maps for efficient undistortion
        undistorted_image = cv2.remap(
            image, self.map1, self.map2, cv2.INTER_LINEAR
        )
        x, y, w, h = self.undistortion_roi

        return undistorted_image[y:y+h, x:x+w]

    def crop_roi(self, image: np.ndarray) -> np.ndarray:
        """
        Apply simple center crop to focus on the most important part of the image.
        Since we're not using parameters, this applies a conservative center crop
        that removes 10% from each edge to focus on the center region.

        Args:
            image: Input image as numpy array

        Returns:
            Cropped image
        """

        if (
            self.image_roi_top is None
            or self.image_roi_bottom is None
            or self.image_roi_left is None
            or self.image_roi_right is None
        ):
            self.get_logger().warn("ROI not set, skipping crop")
            return image

        # Crop the image
        cropped_image = image[
            self.image_roi_top : self.image_roi_bottom,
            self.image_roi_left : self.image_roi_right,
        ]

        return cropped_image

    def image_callback(self, msg: Image):
        """
        Callback for image messages.
        Processes the image and publishes the result.
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Process the image
            processed_image = self.remove_distortion(cv_image)
            processed_image = self.crop_roi(processed_image)

            # Convert back to ROS Image message
            processed_msg = self.bridge.cv2_to_imgmsg(
                processed_image, encoding="bgr8"
            )

            # Copy header information
            processed_msg.header = msg.header
            processed_msg.header.frame_id = msg.header.frame_id

            # Publish processed image
            self.image_publisher.publish(processed_msg)

            self.get_logger().debug("Image processed and published")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    """Main function to initialize and run the image processor node."""
    rclpy.init(args=args)

    image_processor = ImageProcessingNode()
    rclpy.spin(image_processor)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_processor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
