import logging
from pathlib import Path
from typing import Union

import cv2
import numpy as np
import pandas as pd
from tqdm import tqdm
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time

from scipy.spatial.transform import Rotation as R
from svl.tms.data_structures import DroneImage
from svl.tms.schemas import GeoPoint, Orientation
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


class DroneImageStreamer:
    """Drone image streamer that reads drone images from a folder.

    Parameters
    ----------
    image_folder : Union[str, Path]
        path to the folder containing the images to stream, if the images have ground
        truth metadata, the folder should also contain a CSV file with the metadata.
    has_gt : bool
        whether the images have ground truth metadata
    logger : logging.Logger
        logger to use for logging

    """

    IMAGE_EXTENSIONS = [".jpg", ".jpeg", ".png", ".JPG", ".JPEG", ".PNG"]
    COLUMN_NAMES = [
        "Filename",
        "Latitude",
        "Longitude",
        "Altitude",
        "Gimball_Roll",
        "Gimball_Yaw",
        "Gimball_Pitch",
        "Flight_Roll",
        "Flight_Yaw",
        "Flight_Pitch",
    ]

    def __init__(
        self,
        image_folder: Union[str, Path],
        has_gt: bool = False,
        logger: logging.Logger = None,
    ) -> None:
        self.image_folder = (
            image_folder if isinstance(image_folder, Path) else Path(image_folder)
        )
        if not self.image_folder.exists():
            raise FileNotFoundError(f"Image folder not found at {self.image_folder}")
        self.cv2_read_mode = cv2.IMREAD_GRAYSCALE
        if logger is None:
            logger = logging.getLogger(__name__)
        self.logger = logger
        self.has_gt = has_gt
        self._initialize_db()

    def __len__(self) -> int:
        return self._num_images

    def __add__(self, other: "DroneImageStreamer") -> "DroneImageStreamer":
        new_streamer = DroneImageStreamer(
            image_folder=self.image_folder,
            has_gt=self.has_gt,
            logger=self.logger,
        )
        new_streamer._image_db = {**self._image_db, **other._image_db}
        new_streamer._num_images = len(new_streamer._image_db)
        return new_streamer

    @property
    def image_names(self) -> list[str]:
        """List of image names in the streamer."""
        return list(self._image_db.keys())

    def _initialize_db(self) -> None:
        """Initialize the image database."""
        self._image_db = dict()
        self._num_images = 0
        self._metadata = None
        if self.has_gt:
            self._build_image_db_with_gt()
        else:
            self._build_image_db()
        self.current_idx = 0

    def _build_image_db(self) -> None:
        """Build the image database without ground truth metadata."""
        self.logger.info(f"Building image database for {self.image_folder}")
        for image_path in tqdm(
            self.image_folder.glob("*"), total=len(list(self.image_folder.glob("*")))
        ):
            if image_path.suffix in self.IMAGE_EXTENSIONS:
                drone_image = DroneImage(image_path=image_path)
                self._image_db[image_path.name] = drone_image
                self._num_images += 1
        self.logger.info(
            f"Image database built successfully with {self._num_images} images"
        )

    def _build_image_db_with_gt(self) -> None:
        """Build the image database with ground truth metadata.

        The ground truth metadata should be in a CSV file with the following columns:
        - Filename
        - Latitude
        - Longitude
        - Altitude
        - Gimball_Roll
        - Gimball_Yaw
        - Gimball_Pitch
        - Flight_Roll
        - Flight_Yaw
        - Flight_Pitch
        """
        csv_files = list(self.image_folder.glob("*.csv"))
        if len(csv_files) == 0:
            raise FileNotFoundError(f"No CSV files found in {self.image_folder}")
        if len(csv_files) > 1:
            raise ValueError(f"Multiple CSV files found in {self.image_folder}")
        csv_file = csv_files[0]
        self.logger.info(f"Building image database with GT from {csv_file}")
        self._metadata = pd.read_csv(csv_file)
        for _, row in tqdm(self._metadata.iterrows(), total=len(self._metadata)):
            image_path = self.image_folder / row["Filename"]
            drone_image = DroneImage(
                image_path=image_path,
                geo_point=GeoPoint(
                    latitude=row["Latitude"],
                    longitude=row["Longitude"],
                    altitude=row["Altitude"],
                ),
                camera_orientation=Orientation(
                    pitch=row["Gimball_Pitch"],
                    roll=row["Gimball_Roll"],
                    yaw=row["Gimball_Yaw"],
                ),
                drone_orientation=Orientation(
                    pitch=row["Flight_Pitch"],
                    roll=row["Flight_Roll"],
                    yaw=row["Flight_Yaw"],
                ),
            )
            self._image_db[image_path.name] = drone_image
            self._num_images += 1
        self.logger.info(
            f"Image database built successfully with {self._num_images} images"
        )

    def read_image(self, image_path: Union[str, Path]) -> np.ndarray:
        """Read an image from a path.

        Parameters
        ----------
        image_path : Union[str, Path]
            path to the image

        Returns
        -------
        np.ndarray
            image as a numpy array
        """
        image_path = image_path if isinstance(image_path, Path) else Path(image_path)
        if not image_path.exists():
            raise FileNotFoundError(f"Image not found at {image_path}")
        return cv2.imread(str(image_path), self.cv2_read_mode)

    def __next__(self) -> DroneImage:
        """Get the next image in the streamer."""
        if self._num_images == 0 or self.current_idx >= self._num_images:
            raise StopIteration

        drone_image: DroneImage = self._image_db[self.image_names[self.current_idx]]
        drone_image.image = self.read_image(drone_image.image_path)

        self.current_idx += 1
        return drone_image

    def __iter__(self) -> "DroneImageStreamer":
        """Return the streamer as an iterator."""
        self.current_idx = 0
        return self

class RosDataSubscriber(Node):
    def __init__(self, image_topic):
        super().__init__('ros_streamer_bridge')
        self.bridge = CvBridge()

        self.latest_frame = None
        self.latest_gps = None
        self.latest_odom = None

        from rclpy.qos import qos_profile_sensor_data

        self.sub_image = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)

        self.sub_gps = self.create_subscription(
            NavSatFix, '/m100_1/aircraft/raw_gps', self.gps_callback, qos_profile_sensor_data)

        self.sub_odom = self.create_subscription(
            Odometry, '/m100_1/aircraft/odometry', self.odom_callback, qos_profile_sensor_data)

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def gps_callback(self, msg):
        self.latest_gps = msg

    def odom_callback(self, msg):
        self.latest_odom = msg


class RosCameraStreamer:
    def __init__(self, topic_name="/m100_1/sensors/pylon_camera_node/image_raw", logger=None):
        self.logger = logger or logging.getLogger(__name__)
        
        if not rclpy.ok():
            rclpy.init()

        self.ros_node = RosDataSubscriber(topic_name)

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.spin_thread.start()
        
        self.current_idx = 0
        self.logger.info(f"RosCameraStreamer connected to camera, GPS and odometry.")

    def quat_to_rpy_map(self, qx, qy, qz, qw):
        r = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)

        yaw_map = np.pi/2 - yaw

        yaw_map -= np.pi/2

        roll = np.degrees(roll)
        pitch = np.degrees(pitch)
        yaw_map = np.degrees(yaw_map)

        yaw_map = (yaw_map + 360) % 360

        return roll, pitch, yaw_map

    def __iter__(self):
        return self

    def __len__(self):
        return 1000000

    def __next__(self):
        while self.ros_node.latest_frame is None:
            time.sleep(0.01)
            
        frame = self.ros_node.latest_frame.copy()

        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        lat, lon, alt = 0.0, 0.0, 0.0
        if self.ros_node.latest_gps is not None:
            lat = self.ros_node.latest_gps.latitude
            lon = self.ros_node.latest_gps.longitude
            alt = self.ros_node.latest_gps.altitude

        geo = GeoPoint(
            latitude=lat,
            longitude=lon,
            altitude=alt
            )

        yaw, pitch, roll = 0.0, 0.0, 0.0
        if self.ros_node.latest_odom is not None:
             q = self.ros_node.latest_odom.pose.pose.orientation
             roll, pitch, yaw = self.quat_to_rpy_map(q.x, q.y, q.z, q.w)


        ori_drone = Orientation(yaw=yaw, pitch=pitch, roll=roll)
        ori_cam = Orientation(yaw=0.0, pitch=0.0, roll=0.0)

        drone_image = DroneImage(
            image_path=Path(f"ros_frame_{self.current_idx}.jpg"),
            geo_point=geo,
            camera_orientation=ori_cam,
            drone_orientation=ori_drone
        )
        drone_image.image = frame

        self.current_idx += 1
        self.ros_node.latest_frame = None

        return drone_image

    def release(self):
        rclpy.shutdown()
        self.spin_thread.join()
