import logging
from pathlib import Path
from pprint import pformat
from threading import Thread
from typing import Iterable, Optional, Tuple
import traceback

import cv2
import rclpy
from rclpy.node import Node

from svl.keypoint_pipeline.detection_and_description import SuperPointAlgorithm
from svl.keypoint_pipeline.matcher import SuperGlueMatcher
from svl.keypoint_pipeline.typing import SuperGlueConfig, SuperPointConfig
from svl.localization.drone_streamer import DroneImageStreamer, RosCameraStreamer
from svl.localization.map_reader import SatelliteMapReader
from svl.localization.pipeline import Pipeline
from svl.localization.preprocessing import QueryProcessor
from svl.localization.base import PipelineConfig
from svl.tms.data_structures import CameraModel


def _normalize_size(value: Iterable[int]) -> Optional[Tuple[int, ...]]:
    size = tuple(int(v) for v in value if int(v) > 0)
    return size or None


class VisualLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("visual_localization")
        self._declare_parameters()
        self._worker: Optional[Thread] = None
        self._ros_streamer = None

    def _declare_parameters(self) -> None:
        self.declare_parameter("map_db_path", "data/fekt_z18")
        self.declare_parameter("map_resize_size", [1024])
        self.declare_parameter("image_folder", "data/fekt_merged")
        self.declare_parameter("has_gt", True)
        self.declare_parameter("use_ros_streamer", False)
        self.declare_parameter(
            "image_topic", "/m100_1/sensors/pylon_camera_node/image_raw"
        )
        self.declare_parameter("output_path", "data/output")
        self.declare_parameter("processings", ["resize", "warp"])
        self.declare_parameter("query_size", [1024])
        self.declare_parameter("satellite_resolution", 0.0)
        self.declare_parameter("superpoint_device", "cpu")
        self.declare_parameter("superpoint_nms_radius", 4)
        self.declare_parameter("superpoint_keypoint_threshold", 0.01)
        self.declare_parameter("superpoint_max_keypoints", -1)
        self.declare_parameter("superglue_device", "cpu")
        self.declare_parameter("superglue_weights", "outdoor")
        self.declare_parameter("superglue_sinkhorn_iterations", 20)
        self.declare_parameter("superglue_match_threshold", 0.5)
        self.declare_parameter("camera_focal_length", 2.95 / 1000)
        self.declare_parameter("camera_resolution_height", 540)
        self.declare_parameter("camera_resolution_width", 720)
        self.declare_parameter("camera_hfov_deg", 79.3)
        self.declare_parameter("homography_method", int(cv2.RANSAC))
        self.declare_parameter("homography_threshold", 5.0)
        self.declare_parameter("homography_confidence", 0.995)
        self.declare_parameter("homography_max_iter", 2000)

    def start(self) -> None:
        self._worker = Thread(target=self._run_pipeline, daemon=True)
        self._worker.start()

    def _run_pipeline(self) -> None:
        try:
            logging.basicConfig(
                format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
                level=logging.INFO,
                datefmt="%H:%M:%S",
            )

            superpoint_algorithm = SuperPointAlgorithm(
                SuperPointConfig(
                    device=self.get_parameter("superpoint_device").value,
                    nms_radius=int(self.get_parameter("superpoint_nms_radius").value),
                    keypoint_threshold=float(
                        self.get_parameter("superpoint_keypoint_threshold").value
                    ),
                    max_keypoints=int(
                        self.get_parameter("superpoint_max_keypoints").value
                    ),
                )
            )
            superglue_matcher = SuperGlueMatcher(
                SuperGlueConfig(
                    device=self.get_parameter("superglue_device").value,
                    weights=self.get_parameter("superglue_weights").value,
                    sinkhorn_iterations=int(
                        self.get_parameter("superglue_sinkhorn_iterations").value
                    ),
                    match_threshold=float(
                        self.get_parameter("superglue_match_threshold").value
                    ),
                )
            )

            map_reader = SatelliteMapReader(
                db_path=self.get_parameter("map_db_path").value,
                resize_size=_normalize_size(
                    self.get_parameter("map_resize_size").value
                ),
                logger=logging.getLogger(f"{__name__}.SatelliteMapReader"),
            )
            map_reader.initialize_db()
            map_reader.setup_db()
            map_reader.resize_db_images()
            map_reader.describe_db_images(superpoint_algorithm)

            if self.get_parameter("use_ros_streamer").value:
                self._ros_streamer = RosCameraStreamer(
                    topic_name=self.get_parameter("image_topic").value,
                    logger=logging.getLogger(f"{__name__}.RosCameraStreamer"),
                )
                streamer = self._ros_streamer
            else:
                streamer = DroneImageStreamer(
                    image_folder=self.get_parameter("image_folder").value,
                    has_gt=bool(self.get_parameter("has_gt").value),
                    logger=logging.getLogger(f"{__name__}.DroneImageStreamer"),
                )
                self.get_logger().info(f"Loaded {len(streamer)} offline images")

            satellite_resolution = float(
                self.get_parameter("satellite_resolution").value
            )
            query_processor = QueryProcessor(
                processings=list(self.get_parameter("processings").value),
                camera_model=CameraModel(
                    focal_length=float(self.get_parameter("camera_focal_length").value),
                    resolution_height=int(
                        self.get_parameter("camera_resolution_height").value
                    ),
                    resolution_width=int(
                        self.get_parameter("camera_resolution_width").value
                    ),
                    hfov_deg=float(self.get_parameter("camera_hfov_deg").value),
                ),
                satellite_resolution=(
                    satellite_resolution if satellite_resolution > 0 else None
                ),
                size=_normalize_size(self.get_parameter("query_size").value),
            )

            pipeline_logger = logging.getLogger(f"{__name__}.Pipeline")
            pipeline_logger.setLevel(logging.DEBUG)
            pipeline = Pipeline(
                map_reader=map_reader,
                drone_streamer=streamer,
                detector=superpoint_algorithm,
                matcher=superglue_matcher,
                query_processor=query_processor,
                config=PipelineConfig(
                    homography_method=int(
                        self.get_parameter("homography_method").value
                    ),
                    homography_threshold=float(
                        self.get_parameter("homography_threshold").value
                    ),
                    homography_confidence=float(
                        self.get_parameter("homography_confidence").value
                    ),
                    homography_max_iter=int(
                        self.get_parameter("homography_max_iter").value
                    ),
                ),
                logger=pipeline_logger,
            )

            output_path = Path(self.get_parameter("output_path").value)
            output_path.mkdir(parents=True, exist_ok=True)
            preds = pipeline.run(output_path=output_path)
            metrics = pipeline.compute_metrics(preds)
            self.get_logger().info(f"Pipeline metrics:\n{pformat(metrics)}")

            if not self.get_parameter("use_ros_streamer").value and rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            logging.error("Pipeline failed:\n%s", traceback.format_exc())
            self.get_logger().error(f"Pipeline failed: {exc}")
            if rclpy.ok():
                rclpy.shutdown()

    def destroy_node(self) -> bool:
        if self._ros_streamer is not None:
            self._ros_streamer.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualLocalizationNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
