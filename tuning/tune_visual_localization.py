import csv
import itertools
import json
import logging
import time
import traceback
from pathlib import Path
from pprint import pprint

from svl.keypoint_pipeline.detection_and_description import SuperPointAlgorithm
from svl.keypoint_pipeline.matcher import SuperGlueMatcher
from svl.keypoint_pipeline.typing import SuperGlueConfig, SuperPointConfig
from svl.localization.drone_streamer import DroneImageStreamer
from svl.localization.map_reader import SatelliteMapReader
from svl.localization.pipeline import Pipeline, PipelineConfig
from svl.localization.preprocessing import QueryProcessor
from svl.tms.data_structures import CameraModel


MAP_DB_PATH = "data/fekt_z18"
QUERY_IMAGE_FOLDER = "data/fekt_merged/"
OUTPUT_ROOT = Path("data/experiments_tuning")

DEVICE = "cpu"
SUPERGLUE_WEIGHTS = "outdoor"

MAP_RESIZE_SIZE = (800,)
QUERY_PROCESSINGS = ["resize", "warp"]
QUERY_SIZE = (800,)

CAMERA_MODEL = CameraModel(
    focal_length=2.95 / 1000,
    resolution_height=540,
    resolution_width=720,
    hfov_deg=79.3,
)

MAIN_GRID = {
    "nms_radius": [4],
    "keypoint_threshold": [0.003, 0.005, 0.01, 0.02],
    "max_keypoints": [1024, 2048, 4096],
    "sinkhorn_iterations": [20],
    "match_threshold": [0.2, 0.3, 0.5],
}

REPRESENTATIVE_CONFIGS = [
    {
        "nms_radius": 4,
        "keypoint_threshold": 0.005,
        "max_keypoints": 2048,
        "sinkhorn_iterations": 20,
        "match_threshold": 0.3,
    },
    {
        "nms_radius": 4,
        "keypoint_threshold": 0.01,
        "max_keypoints": 1024,
        "sinkhorn_iterations": 20,
        "match_threshold": 0.5,
    },
]

SINKHORN_ABLATION_VALUES = [10, 20, 30]
NMS_ABLATION_VALUES = [3, 4, 6, 8]


def setup_logging() -> None:
    fmt = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    logging.basicConfig(format=fmt, level=logging.INFO, datefmt="%H:%M:%S")


def expand_grid(grid: dict) -> list[dict]:
    keys = list(grid.keys())
    return [dict(zip(keys, values)) for values in itertools.product(*grid.values())]


def build_experiment_list() -> list[dict]:
    experiments = []

    # main grid
    for cfg in expand_grid(MAIN_GRID):
        experiments.append(
            {
                "experiment_group": "main_grid",
                **cfg,
            }
        )

    # sinkhorn ablation
    for base_cfg in REPRESENTATIVE_CONFIGS:
        for sinkhorn in SINKHORN_ABLATION_VALUES:
            cfg = dict(base_cfg)
            cfg["sinkhorn_iterations"] = sinkhorn
            experiments.append(
                {
                    "experiment_group": "sinkhorn_ablation",
                    **cfg,
                }
            )

    # NMS radius ablation
    for base_cfg in REPRESENTATIVE_CONFIGS:
        for nms in NMS_ABLATION_VALUES:
            cfg = dict(base_cfg)
            cfg["nms_radius"] = nms
            experiments.append(
                {
                    "experiment_group": "nms_ablation",
                    **cfg,
                }
            )

    return experiments


def build_pipeline(cfg: dict) -> Pipeline:
    superpoint_config = SuperPointConfig(
        device=DEVICE,
        nms_radius=cfg["nms_radius"],
        keypoint_threshold=cfg["keypoint_threshold"],
        max_keypoints=cfg["max_keypoints"],
    )
    superpoint_algorithm = SuperPointAlgorithm(superpoint_config)

    superglue_config = SuperGlueConfig(
        device=DEVICE,
        weights=SUPERGLUE_WEIGHTS,
        sinkhorn_iterations=cfg["sinkhorn_iterations"],
        match_threshold=cfg["match_threshold"],
    )
    superglue_matcher = SuperGlueMatcher(superglue_config)

    map_reader = SatelliteMapReader(
        db_path=MAP_DB_PATH,
        resize_size=MAP_RESIZE_SIZE,
        logger=logging.getLogger("SatelliteMapReader"),
    )
    map_reader.initialize_db()
    map_reader.setup_db()
    map_reader.resize_db_images()
    map_reader.describe_db_images(superpoint_algorithm)

    streamer = DroneImageStreamer(
        image_folder=QUERY_IMAGE_FOLDER,
        has_gt=True,
        logger=logging.getLogger("DroneImageStreamer"),
    )

    query_processor = QueryProcessor(
        processings=QUERY_PROCESSINGS,
        camera_model=CAMERA_MODEL,
        satellite_resolution=None,
        size=QUERY_SIZE,
    )

    pipeline_logger = logging.getLogger("Pipeline")
    pipeline_logger.setLevel(logging.INFO)

    return Pipeline(
        map_reader=map_reader,
        drone_streamer=streamer,
        detector=superpoint_algorithm,
        matcher=superglue_matcher,
        query_processor=query_processor,
        config=PipelineConfig(),
        logger=pipeline_logger,
    )


def flatten_metrics(metrics: dict) -> dict:
    """
    Converts nested metric dictionaries into flat CSV-friendly keys.

    Example:
        {"error": {"mean": 12.3}} -> {"error.mean": 12.3}
    """
    flat = {}

    def _walk(prefix, obj):
        if isinstance(obj, dict):
            for key, value in obj.items():
                new_prefix = f"{prefix}.{key}" if prefix else str(key)
                _walk(new_prefix, value)
        else:
            flat[prefix] = obj

    _walk("", metrics)
    return flat


def append_row_csv(csv_path: Path, row: dict) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    file_exists = csv_path.exists()

    existing_rows = []
    fieldnames = list(row.keys())

    if file_exists:
        with csv_path.open("r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            existing_rows = list(reader)
            old_fields = reader.fieldnames or []

        fieldnames = list(dict.fromkeys(old_fields + list(row.keys())))

    existing_rows.append(row)

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(existing_rows)


def run_experiment(exp_id: int, cfg: dict) -> dict:
    output_path = OUTPUT_ROOT / f"exp_{exp_id:04d}_{cfg['experiment_group']}"
    output_path.mkdir(parents=True, exist_ok=True)

    with (output_path / "config.json").open("w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)

    pipeline = build_pipeline(cfg)

    start = time.time()
    preds = pipeline.run(output_path=output_path)
    runtime_s = time.time() - start

    metrics = pipeline.compute_metrics(preds)
    flat_metrics = flatten_metrics(metrics)

    result = {
        "exp_id": exp_id,
        "experiment_group": cfg["experiment_group"],
        "nms_radius": cfg["nms_radius"],
        "keypoint_threshold": cfg["keypoint_threshold"],
        "max_keypoints": cfg["max_keypoints"],
        "sinkhorn_iterations": cfg["sinkhorn_iterations"],
        "match_threshold": cfg["match_threshold"],
        "runtime_s": runtime_s,
        **flat_metrics,
    }

    return result


def main() -> None:
    setup_logging()

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    results_csv = OUTPUT_ROOT / "results.csv"
    failed_log = OUTPUT_ROOT / "failed_experiments.txt"

    experiments = build_experiment_list()

    print("=" * 80)
    print(f"Total experiments: {len(experiments)}")
    print(f"Results CSV: {results_csv}")
    print("=" * 80)

    for exp_id, cfg in enumerate(experiments):
        print("\n" + "=" * 80)
        print(f"Experiment {exp_id + 1}/{len(experiments)}")
        pprint(cfg)
        print("=" * 80)

        try:
            result = run_experiment(exp_id, cfg)
            append_row_csv(results_csv, result)

            print("DONE")
            pprint(result)

        except Exception as exc:
            print(f"FAILED experiment {exp_id}: {exc}")

            with failed_log.open("a", encoding="utf-8") as f:
                f.write("=" * 80 + "\n")
                f.write(f"Experiment ID: {exp_id}\n")
                f.write(json.dumps(cfg, indent=2) + "\n")
                f.write(traceback.format_exc() + "\n")

    print("\nFinished.")
    print(f"Results saved to: {results_csv}")


if __name__ == "__main__":
    main()
