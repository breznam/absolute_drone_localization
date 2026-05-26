# Visual Localization for UAVs using Satellite Imagery

This repository is a project-specific extension of the original
[`TerboucheHacene/visual_localization`](https://github.com/TerboucheHacene/visual_localization/tree/main)
project for UAV visual localization against satellite imagery.

It is not a from-scratch reimplementation. The original repository provides the
base localization pipeline, and this version adapts and extends it for a
project-specific dataset, map layout, preprocessing flow, and experiment setup.

## What This Version Adds or Changes

- Project-specific map and query dataset paths.
- Changes to the main localization pipeline and query preprocessing.
- Support for project-specific drone image streaming and ROS2 integration.
- A standalone TMS download helper for building local reference maps.
- Parameter tuning utilities for batch testing localization settings.
- Strict result filtering utilities for reviewing tuning outputs.

## Repository Structure

- `src/svl/`: main localization package
- `scripts/main.py`: main offline localization runner
- `scripts/tms_downloader.py`: helper script for downloading reference map tiles
- `tuning/tune_visual_localization.py`: parameter sweep runner
- `tuning/analyze_tuning_results_strict.py`: strict filtering and ranking of tuning results
- `config/`, `launch/`, `package.xml`, `setup.py`, `setup.cfg`: ROS2 packaging and launch support
- `data/`: local datasets, maps, and generated outputs

## Main Project Files Modified for This Version

The main project-specific runtime changes are in:

- `pipeline.py`
- `main.py` in practice this repository uses `scripts/main.py`
- `preprocessing.py`
- `drone_streamer.py`
- `constants.py`

In this repository those correspond to:

- `src/svl/localization/pipeline.py`
- `scripts/main.py`
- `src/svl/localization/preprocessing.py`
- `src/svl/localization/drone_streamer.py`
- `src/svl/utils/constants.py`

Other files worth mentioning for this project variant:

- `src/svl/localization/map_reader.py`
- `src/svl/ros2_node.py`
- `config/visual_localization.yaml`
- `launch/visual_localization.launch.py`
- `package.xml`
- `setup.py`
- `setup.cfg`

## Installation

Clone your repository and initialize the bundled SuperGlue submodule:

```bash
cd visual_localization
git submodule update --init --recursive
```

Install dependencies with Poetry:

```bash
pip install poetry
poetry install
```

If you only want the main Python dependencies:

```bash
poetry install --only main
```

## Running the Main Localization Script

The main offline runner is:

```bash
poetry run python scripts/main.py
```

The current script is configured for:

- map database: `data/map/fekt_z18`
- query images: `data/query/fekt/`
- output visualizations: `data/output`

## Downloading Reference Maps with `scripts/tms_downloader.py`

The downloader script is a simple project helper with hardcoded parameters.
Before running it, edit the following values inside the script as needed:

- `flight_zone`
- `tms_url`
- `output_path`
- `zoom_level`

Run it with:

```bash
poetry run python scripts/tms_downloader.py
```

By default it downloads tiles for the configured flight zone and writes the
resulting mosaic and tiles into the selected local output directory.

## Parameter Tuning Workflow

Run the parameter sweep:

```bash
poetry run python tuning/tune_visual_localization.py
```

This script runs batches of localization experiments and writes generated results
under `data/experiments_tuning/`.

Then run the strict analysis script:

```bash
poetry run python tuning/analyze_tuning_results_strict.py
```

This script filters and ranks tuning results using strict acceptance criteria.

Important: the current analysis script reads:

```text
data/experiments_tuning_refined_3/results.csv
```

If your tuning run writes to a different experiment directory, update
`RESULTS_CSV` in `tuning/analyze_tuning_results_strict.py` before running it.

## Note on Generative AI Assistance

The tuning utilities
`tuning/analyze_tuning_results_strict.py` and
`tuning/tune_visual_localization.py`
were created with the help of generative AI to support parameter testing and
strict filtering of tuning results.

## Data and Git Hygiene

This repository contains project-specific local data. For a clean GitHub upload:

- do not commit generated experiment outputs
- do not commit generated visualization images
- do not commit large local datasets or downloaded map tiles unless they are intentionally part of the repository
- do not commit temporary CSV outputs unless they are meant to be preserved as reference results

In particular, local folders such as `data/output/`, tuning output folders, and
large datasets under `data/map/` and `data/query/` should remain untracked.

## Acknowledgments

- Original base repository:
  `TerboucheHacene/visual_localization`
- Paper context:
  *Vision-based GNSS-Free Localization for UAVs in the Wild*
- SuperPoint and SuperGlue dependencies are provided through the bundled
  `src/superglue_lib` submodule

## License

This project remains under the repository's existing license. See `LICENSE`.
