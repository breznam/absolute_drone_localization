from pathlib import Path
import pandas as pd


RESULTS_CSV = Path("data/experiments_tuning_refined_3/results.csv")

# A configuration with max_distance above this value is considered unsafe,
# because it produced at least one critically wrong accepted match.
#
# Change this according to what you consider acceptable:
#   50   = very strict
#   100  = moderate
#   None = no hard outlier rejection
MAX_ALLOWED_DISTANCE = 50.0

# Optional: require at least this many accepted matches.
# Keep low for strict precision-first experiments.
MIN_NUM_MATCHES = 1

# Optional: require at least this ratio.
# Usually keep this low, because ratio_matches includes false positives.
MIN_RATIO_MATCHES = 0.0


def main() -> None:
    if not RESULTS_CSV.exists():
        raise FileNotFoundError(
            f"Missing file: {RESULTS_CSV}\n"
            "Run tuning first:\n"
            "    python scripts/tune_visual_localization.py"
        )

    df = pd.read_csv(RESULTS_CSV)

    required = [
        "num_matches",
        "mae",
        "max_distance",
        "min_distance",
        "ratio_matches",
        "runtime_s",
    ]

    missing = [c for c in required if c not in df.columns]
    if missing:
        raise SystemExit(f"Missing columns in results.csv: {missing}")

    numeric = required + [
        "nms_radius",
        "keypoint_threshold",
        "max_keypoints",
        "sinkhorn_iterations",
        "match_threshold",
    ]

    for col in numeric:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    filtered = df.copy()

    if MAX_ALLOWED_DISTANCE is not None:
        filtered = filtered[filtered["max_distance"] <= MAX_ALLOWED_DISTANCE]

    filtered = filtered[filtered["num_matches"] >= MIN_NUM_MATCHES]
    filtered = filtered[filtered["ratio_matches"] >= MIN_RATIO_MATCHES]

    if filtered.empty:
        raise SystemExit(
            "No configurations left after strict filtering. "
            "Try increasing MAX_ALLOWED_DISTANCE to 75 or 100."
        )

    # Precision-first ranking:
    #   lower max_distance first, then lower mae, then lower runtime,
    #   then higher ratio_matches.
    sorted_df = filtered.sort_values(
        by=["mae", "max_distance", "runtime_s", "ratio_matches"],
        ascending=[True, True, True, False],
    )

    cols = [
        "exp_id",
        "experiment_group",
        "nms_radius",
        "keypoint_threshold",
        "max_keypoints",
        "sinkhorn_iterations",
        "match_threshold",
        "num_matches",
        "ratio_matches",
        "mae",
        "max_distance",
        "min_distance",
        "runtime_s",
    ]

    cols = [c for c in cols if c in sorted_df.columns]

    print("=" * 100)
    print("STRICT PRECISION-FIRST ANALYSIS")
    print(f"MAX_ALLOWED_DISTANCE = {MAX_ALLOWED_DISTANCE}")
    print(f"MIN_NUM_MATCHES = {MIN_NUM_MATCHES}")
    print(f"MIN_RATIO_MATCHES = {MIN_RATIO_MATCHES}")
    print("Ranking: lowest mae -> lowest max_distance -> lowest runtime -> highest ratio")
    print("=" * 100)

    print("\nTOP CONFIGURATIONS")
    print(sorted_df[cols].head(20).to_string(index=False))

    print("\nBEST CONFIGURATION")
    print(sorted_df.iloc[0][cols].to_string())

    out = RESULTS_CSV.parent / "results_strict_sorted.csv"
    sorted_df.to_csv(out, index=False)
    print(f"\nStrict sorted results saved to: {out}")


if __name__ == "__main__":
    main()
