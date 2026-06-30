#!/usr/bin/env python3
"""Plot simulation CSV outputs from the motor control course."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


PRESETS: dict[str, list[list[str]]] = {
    "dc_position_pid": [
        ["target_position_rad", "position_rad"],
        ["velocity_rad_s"],
        ["current_a", "torque_nm"],
        ["voltage_v"],
        ["load_torque_nm"],
    ],
    "cascaded_servo": [
        ["target_position_rad", "position_rad"],
        ["velocity_setpoint_rad_s", "velocity_rad_s"],
        ["current_setpoint_a", "current_a"],
        ["voltage_command_v"],
        ["load_torque_nm"],
    ],
    "pmsm_foc": [
        ["id_ref_a", "id_a"],
        ["iq_ref_a", "iq_a"],
        ["torque_nm"],
        ["mechanical_velocity_rad_s"],
        ["vd_v", "vq_v"],
    ],
    "encoder_velocity_filter": [
        ["true_position_rad", "measured_position_rad"],
        ["true_velocity_rad_s", "raw_velocity_rad_s", "filtered_velocity_rad_s"],
        ["position_error_rad"],
    ],
    "frequency_response": [
        ["gain"],
        ["phase_deg"],
    ],
    "disturbance_observer": [
        ["target_position_rad", "position_no_dob_rad", "position_dob_rad"],
        ["load_torque_nm", "estimated_disturbance_nm"],
    ],
    "single_joint_gravity_friction": [
        ["target_position_rad", "position_feedback_only_rad", "position_compensated_rad"],
        ["error_feedback_only_rad", "error_compensated_rad"],
        ["torque_feedback_only_nm", "torque_compensated_nm"],
    ],
    "joint_impedance": [
        ["external_torque_nm"],
        ["soft_position_rad", "stiff_position_rad"],
        ["soft_torque_nm", "stiff_torque_nm"],
    ],
    "two_link_computed_torque": [
        ["q1_des_rad", "q1_rad"],
        ["q2_des_rad", "q2_rad"],
        ["q1_error_rad", "q2_error_rad"],
        ["tau1_nm", "tau2_nm"],
    ],
    "realtime_jitter_safety": [
        ["dt_s", "jitter_s"],
        ["command_current_a", "measured_current_a"],
        ["is_enabled", "is_fault", "watchdog_miss"],
    ],
    "tuning_suite": [
        ["target_position_rad", "position_rad"],
        ["tracking_error_rad", "absolute_error_rad"],
        ["command_torque_nm", "external_torque_nm"],
        ["thermal_index"],
    ],
}


def infer_preset(csv_path: Path) -> str | None:
    stem = csv_path.stem
    return stem if stem in PRESETS else None


def read_csv(csv_path: Path) -> tuple[list[str], dict[str, list[float]]]:
    columns: dict[str, list[float]] = {}
    with csv_path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise ValueError(f"No header found in {csv_path}")

        for name in reader.fieldnames:
            columns[name] = []

        for row in reader:
            for name, value in row.items():
                columns[name].append(float(value))

    return list(columns.keys()), columns


def plot_groups(
    csv_path: Path,
    groups: list[list[str]],
    output_path: Path | None,
    show: bool,
) -> None:
    _, columns = read_csv(csv_path)
    time_key = "time_s" if "time_s" in columns else "frequency_hz"
    if time_key not in columns:
        raise ValueError(f"CSV must contain time_s or frequency_hz: {csv_path}")

    x_values = columns[time_key]
    x_label = "Time (s)" if time_key == "time_s" else "Frequency (Hz)"

    figure, axes = plt.subplots(len(groups), 1, figsize=(10, 2.8 * len(groups)), sharex=True)
    if len(groups) == 1:
        axes = [axes]

    for axis, group in zip(axes, groups):
        for column in group:
            if column not in columns:
                raise ValueError(f"Column '{column}' not found in {csv_path}")
            axis.plot(x_values, columns[column], linewidth=1.5, label=column)
        axis.set_ylabel(group[0])
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best", fontsize=8)

    axes[-1].set_xlabel(x_label)
    figure.suptitle(csv_path.name, fontsize=12)
    figure.tight_layout()

    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Saved plot: {output_path}")

    if show:
        plt.show()
    else:
        plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot motor control simulation CSV files.")
    parser.add_argument(
        "csv",
        nargs="?",
        default="outputs/dc_position_pid.csv",
        help="CSV file path (default: outputs/dc_position_pid.csv)",
    )
    parser.add_argument(
        "--preset",
        help="Plot preset name. Defaults to CSV file stem when available.",
    )
    parser.add_argument(
        "--output",
        help="Output image path. Defaults to outputs/plots/<csv_stem>.png",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show interactive plot window.",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    preset_name = args.preset or infer_preset(csv_path)
    if preset_name is None:
        raise SystemExit(
            f"No preset found for {csv_path.name}. Use --preset with one of: {', '.join(PRESETS)}"
        )
    if preset_name not in PRESETS:
        raise SystemExit(f"Unknown preset: {preset_name}")

    output_path = Path(args.output) if args.output else Path("outputs/plots") / f"{csv_path.stem}.png"
    plot_groups(csv_path, PRESETS[preset_name], output_path, args.show)


if __name__ == "__main__":
    main()
