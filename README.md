# DDS Sensor Jitter Analysis for Unitree Humanoid Robot

## Overview

This project performs a temporal characterization of sensor streams transmitted through the DDS middleware in a Unitree humanoid robot platform.

The script measures timing jitter and effective sampling behavior of:

* IMU streams (`rt/lf/lowstate`)
* LiDAR Mid-360 point cloud streams (`rt/utlidar/cloud_livox_mid360`)

using the native Unitree SDK2 DDS interface.

The acquisition is executed sequentially to avoid concurrent subscriber interference and artificial jitter inflation.

---

# Objectives

The main goals of this experiment are:

* Measure temporal stability of DDS sensor streams
* Quantify timing jitter in real robotic middleware
* Estimate effective sensor frequencies
* Compare temporal behavior between IMU and LiDAR streams
* Generate datasets for statistical analysis and scientific publication

---

# Architecture

The experimental pipeline is:

```text
Sensors → Robot Drivers/Firmware → DDS → Unitree SDK2 → Python Callback
```

The script directly subscribes to DDS topics using the Unitree SDK2 interface.

No ROS2 subscriber is used in this version.

---

# Sensors

## IMU

Topic:

```text
rt/lf/lowstate
```

Measured variables:

* receive interval (`inter_recv`)
* accelerometer values
* gyroscope values

Expected frequency:

```text
≈ 20 Hz
```

Expected period:

T_{IMU} \approx 50\text{ ms}

---

## LiDAR Mid-360

Topic:

```text
rt/utlidar/cloud_livox_mid360
```

Measured variables:

* receive interval (`inter_recv`)
* internal timestamp interval (`inter_stamp`)
* number of points per scan

Expected frequency:

```text
≈ 10 Hz
```

Expected period:

T_{LiDAR} \approx 100\text{ ms}

Average points per scan:

```text
≈ 20,000 points
```

---

# Requirements

## ROS2

ROS 2 Foxy:

```bash
source /opt/ros/foxy/setup.bash
```

---

## CycloneDDS

Export CycloneDDS library path:

```bash
export LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/cyclonedds/lib:$LD_LIBRARY_PATH
```

---

## Python

Python 3.8+ recommended.

---

# Running the Experiment

Basic execution:

```bash
python3 jitter_imu_lidar_seq.py --duration 120 --output ~/data/jitter
```

Parameters:

| Parameter      | Description                    |
| -------------- | ------------------------------ |
| `--duration`   | Collection duration per sensor |
| `--output`     | Output directory               |
| `--iface`      | Network interface              |
| `--skip-imu`   | Skip IMU phase                 |
| `--skip-lidar` | Skip LiDAR phase               |

Example:

```bash
python3 jitter_imu_lidar_seq.py \
    --duration 60 \
    --output ~/data/jitter
```

---

# Output Files

## IMU

```text
imu_jitter.csv
```

Fields:

| Field        | Description                    |
| ------------ | ------------------------------ |
| `t_recv`     | Local receive timestamp        |
| `inter_recv` | Interval between DDS callbacks |
| `acc_x/y/z`  | Accelerometer                  |
| `gyr_x/y/z`  | Gyroscope                      |

---

## LiDAR

```text
lidar_jitter.csv
```

Fields:

| Field         | Description                 |
| ------------- | --------------------------- |
| `t_recv`      | Local receive timestamp     |
| `stamp_ns`    | Internal LiDAR timestamp    |
| `inter_recv`  | DDS receive interval        |
| `inter_stamp` | Internal timestamp interval |
| `n_points`    | Number of points            |

---

# Statistical Metrics

The script computes:

| Metric | Description        |
| ------ | ------------------ |
| `min`  | Minimum interval   |
| `max`  | Maximum interval   |
| `mean` | Average interval   |
| `std`  | Standard deviation |
| `p50`  | Median             |
| `p95`  | 95th percentile    |
| `p99`  | 99th percentile    |

Example:

```text
mean = 49.388 ms
std  = 0.414 ms
```

---

# Example Results

## IMU DDS Stream

| Metric | Value     |
| ------ | --------- |
| Mean   | 49.388 ms |
| Std    | 0.414 ms  |
| P95    | 49.948 ms |
| P99    | 50.441 ms |

Estimated frequency:

f_{IMU} \approx \frac{1}{0.049388} \approx 20.25\text{ Hz}

---

## LiDAR DDS Stream

| Metric | Value      |
| ------ | ---------- |
| Mean   | 100.189 ms |
| Std    | 2.637 ms   |
| P95    | 104.016 ms |
| P99    | 106.817 ms |

Estimated frequency:

f_{LiDAR} \approx \frac{1}{0.100189} \approx 9.98\text{ Hz}

---

# Interpretation

The results indicate:

* highly stable DDS communication for IMU streams
* larger variability in LiDAR due to point cloud payload size
* preservation of expected sensor periodicity
* low temporal overhead introduced by DDS middleware

---

# Scientific Motivation

This project supports experimental studies involving:

* robotic middleware evaluation
* real-time robotic systems
* temporal analysis of sensor pipelines
* DDS communication performance
* humanoid robot perception systems

Potential applications include:

* SLAM
* sensor fusion
* robot navigation
* real-time control
* distributed robotic systems

---

# Suggested Paper Title

## Option 1

**Temporal Characterization of DDS Sensor Streams in a Humanoid Robot**

## Option 2

**Timing Jitter Analysis of DDS-Based IMU and LiDAR Streams in a Humanoid Robot**

## Option 3

**Experimental Evaluation of DDS Temporal Stability in Humanoid Robot Sensors**

---

# Notes

* Sequential acquisition avoids concurrent callback interference.
* The experiment focuses on DDS timing behavior.
* Hardware clock synchronization is not assumed.
* Measurements use Linux monotonic clock timestamps.

---

# License

This project is intended for academic and research purposes.
