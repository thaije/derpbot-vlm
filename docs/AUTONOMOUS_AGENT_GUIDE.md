# Autonomous Agent Guide — ARST

Gazebo Harmonic + ROS 2 Jazzy simulation testbed. Your task: deploy a fully autonomous agent and complete the scenario.

---

## 1. Task

You will be placed in an unknown indoor environment. Before each run you receive a **mission description** with the mission goal. There are two scenario types:

1. **explore_detect** — Find and report all target objects within the time limit. Your agent explores the environment, detects instances of every target type, and reports detections.

2. **proximity** — Navigate to within a radius of a single target object. No detection reporting required — success is purely based on ground-truth robot position reaching the proximity radius.

Scenarios come in multiple **difficulty tiers** and **world templates**. The environment may challenge your agent in various ways — expect conditions to differ between tiers and runs. Object placement and spawn position are **randomised each run** unless a seed is fixed. Doors are passable or permanently locked; no unlocking or interaction is required.

> **Design advice:** Build a general-purpose agent. Tiers, object types, and environment conditions may be extended or changed at any time. Agents that adapt to what they observe will hold up far better than those tuned to specific scenarios.

**Explore-detect success** = all required instances detected before the time limit. Thorough exploration of the full environment earns bonus points regardless of outcome.

**Proximity success** = robot ground-truth position enters the proximity radius of the target object before the time limit.

---

## 2. Grading

### Explore-detect scenarios

Runs are scored 0–100 across five categories, combined as a weighted sum.

| Category | What it measures |
|---|---|
| **Speed** | Task completion time relative to par (par → B grade) |
| **Accuracy** | `found_ratio` (recall, 0.60) + `precision` (0.40) |
| **Safety** | Collisions and near-misses |
| **Efficiency** | Coverage-per-metre (0.60) + path length vs par (0.40) |
| **Effectiveness** | Detection completeness (0.65) + exploration coverage % (0.35) |

Par values are derived from a **human perception baseline** (teleop + manual keypress detections, 5 seeds per tier). At par, a category scores ~70 (B grade). Faster, more accurate, or more efficient than par scores above 70; worse scores below.

Grade thresholds: **S** ≥ 95 · **A** ≥ 85 · **B** ≥ 70 · **C** ≥ 55 · **D** ≥ 40 · **F** < 40.

### Proximity-goal scenarios

Proximity scenarios have a different scoring structure — success is binary (reached / not reached), and detection metrics are replaced by navigation efficiency.

| Category | Weight | What it measures |
|---|---|---|
| **Success** | 0.30 | Binary: 100 if robot reaches within `proximity_radius` of target, 0 if not |
| **Time** | 0.25 | Time to reach target vs par (par → B grade) |
| **Safety** | 0.20 | Collisions and near-misses (same as explore_detect) |
| **Efficiency** | 0.25 | `straight_line_distance / path_length` (directness of path; lower is better, capped at 100) |

Raw metrics for proximity scenarios include `proximity_reached` (bool), `min_distance_to_target`, `proximity_path_length`, and `straight_line_distance`.

| Tier | `completion_time_par` (s) | `path_length_par` (m) | `coverage_per_meter_par` |
|---|---|---|---|
| easy | 167.1 | 39.8 | 2.36 |
| medium | 160.2 | 43.8 | 2.10 |
| hard | 212.9 | 61.9 | 1.58 |
| brutal | 182.3 | 64.1 | 1.56 |
| perception_stress | 176.1 | 56.3 | 1.79 |

Full score breakdown is written to `results/` as JSON after each run.

---

## 3. Platform — DerpBot

Ground robot (differential drive — currently the only supported model).

| Property | Value |
|---|---|
| Body | 30 × 20 × 10 cm |
| Max linear speed | 0.5 m/s |
| Max angular speed | 2.0 rad/s |
| Drive command | `geometry_msgs/Twist` |

### Sensors

| Sensor | Topic | Rate | Details |
|---|---|---|---|
| 2-D LiDAR | `/derpbot_0/scan` | ~10 Hz | 360°, 720 rays, 0.5° resolution, 0.15–12 m range, `sensor_msgs/LaserScan` |
| IMU | `/derpbot_0/imu` | 100 Hz | `sensor_msgs/Imu`, **BEST_EFFORT QoS** |
| RGBD — RGB | `/derpbot_0/rgbd/image` | 10 Hz | 640 × 480, 90° H-FOV, forward-facing, `sensor_msgs/Image` |
| RGBD — depth | `/derpbot_0/rgbd/depth_image` | 10 Hz | float32 metres, 0.15–6.0 m range, Gaussian noise σ=0.01 m, `sensor_msgs/Image` |
| RGBD — intrinsics | `/derpbot_0/rgbd/camera_info` | 10 Hz | `sensor_msgs/CameraInfo` — required for 3-D back-projection |
| RGBD — point cloud | `/derpbot_0/rgbd/points` | 10 Hz | `sensor_msgs/PointCloud2` — **off by default**, enable with `--enable-pointcloud` |
| Odometry | `/derpbot_0/odom` | — | `nav_msgs/Odometry`, IMU-fused (EKF) — yaw drift corrected |
| Raw wheel odometry | `/derpbot_0/odom_raw` | — | `nav_msgs/Odometry`, raw wheel-encoder dead-reckoning (for custom sensor fusion) |

### Control & TF

| Topic/Frame | Details |
|---|---|
| `/derpbot_0/cmd_vel` | `geometry_msgs/Twist` — drive command |
| `/derpbot_0/joint_states` | `sensor_msgs/JointState` |
| TF tree | `odom → base_footprint → base_link → lidar_link / camera_link` |

### Odometry

`/derpbot_0/odom` is **IMU-fused** (EKF via `robot_localization`). It combines wheel-encoder odometry with IMU yaw-rate to correct differential-drive yaw drift. Use this topic for all position and heading estimates.

`/derpbot_0/odom_raw` is the **raw wheel-encoder** odometry before EKF fusion. It is available for agents that want to implement their own sensor fusion (e.g. SLAM). Expect significant yaw drift on long runs — up to several metres after 2–3 minutes of driving.

### Detection output

Publish your detections on `/derpbot_0/detections` as `vision_msgs/Detection2DArray`. The scoring system validates each detection against the ground truth.

**Required fields per `Detection2D`:**

| Field | Content |
|---|---|
| `results[0].hypothesis.class_id` | Object type string, e.g. `"fire_extinguisher"` |
| `id` | Persistent per-instance tracking ID (e.g. `"track_23"`). Use a stable ID for the same physical object across frames. |
| `results[0].pose.pose.position.{x,y}` | Estimated position in **map frame** (odom origin = robot spawn pose), in metres. The scorer converts to world frame automatically using the spawn offset. |

**Validation logic:** a detection is a **true positive** if the class matches a known target type, the claimed map-frame position converts to within 1.5 m of a real object of that type in world frame, and there is line of sight. Detections of unknown classes are silently ignored (no penalty). False positives, duplicate positives and localization error reduce your score.

**Safety threshold:** a near-miss is any obstacle within **≤ 0.2 m** of the robot (as measured by LiDAR). Collisions and near-misses both reduce the Safety score. The robot body is 30 × 20 cm (LiDAR-to-surface offset ≈ 0.15 m front/back).

**Note:** a ground-truth oracle can be enabled with `--enable-oracle` (see §4). When active, the simulator bridges its bbox camera directly to `/derpbot_0/detections`, replacing the need for your own vision pipeline. **Off by default** — for scored runs you must publish your own detections using `/derpbot_0/rgbd` and `/derpbot_0/rgbd/depth_image`.

---

## 4. Running a scenario

**Requirements:** ROS 2 Jazzy, Gazebo Harmonic, Python 3.12, a sourced workspace.

```bash
# Single run, headless, random seed
./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless

# Pin layout for reproducibility
./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless --seed 42

# Shorten timeout for iteration
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --headless --timeout 300

# Enable oracle detections (dev/cheat — bbox camera feeds /detections directly)
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --headless --enable-oracle

# Enable RGBD point cloud bridge (high bandwidth — off by default)
./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless --enable-pointcloud
```

Available scenarios and difficulty tiers:

**office_explore_detect** (ascending difficulty):

| Tier | Timeout |
|---|---|
| `easy` | 900 s |
| `medium` | 600 s |
| `hard` | 300 s |
| `brutal` | 180 s |
| `perception_stress` | 600 s |

**basement_find** (proximity-goal):

| Tier | Timeout |
|---|---|
| `easy` | 300 s |
| `medium` | 300 s |

All YAML files are under `config/scenarios/<scenario_name>/`.

Startup takes ~5 s. The scenario ends on `SUCCESS` or `TIME_LIMIT`; scorecard prints to stdout and JSON is written to `results/`.

### Mission description endpoint

Once the scenario is running, fetch the mission description from:

```
GET http://localhost:7400/mission
```

The response is JSON with both a **human-readable** `description` field and **structured** `targets` for rule-based consumption:

```json
{
  "scenario": "office_medium_001",
  "goal_type": "explore_detect",
  "goal": "Explore the environment and locate all required target objects. Report detections before the time limit expires. Thorough exploration earns bonus points.",
  "time_limit_seconds": 600,
  "targets": [
    {"type": "fire_extinguisher", "count": 3,   "count_exact": true},
    {"type": "first_aid_kit",     "count": 2,   "count_exact": true},
    {"type": "exit_sign",       "count_exact": false, "min_count": 1}
  ],
  "status": "running",
  "description": "Explore the environment and locate all target objects within 600s. Targets: exactly 3 fire extinguisher(s), exactly 2 first aid kit(s), at least 1 hazard sign(s) (exact count unknown). Status: running."
}
```

For **proximity-goal** scenarios, the response includes:

```json
{
  "scenario": "basement_find_easy",
  "goal_type": "proximity",
  "goal": "Find the sewer pipe and navigate within 2 metres of it before the time limit expires.",
  "target_object": "sewer_pipe",
  "target_description": "find the sewer pipe and navigate within 2m",
  "proximity_radius": 2.0,
  "time_limit_seconds": 300,
  "targets": [
    {"type": "sewer_pipe", "count": 1, "count_exact": true}
  ],
  "status": "running",
  "description": "Navigate within 2m of the sewer pipe within 300s. Status: running."
}
```

`count_exact: false` means the scenario intentionally withholds the exact count — treat `min_count` as a lower bound. `status` transitions to `"completed"` with an `"outcome"` field (`SUCCESS` / `TIME_LIMIT`) after the run ends; the server stays up until the process exits.

A mission brief is also printed to stdout at scenario start and again just before the scorecard.

### ROS 2 node requirements

- Create your node with `use_sim_time=True` or sensor messages will be silently dropped.
- Subscribe to IMU with `ReliabilityPolicy.BEST_EFFORT`.
- Isolated parallel runs: prefix with `ROS_DOMAIN_ID=N`.

---

## 5. What your agent must do

**Explore-detect scenarios:**

1. **Receive the mission** — fetch the mission description (see section 4) before or at run start.
2. **Explore** the environment — no map is provided; build it from LiDAR and odometry.
3. **Detect objects** — publish `vision_msgs/Detection2DArray` on `/derpbot_0/detections` with class, tracking ID, and map-frame position (see §3).
4. **Navigate safely** — collisions and near-misses reduce your Safety score.
5. **Stop or let the runner end the episode** — the runner polls detections and terminates automatically.

**Proximity-goal scenarios:**

1. **Receive the mission** — fetch the mission description to learn the target object type and proximity radius.
2. **Navigate** to the target — no detection reporting required. The system tracks your ground-truth position via odometry.
3. **Navigate safely** — collisions and near-misses reduce your Safety score.
4. The episode ends automatically when you enter the proximity radius (success) or the time limit expires.

---

## 6. Development tips

- Start with `easy.yaml` for a more forgiving baseline.
- Use `--seed N` to freeze the layout while iterating on your algorithm.
- `scripts/robot_control.py` — manual drive / camera snapshot (dev/cheat tool).
- `scripts/world_state.py` — PNG map + object ground-truth positions (dev/cheat tool).

---

## 7. Benchmark submission

### Protocol

| Property | Value |
|---|---|
| Scenario | `office_explore_detect` |
| Difficulties | easy / medium / hard / brutal / perception_stress |
| Seeds | 1 – 5 (fixed) |
| Runs per seed | 3 |
| **Total per entry** | **15 runs per difficulty** |

Use `--seed N` to pin the layout. **Do not use `--enable-oracle`** for scored runs — you must publish your own detections via `/derpbot_0/rgbd`.

### Result file naming

Each run produces a JSON scorecard in `results/` (e.g. `results/office_easy_001_20260101T120000.json`). Copy the relevant files into `results/submissions/<your-agent-name>/` and rename them to:

```
<difficulty>_seed<N>_run<K>.json
```

Example: `easy_seed1_run1.json`, `brutal_seed3_run2.json`.

### Submission YAML

Copy `results/submissions/example_benchmark_submission.yaml`, fill in your details, and place it alongside your results directory.

**`sandbox_version`** must be the **latest release tag** of robot-sandbox (e.g. `v1.0.0`), not a branch name or commit hash. This ensures reproducibility — the tag pins the exact benchmark version used for your runs. Check available tags with `git tag --list`.

**`code_url`** (optional but recommended) — link to the exact tag or commit of your agent code that produced the results (e.g. a GitHub release URL like `https://github.com/you/your-agent/releases/tag/v1.0`). This ensures anyone can reproduce or inspect the exact code that was tested.

### Validate before submitting

```bash
python3.12 scripts/validate_submission.py path/to/benchmark_submission.yaml
```

Checks: all expected files present, JSON schema valid, `overall_score` in [0, 100], `sandbox_version` tag exists. Exit 0 = pass, exit 1 = fail with itemised errors.

### Submitting

Open a PR adding your YAML + results directory to `results/submissions/`. Submissions that fail validation will not be merged.

Accepted submissions appear on the **[leaderboard](leaderboard.html)**. Ranking is by `found_ratio` (mission targets found / total mission targets); click a score cell to expand per-metric details. After adding a submission, regenerate with `python3.12 scripts/generate_leaderboard.py`.
