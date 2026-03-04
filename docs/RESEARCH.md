# Sensor Noise Models — Research & Parameter Justification

> **Validation status:** All parameters in this document are **estimates derived from vendor datasheets and field observation**. No systematic calibration campaign has been performed on the physical platform. The values are designed to be plausible and conservative, not validated against recorded ground truth. A real-hardware measurement pass (Allan variance on the physical IMU, rosbag-based odometry drift characterisation) is explicitly listed as future work.

This document consolidates the scientific rationale behind every noise model implemented in `src/`. Its purpose is twofold: to make the design decisions transparent and reproducible, and to position the repository as a metrologically grounded simulation framework rather than a collection of ad-hoc noise injections.

---

## 1. IMU — WitMotion WT901C-TTL (`noisy_imu.cpp`)

### 1.1 Sensor identification

The WT901C-TTL is a consumer-grade AHRS module built around the **InvenSense MPU-9250** MEMS die. The raw inertial measurements (gyroscope + accelerometer) come directly from the MPU-9250; the onboard microcontroller applies WitMotion's Kalman filter to produce fused angle outputs. In this pipeline the raw IMU topic is consumed before any WitMotion fusion, so the relevant noise specifications are those of the bare MPU-9250.

**Datasheet reference:** [InvenSense MPU-9250 Product Specification Rev 1.1 (PS-MPU-9250A-01)](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)

### 1.2 Gyroscope model

The node implements a **first-order Gauss-Markov bias process** superimposed on white measurement noise, plus G-sensitivity cross-coupling and ADC quantization.

#### White noise (Angle Random Walk)

The MPU-9250 datasheet (Table 1, §3.1) reports:

```
Rate Noise Spectral Density: 0.01 °/s/√Hz  (typical, DLPF enabled)
```

Converting to rad/s/√Hz and integrating over a 100 Hz bandwidth:

```
PSD = 0.01 × (π/180) ≈ 1.745 × 10⁻⁴ rad/s/√Hz
σ_white = PSD × √(BW) = 1.745e-4 × √100 ≈ 0.00175 rad/s
```

This directly matches `gyro_white_std_ = 0.00175` in the code.

#### Bias instability (Gauss-Markov process)

The MPU-9250 datasheet does not publish a bias instability figure directly; it reports **Initial ZRO (Zero Rate Output) Tolerance ±5 °/s** and **ZRO Variation Over Temperature ±30 °/s**. These are worst-case bounds, not in-run instability.

For MEMS gyroscopes of this class, published literature and Allan variance analyses consistently report in-run bias instability in the range **0.01–0.05 °/s (1σ)**. The value used:

```
gyro_bias_std_ = 0.0003 rad/s  ≈  0.017 °/s
```

This is at the optimistic end of the range, appropriate for a temperature-stabilised, stationary-start scenario. The **correlation time τ = 400 s** is a standard assumption for MEMS gyroscopes operating over minutes-to-hours; it ensures the bias evolves slowly enough to be observable but not so fast as to be indistinguishable from white noise.

> **Validation gap:** The bias instability and correlation time have not been characterised via Allan variance on the physical WT901C unit. These are literature-informed estimates. A static rosbag recording of ≥30 min at rest would allow an Allan variance plot to validate or correct these values.

#### G-sensitivity

The MPU-9250 datasheet reports **Cross-Axis Sensitivity ±2%** (Table 1). This describes the fraction of linear acceleration that bleeds into the gyroscope output. The implementation uses:

```
G_sensitivity_ = gyro_white_std_ / 9.81  ≈  1.78 × 10⁻⁴ (rad/s)/(m/s²)
```

> **Note:** This derivation (ARW / g) is not a standard way to express G-sensitivity. The correct parameter would be taken directly from a characterised G-sensitivity coefficient, e.g. in units of `(rad/s)/(m/s²)` derived from the datasheet cross-axis spec. The current value is numerically small and produces a negligible effect in practice, but it is **not metrologically justified**. Fixing this requires either a dedicated vibration test on the physical unit or a proper conversion of the ±2% cross-axis figure. Flagged as known approximation; functional impact is minimal.

#### ADC quantization

The MPU-9250 uses **16-bit ADCs** with a selectable full-scale range. At FS_SEL=3 (±2000 °/s), matching the WT901C default:

```
gyro_LSB = (2 × 2000°/s × π/180) / 65536 = 1.065 × 10⁻³ rad/s per LSB
```

For the accelerometer at AFS_SEL=3 (±16g), also matching WT901C defaults:

```
accel_LSB = (2 × 16 × 9.81) / 65536 = 4.789 × 10⁻³ m/s² per LSB
```

Both values are computed from the sensitivity scale factors in Tables 1 and 2 of the MPU-9250 datasheet.

#### Accelerometer white noise

The MPU-9250 datasheet (Table 2, §3.2) reports:

```
Noise Power Spectral Density (low-noise mode): 300 µg/√Hz
```

Converting and integrating over 100 Hz bandwidth:

```
σ_accel_white = 300e-6 × 9.81 × √100 ≈ 0.0294 m/s²
```

The code uses `accel_white_std_ = 0.020 m/s²`, which is slightly optimistic relative to the raw PSD. This implicitly assumes the WitMotion module's output benefits from the onboard Kalman filter even in raw mode, or that the effective bandwidth is lower than 100 Hz. This is an acknowledged approximation.

---

## 2. 2D LiDAR — SLAMTEC RPLiDAR A2M12 (`noisy_lidar.cpp`)

### 2.1 Datasheet reference

**[SLAMTEC RPLIDAR A2 Specifications](https://www.slamtec.com/en/Lidar/A2Spec)**

### 2.2 Noise model

The RPLiDAR A2M12 specification table reports the following ranging accuracy:

| Distance range | Accuracy (typical) |
|---|---|
| ≤ 3 m | ±1% of range |
| 3 – 5 m | ±2% of range |
| 5 – 25 m | ±2.5% of range |

The model uses a single proportional Gaussian:

```
σ(r) = max(min_noise, rel_noise × r)

rel_noise = 0.01  (1% — conservative bound valid for ≤3 m)
min_noise  = 0.003 m  (3 mm — absolute floor for very short ranges)
```

Using 1% across the full range is conservative: at distances >3 m the actual A2M12 accuracy is up to 2.5%, so the simulation slightly *underestimates* real noise at medium and long range. This is an acceptable bias in the context of SLAM stress-testing.

> **Note:** The A2M12 specification uses the term "accuracy" (systematic bound), not 1σ standard deviation. Treating it as a Gaussian σ is a modelling simplification. A more rigorous model would separate systematic and random components.

---

## 3. 3D LiDAR — Livox Mid-360 (`noisy_livox_mid360.cpp`)

### 3.1 Datasheet reference

**[Livox Mid-360 Technical Specifications](https://www.livoxtech.com/mid-360/specs)**

### 3.2 Noise model

The Livox Mid-360 datasheet reports range precision as a direct 1σ value:

```
Range Precision (1σ):
  ≤ 2 cm  @ 10 m  (80% reflectivity target, 25°C)
  ≤ 3 cm  @ 0.2 m (80% reflectivity target, 25°C)
```

This directly provides a 1σ value, making it more directly usable than the A2M12 figure. The parameters chosen:

```
rel_noise = 0.005  (0.5% → σ ≈ 2 cm at 4 m — matches ≤2 cm spec at mid-range)
min_noise = 0.002  (2 mm  — absolute floor, conservative relative to the 3 cm spec at 0.2 m)
```

The noise is applied **radially** (along line-of-sight), preserving azimuth and elevation angles. This matches the physical mechanism: ToF/phase-shift LiDAR errors manifest as range errors, not angular errors.

```
ratio = 1 + Δr/r
(x', y', z') = ratio × (x, y, z)
```

The angular precision spec of the Mid-360 (< 0.15° 1σ) is not modelled; at typical indoor ranges (1–10 m) it contributes < 2.6 cm lateral error, comparable to the range noise already applied.

---

## 4. 3D LiDAR — Livox Mid-70 (`noisy_livox_mid70.cpp`)

The Mid-70 shares the same underlying Livox ToF architecture as the Mid-360, and its datasheet reports the same **±2 cm ranging accuracy** at similar conditions.

**[Livox Mid-70 Product Page / Specifications](https://www.livoxtech.com/mid-70)**

Parameters are identical to the Mid-360 node:

```
rel_noise = 0.005
min_noise = 0.002
```

> **Code note:** `noisy_livox_mid70.cpp` and `noisy_livox_mid360.cpp` share identical logic with different node names and topic routes. Consolidating them into a single parametric node is a known refactoring opportunity.

---

## 5. Wheel Odometry — AgileX Tracer 2.0 (`noisy_odom.cpp`)

### 5.1 Context and limitations

The AgileX Tracer 2.0 uses brushless DC motors with integrated encoders. No public datasheet for the specific encoder resolution or systematic odometry error characterisation has been located. The noise parameters are therefore **empirically motivated**, not datasheet-derived.

**[AgileX Tracer 2.0 Specifications (Trossen Robotics)](https://docs.trossenrobotics.com/agilex_tracer_docs/specifications.html)**

### 5.2 Linear slip (`lin_noise_ratio = 0.02`, i.e. 2%)

A 2% linear velocity noise is consistent with characterisation studies of differential-drive robots with rubber wheels on smooth indoor floors. It models encoder quantization and minor wheel slip, and is broadly used as a conservative starting point for AGV odometry in the ROS 2 / Nav2 community.

### 5.3 Angular noise (`ang_noise_ratio = 0.08`, i.e. 8%)

> **Important caveat:** This value is **not derived from a datasheet** and has not been validated with a structured measurement protocol. It is based on direct observation of the physical Tracer 2.0: the robot exhibits a **visibly large yaw error after in-place rotations** — e.g. 90° commanded turns produce visible heading offsets in the range of several degrees. The 8% figure is a conservative estimate intended to reproduce this behaviour in simulation.
>
> Architecturally, the high angular error relative to linear (8% vs. 2%) reflects the well-known dominance of heading error in differential-drive dead reckoning: small encoder imbalances between left and right wheels amplify into large yaw divergence over time.
>
> **Pending validation:** A proper characterisation requires commanding known rotation angles (90°, 180°, 360°) via the ROS 2 control interface, recording the resulting `/odom` topic, and computing the ratio of actual vs. commanded yaw over multiple trials on the deployment floor surface. This is explicitly deferred to the hardware validation phase (Plan Phase 3, post-exams).

### 5.4 Yaw random walk (`yaw_drift_rate = 0.005 rad/m`)

This models the **systematic, unbounded heading drift** that accumulates with distance travelled, distinct from the per-step stochastic noise above. At 0.005 rad/m, after 10 m of travel the expected drift is ±0.05 rad (≈ ±2.9°). This is a standard-order-of-magnitude assumption for consumer-grade encoders without gyroscope correction, consistent with values used in the ROS 2 `robot_localization` package documentation examples.

The drift is intentionally **unbounded** — it models real encoder heading error that compounds without loop closure, and is designed to stress-test SLAM loop-closure and IMU fusion rather than be physically exact.

### 5.5 Covariance matrices

The covariance values assigned to `pose.covariance` and `twist.covariance` are designed to correctly communicate uncertainty to the downstream EKF (`robot_localization`):

| Field | Value | Rationale |
|---|---|---|
| `pose.yaw` | 0.08 rad² | High uncertainty — EKF should weight IMU heading over odometry |
| `pose.z`, `roll`, `pitch` | 1×10⁶ | Unmeasured DOFs (planar robot); signals "ignore this axis" to EKF |
| `twist.vy` | 1×10⁻⁴ m²/s² | Near-zero; enforces non-holonomic constraint |
| `twist.vx` | 0.001 m²/s² | Reliable encoder measurement |
| `twist.wz` | 0.05 rad²/s² | Imprecise differential drive |

---

## 6. Summary Table

| Node | Sensor | Model | Primary source | Validation status |
|---|---|---|---|---|
| `noisy_imu.cpp` | WT901C / MPU-9250 | Gauss-Markov bias + white noise + G-sensitivity + ADC quant. | MPU-9250 datasheet §3.1–3.2 | Partial — white noise and ADC from datasheet; bias τ and instability from literature |
| `noisy_lidar.cpp` | RPLiDAR A2M12 | Proportional Gaussian `σ = max(min, rel·r)` | SLAMTEC A2 spec page | Conservative estimate — accuracy treated as 1σ (simplification) |
| `noisy_livox_mid360.cpp` | Livox Mid-360 | Radial Gaussian `σ = max(min, rel·r)` | Livox Mid-360 datasheet (1σ spec) | Direct datasheet match |
| `noisy_livox_mid70.cpp` | Livox Mid-70 | Radial Gaussian (same as Mid-360) | Livox Mid-70 product page | Same architecture, assumed same accuracy |
| `noisy_odom.cpp` | Tracer 2.0 encoders | Proportional slip + yaw random walk + systematic drift | Field observation + literature | **Linear: literature-consistent; Angular 8%: empirical observation, unvalidated — pending hardware measurement** |

---

## 7. Future Work

1. **Allan variance of physical WT901C:** Record ≥30 min static rosbag on the Jetson Orin NX, run Allan deviation analysis to characterise `gyro_bias_tau_`, `gyro_bias_std_`, and confirm `gyro_white_std_`.
2. **Odometry angular error characterisation:** Command known rotation angles on the Tracer 2.0, record rosbags, compute yaw error statistics. Replace the empirical 8% with a measured value and update `ang_noise_ratio` accordingly.
3. **G-sensitivity correction:** Derive the coefficient correctly from the MPU-9250 ±2% cross-axis sensitivity spec, or measure it on the physical unit. Remove the current ARW-based approximation.
4. **LiDAR reflectivity dependence:** Both Livox and RPLiDAR accuracy degrade on low-reflectivity surfaces (dark flooring, glass). A reflectivity-modulated σ would improve fidelity for hospital environments.
5. **Mid-70 / Mid-360 refactor:** Consolidate into a single `noisy_livox.cpp` node with a `sensor_model` parameter to eliminate code duplication.
