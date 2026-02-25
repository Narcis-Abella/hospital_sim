## Pipeline Design

Each sensor follows the same pattern:

```
[Gazebo Plugin] → gz topic → [ros_gz_bridge] → /sensor_raw → [C++ Noise Node] → /sensor
```

The bridge uses **Reliable QoS** for all sensor topics (default for `parameter_bridge`).
Noise nodes subscribe with matching Reliable QoS. A Reliable subscriber connected to a
BestEffort publisher will silently fail to match in ROS 2 DDS — always verify with
`ros2 topic info <topic> --verbose` when changing the bridge implementation.

## IMU Model Detail

The WT901C gyroscope bias is modeled as a first-order Gauss-Markov process:

```
b[k] = α·b[k-1] + w[k]

where:
  α = exp(-dt / τ)           # correlation coefficient
  τ = 400 s                  # bias time constant (gyro)
  w ~ N(0, σ_b · √(1-α²))   # driving noise
```

Full gyroscope measurement model:

```
ω_measured = ω_true + η + b + K_g · a

where:
  η ~ N(0, σ_ARW²)           # white noise (angle random walk)
  b                           # Gauss-Markov bias state
  K_g = 0.00175 / 9.81       # G-sensitivity (rad/s per m/s²)
  a                           # linear acceleration vector
```

ADC quantization uses the full-scale range / 16-bit resolution:

```
gyro_LSB  = (2 × 2000°/s × π/180) / 65536   # rad/s per LSB
accel_LSB = (2 × 16g × 9.81)      / 65536   # m/s² per LSB
```

## Covariance Matrix Assignment

Noise nodes assign covariance matrices to every published message. This is critical for
EKF-based fusion (e.g., `robot_localization`): if covariances are zero or `-1` (unknown),
the filter either ignores the sensor or behaves numerically unstably.

Indices below refer to the flat 36-element row-major array of the 6×6 covariance matrix.

| Matrix | Index | Value | Rationale |
|--------|-------|-------|-----------|
| `pose.covariance` | [0], [7] | 0.005 | x, y position uncertainty |
| `pose.covariance` | [14],[21],[28] | 1e6 | z, roll, pitch: unmeasured (planar robot) |
| `pose.covariance` | [35] | 0.08 | yaw: high → EKF trusts IMU over odometry |
| `twist.covariance` | [0] | 0.001 | vx: reliable encoder measurement |
| `twist.covariance` | [7] | 0.0001 | vy: near-zero (non-holonomic) |
| `twist.covariance` | [35] | 0.05 | ωz: imprecise differential drive |
| `angular_velocity_covariance` | [0],[4],[8] | σ²_gyro_total | white + bias variance |
| `linear_acceleration_covariance` | [0],[4],[8] | σ²_accel_total | white + bias variance |
