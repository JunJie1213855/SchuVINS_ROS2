#!/usr/bin/env python3
"""Convert the measured OpenVIO TideSense rig to the upstream SVO YAML schema."""
import argparse
import hashlib
import math
from pathlib import Path
import yaml


def convert(source, imu_source):
    # OpenCV's YAML 1.0 directive is not understood by PyYAML.
    text = source.read_text()
    rig = yaml.safe_load('\n'.join(line for line in text.splitlines() if not line.startswith('%YAML:')))
    imu = yaml.safe_load(imu_source.read_text())['imu0']
    cameras = []
    for key, extrinsic, label in [('camera', 'imu_from_camera', 'cam0_right'),
                                  ('camera1', 'imu_from_camera1', 'cam1_left')]:
        camera = rig[key]
        if camera['model'] != 'FISHEYE_EQUIDISTANT' or len(camera['params']) != 8:
            raise ValueError('Expected calibrated eight-parameter equidistant cameras')
        transform = rig['rig'][extrinsic]
        if len(transform) != 4 or any(len(row) != 4 for row in transform):
            raise ValueError('Expected a 4x4 IMU-from-camera transformation')
        if transform[-1] != [0, 0, 0, 1]:
            raise ValueError('Invalid homogeneous transformation')
        if not all(math.isfinite(x) for row in transform for x in row):
            raise ValueError('Nonfinite extrinsics')
        values = camera['params']
        cameras.append({'camera': {'label': label, 'type': 'pinhole',
            'image_width': camera['width'], 'image_height': camera['height'],
            'intrinsics': {'rows': 4, 'cols': 1, 'data': values[:4]},
            'distortion': {'type': 'equidistant', 'parameters': {'rows': 4, 'cols': 1, 'data': values[4:]}}},
            'T_B_C': {'rows': 4, 'cols': 4, 'data': [v for row in transform for v in row]}})
    noise = rig['imu']
    for key, other in [('gyro_noise', 'gyroscope_noise_density'), ('accel_noise', 'accelerometer_noise_density'),
                       ('gyro_walk', 'gyroscope_random_walk'), ('accel_walk', 'accelerometer_random_walk')]:
        if not math.isclose(noise[key], imu[other], rel_tol=1e-10):
            raise ValueError(f'Camera rig and driver IMU calibration disagree: {key}')
    return {'label': 'TideSense_OV9281_SC7U22', 'cameras': cameras,
        'imu_params': {
            # OpenVIO: t_imu = t_camera + clock_offset.
            # SVO: t_imu = t_camera - delay_imu_cam.
            'delay_imu_cam': -rig['rig']['clock_offset'], 'max_imu_delta_t': 0.01,
            # Legacy SVO requires these fields. They retain its defaults; they
            # are not measured sensor specifications and the Schur filter does not use them.
            'acc_max': 150.0, 'omega_max': 7.8,
            'sigma_omega_c': noise['gyro_noise'], 'sigma_acc_c': noise['accel_noise'],
            'sigma_omega_bias_c': noise['gyro_walk'], 'sigma_acc_bias_c': noise['accel_walk'],
            'g': noise['gravity_magnitude'], 'imu_rate': imu['update_rate']},
        # No measured bias prior is provided by the source configuration.
        'imu_initialization': {'velocity': [0.0]*3, 'omega_bias': [0.0]*3, 'acc_bias': [0.0]*3,
            'velocity_sigma': 2.0, 'omega_bias_sigma': 0.01, 'acc_bias_sigma': 0.1}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('/home/ros/ros_ws/openvio/config/tidesense_stereo.yaml'))
    parser.add_argument('--imu', type=Path, default=Path('/home/ros/lib/tideSense/TideDrive/ros2/src/tidesense_ros2/config/imu.yaml'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = convert(args.source, args.imu)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    header = '# Converted from measured TideSense calibration; do not flip or swap images.\n'
    header += '# Source SHA256: ' + hashlib.sha256(args.source.read_bytes()).hexdigest() + '\n'
    header += '# cam0=physical right; cam1=physical left; driver corrects IMU intrinsics once.\n'
    header += '# delay_imu_cam is applied once by the ROS 2 adapter; no further bag/driver time shift.\n'
    args.output.write_text(header + yaml.safe_dump(data, sort_keys=False))
    print(args.output)


if __name__ == '__main__':
    main()
