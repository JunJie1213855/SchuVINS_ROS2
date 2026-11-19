#!/usr/bin/env python3
"""Run the native node against calibrated synthetic stereo + IMU input.

This checks ROS transport, delayed IMU handling, visual initialization, finite
pose/path/TF output and clean shutdown. It is not a trajectory accuracy test.
"""
import argparse
from collections import deque
import struct
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import cv2
import numpy as np
import yaml


def stereo_images(calib_file):
    calib = yaml.safe_load(Path(calib_file).read_text())['cameras']
    def camera(item):
        c = item['camera']
        fx, fy, cx, cy = c['intrinsics']['data']
        return (np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]], dtype=np.float64),
                np.array(c['distortion']['parameters']['data'], dtype=np.float64),
                np.array(item['T_B_C']['data'], dtype=np.float64).reshape(4, 4), c['distortion']['type'])
    k0, d0, t0, model0 = camera(calib[0])
    k1, d1, t1, model1 = camera(calib[1])
    height, width = calib[0]['camera']['image_height'], calib[0]['camera']['image_width']
    rng = np.random.default_rng(12)
    texture = rng.integers(0, 256, (height, width), dtype=np.uint8)
    texture = cv2.GaussianBlur(texture, (3, 3), 0.7)
    # A textured plane at Z=4 in cam0. Inverse-map cam1 pixels through the
    # calibrated extrinsics and distortion to sample the same physical plane.
    grid = np.stack(np.meshgrid(np.arange(width), np.arange(height)), axis=-1).astype(np.float64)
    undistort = cv2.fisheye.undistortPoints if model1 == 'equidistant' else cv2.undistortPoints
    rays = undistort(grid.reshape(-1, 1, 2), k1, d1).reshape(-1, 2)
    rays = np.column_stack((rays, np.ones(len(rays))))
    transform = np.linalg.inv(t0) @ t1
    directions = rays @ transform[:3, :3].T
    scale = (4.0 - transform[2, 3]) / directions[:, 2]
    points = directions * scale[:, None] + transform[:3, 3]
    project = cv2.fisheye.projectPoints if model0 == 'equidistant' else cv2.projectPoints
    pixels, _ = project(points.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), k0, d0)
    pixels = pixels.reshape(height, width, 2).astype(np.float32)
    right = cv2.remap(texture, pixels[..., 0], pixels[..., 1], cv2.INTER_LINEAR)
    return texture, right


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--node', required=True)
    parser.add_argument('--calib', required=True)
    parser.add_argument('--params', required=True)
    args = parser.parse_args()
    # Keep the integration test away from other ROS nodes on this machine.
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import Image, Imu, PointCloud2
    from nav_msgs.msg import Odometry, Path as PathMsg
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String
    from builtin_interfaces.msg import Time

    left, right = stereo_images(args.calib)
    imu_params = yaml.safe_load(Path(args.calib).read_text())['imu_params']
    delay_ns = round(imu_params['delay_imu_cam'] * 1e9)
    rclpy.init()
    node = rclpy.create_node('schurvins_smoke_test')
    pubs = [node.create_publisher(Image, t, qos_profile_sensor_data) for t in ['/cam0/image_raw', '/cam1/image_raw']]
    imu_pub = node.create_publisher(Imu, '/imu0', qos_profile_sensor_data)
    command_pub = node.create_publisher(String, '/svo/remote_key', 10)
    odometry, paths, transforms = [], [], []
    images = [deque(maxlen=3) for _ in range(3)]
    clouds = deque(maxlen=50)
    cloud_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [
        node.create_subscription(Odometry, '/svo/odometry', odometry.append, 10),
        node.create_subscription(PathMsg, '/svo/path', paths.append, 10),
        node.create_subscription(TFMessage, '/tf', transforms.append, 10),
        node.create_subscription(PointCloud2, '/svo/landmarks', clouds.append, cloud_qos),
    ]
    for messages, topic in zip(images, ['/svo/tracked_image', '/svo/cam0/tracked_image', '/svo/cam1/tracked_image']):
        subscriptions.append(node.create_subscription(Image, topic, messages.append, 2))
    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(0.02, end - time.monotonic()))
    def stamp(ns):
        return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)
    def publish_imu(ns):
        msg = Imu()
        msg.header.stamp = stamp(ns)
        msg.linear_acceleration.z = imu_params['g']
        imu_pub.publish(msg)
    def publish_images(ns, images):
        for pub, image in zip(pubs, images):
            msg = Image()
            msg.header.stamp = stamp(ns)
            msg.height, msg.width = image.shape
            msg.encoding = 'mono8'
            msg.step = msg.width
            msg.data = image.tobytes()
            pub.publish(msg)
    with tempfile.TemporaryDirectory(prefix='schurvins-test-') as folder:
        log_path = Path(folder) / 'node.log'
        trajectory_path = Path(folder) / 'trajectory.txt'
        with log_path.open('w') as log:
            process = subprocess.Popen([args.node, '--ros-args', '--params-file', args.params,
                '-p', 'calib_file:=' + args.calib, '-p', 'trajectory_file:=' + str(trajectory_path)], stdout=log, stderr=log)
            try:
                deadline = time.monotonic() + 20
                while not all(p.get_subscription_count() for p in [*pubs, imu_pub]):
                    if process.poll() is not None:
                        raise AssertionError('Node exited during startup')
                    if time.monotonic() > deadline:
                        raise AssertionError('ROS subscriptions were not discovered')
                    spin(0.1)
                # Images precede IMU: the node must buffer them without producing
                # output or entering the estimator with an incomplete IMU buffer.
                start = 1000000000000
                publish_images(start, (left, right))
                spin(0.2)
                assert not odometry, 'Output before receiving any IMU'
                for i in range(201):
                    publish_imu(start - 1000000000 + i * 5000000)
                    if i % 10 == 0:
                        spin(0.01)
                spin(0.4)
                # Exercise malformed dimensions, then regular synchronized data.
                publish_images(start + 25000000, (left[:100], right[:100]))
                last_imu = start
                for frame in range(1, 31):
                    image_time = start + frame * 50000000
                    publish_images(image_time, (left, right))
                    while last_imu <= image_time + 5000000:
                        last_imu += 5000000
                        publish_imu(last_imu)
                    spin(0.10)
                spin(1.0)
                assert process.poll() is None, 'Node crashed while processing sensor data'
                assert len(odometry) >= 5, f'Expected tracking output, received {len(odometry)} poses'
                assert paths and len(paths[-1].poses) >= 5, 'No accumulated path'
                assert transforms, 'No TF output'
                assert all(images), 'Missing annotated stereo/camera images'
                for index, messages in enumerate(images):
                    msg = messages[-1]
                    width = left.shape[1] * (2 if index == 0 else 1)
                    assert (msg.height, msg.width, msg.encoding) == (left.shape[0], width, 'bgr8')
                    pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :width*3].reshape(msg.height, width, 3)
                    assert np.any(pixels[..., 0] != pixels[..., 1]), 'No colored feature overlay'
                assert clouds and clouds[-1].width > 0, 'No estimated landmarks'
                late_clouds = []
                subscriptions.append(node.create_subscription(PointCloud2, '/svo/landmarks', late_clouds.append, cloud_qos))
                spin(0.5)
                assert late_clouds and late_clouds[-1].width > 0, 'Late RViz subscriber did not receive retained local map' 
                # Invalid/out-of-order images and duplicate IMU must not crash.
                publish_images(start, (left, right))
                publish_imu(last_imu)
                spin(0.2)
                before_reset = len(odometry)
                command_pub.publish(String(data='r'))
                spin(0.3)
                assert clouds[-1].width == 0, 'Reset did not clear retained landmarks'
                for frame in range(31, 34):
                    image_time = start + frame * 50000000
                    publish_images(image_time, (left, right))
                    while last_imu <= image_time + 5000000:
                        last_imu += 5000000
                        publish_imu(last_imu)
                    spin(0.10)
                assert len(odometry) == before_reset, 'Reset did not pause image processing'
                command_pub.publish(String(data='s'))
                spin(0.2)
                for frame in range(34, 45):
                    image_time = start + frame * 50000000
                    publish_images(image_time, (left, right))
                    while last_imu <= image_time + 5000000:
                        last_imu += 5000000
                        publish_imu(last_imu)
                    spin(0.10)
                spin(0.5)
                assert len(odometry) >= before_reset + 5, 'Filter did not recover after reset/start'
                times = []
                for msg in odometry:
                    pose = msg.pose.pose
                    values = [pose.position.x, pose.position.y, pose.position.z,
                              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                    assert all(math.isfinite(v) for v in values), 'Nonfinite pose'
                    assert abs(sum(v*v for v in values[3:]) - 1) < 1e-5, 'Nonunit rotation'
                    assert msg.header.frame_id == 'world' and msg.child_frame_id == 'imu'
                    times.append(msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec)
                assert all(a < b for a, b in zip(times, times[1:])), 'Non-increasing output timestamps'
                expected = {start + frame * 50000000 - delay_ns for frame in range(45)}
                assert all(t in expected for t in times), 'Camera-to-IMU offset missing or applied twice'
                retained_ids = set()
                gray_values = set()
                for cloud in clouds:
                    assert cloud.header.frame_id == 'world' and cloud.is_dense
                    fields = {f.name: f.offset for f in cloud.fields}
                    assert set(fields) == {'x', 'y', 'z', 'rgb', 'id'}
                    ids = []
                    for i in range(cloud.width):
                        base = i * cloud.point_step
                        prefix = '>' if cloud.is_bigendian else '<'
                        xyz = [struct.unpack_from(prefix+'f', cloud.data, base+fields[k])[0] for k in ('x', 'y', 'z')]
                        assert all(math.isfinite(v) for v in xyz), 'Nonfinite landmark'
                        ids.append(struct.unpack_from(prefix+'i', cloud.data, base+fields['id'])[0])
                        rgb = struct.unpack_from(prefix+'I', cloud.data, base+fields['rgb'])[0]
                        red, green, blue = (rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255
                        assert red == green == blue, 'MONO8 landmarks must use actual grayscale RGB, not artificial colors'
                        gray_values.add(red)
                    current_ids = set(ids)
                    assert len(ids) == len(current_ids), 'Duplicate landmark IDs in a map snapshot'
                    if not current_ids:  # Explicit estimator/map reset.
                        retained_ids.clear()
                    else:
                        assert retained_ids <= current_ids, 'Historical landmarks disappeared from the cumulative cloud'
                        retained_ids = current_ids
                assert len(gray_values) > 1, 'Landmark colors were not sampled from the textured source image' 
                assert clouds[-1].width > 0, 'Landmarks did not recover after reset/start'
                for messages in images:
                    for msg in messages:
                        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                        assert stamp_ns in expected, 'Annotated image timestamp mismatch'
                command_pub.publish(String(data='q'))
                process.wait(timeout=10)
                assert process.returncode == 0, f'Unclean exit: {process.returncode}'
                assert len(trajectory_path.read_text().splitlines()) >= 5, 'Trajectory was not flushed'
                print(f'PASS: {len(odometry)} finite poses; path, TF, feature overlays, historical grayscale XYZRGB landmarks, late subscriber/reset, delayed IMU, timestamp/offset checks and clean shutdown')
            except Exception:
                print(log_path.read_text())
                raise
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
