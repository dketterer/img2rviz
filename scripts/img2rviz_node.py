#!/usr/bin/env python3

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, std_msgs
import cv2
import numpy as np


def img2point_cloud(image: np.array, scale_factor: float, gray_scale: bool, pixel_per_meter: float) -> PointCloud2:
    # downsample
    orig_height, orig_width = image.shape[:2]
    if gray_scale:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    new_size = (int(orig_height / scale_factor), int(orig_width / scale_factor))
    img_small = cv2.resize(image, new_size, interpolation=cv2.INTER_CUBIC)
    new_height, new_width = img_small.shape[:2]
    img_len = new_width * new_height

    if gray_scale:
        data = np.zeros(img_len, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint8)
        ])
    else:
        # normalize
        img_small = img_small.astype(np.float32)
        img_small = img_small / 255.
        data = np.zeros(img_len, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('r', np.float32),
            ('g', np.float32),
            ('b', np.float32)
        ])

    # scale back to orig size
    data['x'] = np.tile(np.linspace(0, orig_width, new_width) / pixel_per_meter, new_height)
    data['y'] = ((np.repeat(np.linspace(0, orig_height, new_height) / pixel_per_meter,
                            new_width) - orig_height / pixel_per_meter) * -1)
    data['z'] = np.zeros(img_len)

    if gray_scale:
        data['rgb'] = np.reshape(img_small, img_len)
    else:
        data['r'] = np.reshape(img_small[:, :, 2], img_len)
        data['g'] = np.reshape(img_small[:, :, 1], img_len)
        data['b'] = np.reshape(img_small[:, :, 0], img_len)

    pc_msg = ros_numpy.msgify(PointCloud2, data)
    return pc_msg


def publish_image_as_pointcloud(point_cloud_publisher: rospy.Publisher, message: PointCloud2):
    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()
    message.header = h
    message.header.frame_id = 'img2rviz'
    point_cloud_publisher.publish(message)


if __name__ == '__main__':
    frequency = rospy.get_param('/img2rviz_publisher/frequency', 1)
    image_path = rospy.get_param('/img2rviz_publisher/imgPath', '')
    gray_scale = rospy.get_param('/img2rviz_publisher/grayscale')
    scale_factor = rospy.get_param('/img2rviz_publisher/scaleFactor', 4.)
    pixel_per_meter = rospy.get_param('/img2rviz_publisher/pixel_per_meter', 500)

    print('Start Image Publisher')
    print(f'Image: {image_path}')
    print(f'Grayscale: {"on" if gray_scale else "off"}')
    print(f'Scale (divide) by factor {scale_factor}')
    print(f'Publish every {1 / frequency}s')
    print(f'Pixel per meter : {pixel_per_meter}')

    assert (image_path), 'No image path!'

    rospy.init_node('img2rviz')

    point_cloud_publisher = rospy.Publisher('/visualization_plane', PointCloud2, queue_size=2)

    image = cv2.imread(image_path)
    assert (image is not None), f'No image found at {image_path}'

    pc_msg = img2point_cloud(image, scale_factor, gray_scale, pixel_per_meter)

    r = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        publish_image_as_pointcloud(point_cloud_publisher, pc_msg)
        r.sleep()
