
import cv2
import rclpy
from rclpy.node import Node
from realsense import ImgNode
from scipy.spatial.transform import Rotation
from onrobot import RG
from ultralytics import YOLO
import time
import numpy as np

import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

from DSR_ROBOT2 import (
    get_current_posx,
    movej,
    movel,
    wait,
)

from DR_common2 import posx, posj


class AutoPickWithYOLO(Node):
    def __init__(self):
        super().__init__("auto_pick_node")

        self.img_node = ImgNode()
        rclpy.spin_once(self.img_node)
        time.sleep(1)

        self.intrinsics = self.img_node.get_camera_intrinsic()
        self.gripper2cam = np.load("T_gripper2camera.npy")
        self.JReady = posj([0, 0, 90, 0, 90, -90])
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

        self.model = YOLO("best.pt")  # 여기에 너의 모델 경로 입력

    def get_camera_pos(self, center_x, center_y, center_z, intrinsics):
        camera_x = (center_x - intrinsics["ppx"]) * center_z / intrinsics["fx"]
        camera_y = (center_y - intrinsics["ppy"]) * center_z / intrinsics["fy"]
        camera_z = center_z
        return (camera_x, camera_y, camera_z)

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords):
        coord = np.append(np.array(camera_coords), 1)
        base2gripper = self.get_robot_pose_matrix(*get_current_posx()[0])
        base2cam = base2gripper @ self.gripper2cam
        td_coord = np.dot(base2cam, coord)
        return td_coord[:3]

    def pick_and_drop(self, x, y, z):
        current_pos = get_current_posx()[0]
        pick_pos = posx([x, y, z, current_pos[3], current_pos[4], current_pos[5]])
        movel(pick_pos, time=3)
        self.gripper.close_gripper()
        wait(1)
        movej(self.JReady, time=3)
        self.gripper.open_gripper()
        wait(1)

    def segment_objects_yolo(self, color_image, depth_image):
        results = self.model(color_image)[0]
        centers = []

        if results.masks is None:
            return centers

        for mask in results.masks.data:
            mask_np = mask.cpu().numpy().astype(np.uint8)
            M = cv2.moments(mask_np)
            if M["m00"] > 100:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]:
                    z = depth_image[cy, cx]
                    if z > 0:
                        centers.append((cx, cy, z))
        return centers

    def sort_by_distance(self, coords_3d):
        return sorted(coords_3d, key=lambda pt: np.linalg.norm(pt))

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self.img_node)
            color = self.img_node.get_color_frame()
            depth = self.img_node.get_depth_frame()

            if color is None or depth is None:
                self.get_logger().warn("No image frame.")
                continue

            centers = self.segment_objects_yolo(color, depth)
            coords_camera = []
            for (x, y, z) in centers:
                pt = self.get_camera_pos(x, y, z, self.intrinsics)
                coords_camera.append(pt)

            coords_camera = self.sort_by_distance(coords_camera)

            for pt in coords_camera:
                base_pt = self.transform_to_base(pt)
                self.pick_and_drop(*base_pt)


def main():
    rclpy.init()
    node = rclpy.create_node("dsr_example_demo_py", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    auto_node = AutoPickWithYOLO()
    auto_node.run()


if __name__ == "__main__":
    main()