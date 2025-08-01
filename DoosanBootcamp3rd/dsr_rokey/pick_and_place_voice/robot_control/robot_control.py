import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG

#파일 이름과 위치 수정할 것
from od_msg.srv import ArucoId


package_path = get_package_share_directory("pick_and_place_voice")

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
# BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0
JReady = [-9.89, 11.11, 43.36, -0.1, 125.53, -10.58]
HandPos = [811.330, -7.739, 313.690, 174.86, -124.57, -90.23]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans, DR_MV_MOD_REL, DR_MV_MOD_ABS, posx, wait
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


########### Robot Controller ############

aruco_pos = [320.680, 253.920, 429.890, 36.12, 179.99, 42.57]
aruco_num = [250730, 250811, 250812, 260101]
cabinet_pos = {
    "trash" : [657.390, -429.650, 158.710, 107.59, -151.15, -166.66],
    "snacks" : [415.060, -416.350, 20.700, 123.98, 180.00, -142.8],
    "can" : [80.210, -395.120, 105.380, 108.11, 179.99, -73.43],
    "milk" : [-49.060, -366.150, 91.890, 112.99, 180.0, -60.64],
    "fruits" : [224.610, -383.380, 33.580, 117.99, 179.99, -63.2],
    
}

def add_offset(pos, offset): # trans 함수 구현. 테스크 좌표에 offset 주기
    return posx([pos[i] + offset[i] for i in range(6)])

class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.init_robot()

        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position"
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()
        
        # 날짜용 딕셔너리 생성
        # 좌표는 get_target_pos 사용할 것
        self.target_date = {
            'can': [],
            'fruits': [],
            'milk': [],
            'snacks': []
        }

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        # 좌표 변환 (그리퍼 → 베이스)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def robot_control(self):
        target_list = []
        self.get_logger().info("call get_keyword service")
        self.get_logger().info("say 'Hello Rokey' and speak what you want to pick up")
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self, get_keyword_future)
        if get_keyword_future.result().success:
            get_keyword_result = get_keyword_future.result()

            command = get_keyword_result.message.strip()
            tools, action = self.parse_keyword_result(command)
            
            # if action is None or tools is None:
            #     return

            # target_pos = self.get_target_pos(tool)
            # if target_pos is None:
            #     self.get_logger().warn("No target position")
            #     return

            if action == "clean":
                for tool in tools:
                    self.clean(tool)
            elif action == "eat":
                for tool in tools:
                    self.eat(tool)
            elif action == "trash":
                for tool in tools:
                    self.trash(tool)
            else:
                self.get_logger().warn(f"Unknown action: {action}")
                
        else:
            self.get_logger().warn(f"{get_keyword_result.message}")
        return

    def get_target_pos(self, target):
        self.get_position_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(
            self.get_position_request
        )
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

            gripper2cam_path = os.path.join(
                package_path, "resource", "T_gripper2camera.npy"
            )
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET  # DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)  # MIN_DEPTH: float = 2.0

            target_pos = list(td_coord[:3]) + robot_posx[3:]
        return target_pos
    
    def get_aruco_id(self):
        client = self.create_client(ArucoId, '/detect_aruco')
        while not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("waiting for aruco_id...")

        request = ArucoId.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().id
        else:
            self.get_logger().error('서비스 호출 실패')
            return None
    
    def parse_keyword_result(self, message):
        if "/" not in message:
            self.get_logger().warn(f"Invalid command format: {message}")
            return [],None

        tools_str, action = message.strip().lower().split("/")
        tools = tools_str.strip().split()
        action = action.strip()

        for tool in tools:
            if tool not in self.target_date:
                self.get_logger().warn(f"Unsupported tool: {tool}")
                return [], None

        return tools, action



    def init_robot(self):

        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_target(self, target_pos):
        target_pos[2] -= 60
        target_pos_up = add_offset(target_pos, [0, 0, 80,0,0,0]) 

        movel(target_pos_up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS)
        mwait()

        movel(target_pos, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS)
        mwait()
        gripper.close_gripper(force_val = 80)

        while gripper.get_status()[0]:
            print(gripper.get_status())
            time.sleep(0.5)

        # target_pos_up = trans(target_pos, [0, 0, 200, 0, 0, 0]).tolist()
        # target_pos_up = trans(target_pos, [0, 0, 20, 0, 0, 0]).tolist()

        movel([0,0,120,0,0,0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        # movel(BUCKET_POS, vel=VELOCITY, acc=ACC)
        mwait()
            
    def clean(self, tool):
        
        # JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()
        
        target_pos = self.get_target_pos(tool)


        # 🔥 감지 실패하면 건너뛰기
        if target_pos is None:
            self.get_logger().warn(f"[{tool}] 감지 실패, 스킵합니다.")
            return  # 이 도구는 그냥 건너뛰고 다음 도구로 넘어감

        # 🔽 감지 성공 시 진행
        print("Waiting for picking object...")
        self.pick_and_place_target(target_pos)
        
        print("Moving to Aruco_pos...")
        movel(aruco_pos, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS)
        mwait()
        print(aruco_pos)
        
        
        print("Adding date to dict...")
        aruco_id = self.get_aruco_id()
        
        while aruco_id == -1:
            movel([0,0,-30,0,0,0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
            mwait()
            aruco_id = self.get_aruco_id()
            
        if aruco_id == 0:
            print("Out of date!!!!!!!!!")
            movel(cabinet_pos["trash"], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS)
            mwait()
            gripper.open_gripper()
            while gripper.get_status()[0]:
                print(gripper.get_status())
                time.sleep(0.5)
        else:
            self.target_date[tool].append(aruco_num[aruco_id])
            pos_to_go = add_offset(cabinet_pos[tool], [0,0,200,0,0,0])
            movel(pos_to_go, vel=VELOCITY, acc=ACC)
            mwait()
            pos_to_go = add_offset(cabinet_pos[tool], [0,0,-20,0,0,0])
            movel(pos_to_go, vel=VELOCITY, acc=ACC)
            mwait()
            gripper.open_gripper()
            while gripper.get_status()[0]:
                print(gripper.get_status())
                time.sleep(0.5)
        
    def trash(self, tool):
        
        pass
    
    def eat(self, tool):
        
        pass
            


def main(args=None):
    node = RobotController()
    while rclpy.ok():
        node.robot_control()
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()
