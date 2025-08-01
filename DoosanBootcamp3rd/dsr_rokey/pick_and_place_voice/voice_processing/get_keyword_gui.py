import sys
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class KeywordClient(Node):
    def __init__(self, update_result_callback):
        super().__init__("keyword_client_gui")
        self.cli = self.create_client(Trigger, "get_keyword")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("서비스를 기다리는 중...")
        self.req = Trigger.Request()
        self.update_result_callback = update_result_callback

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            self.update_result_callback(response.message)
        except Exception as e:
            self.get_logger().error(f"서비스 호출 실패: {e}")
            self.update_result_callback("오류 발생")


class KeywordGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RF_Robot_program")
        self.setGeometry(100, 100, 500, 300)

        # 📸 이미지 라벨
        self.image_label = QLabel(self)
        pixmap = QPixmap("/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/voice_processing/lms_main_intro.png")

        self.image_label.setPixmap(pixmap)
        self.image_label.setScaledContents(True)
        self.image_label.setFixedHeight(150)  # 이미지 높이 제한 (선택)

        # 🔊 결과 라벨
        self.label = QLabel("음성 인식을 시작합니다...", self)
        self.label.setStyleSheet("font-size: 18px;")
        self.label.setAlignment(Qt.AlignCenter)


        # 📐 전체 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.label)
        self.setLayout(layout)

        # ✅ ROS 클라이언트 노드 실행
        self.node = None
        self.result_text = ""
        self.ros_thread = threading.Thread(target=self.init_ros_node)
        self.ros_thread.start()

    def init_ros_node(self):
        rclpy.init()
        self.node = KeywordClient(self.update_result)
        self.node.send_request()
        # rclpy.spin(self.node)  ← GUI에서는 이거 생략 (callback으로 처리되므로)

    def update_result(self, text):
        self.result_text = text
        QTimer.singleShot(0, self.update_label)

    def update_label(self):
        self.label.setText(f"명령 : {self.result_text}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = KeywordGUI()
    gui.show()
    sys.exit(app.exec_())
