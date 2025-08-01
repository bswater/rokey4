import cv2
import cv2.aruco as aruco
from collections import Counter
import rclpy
from rclpy.node import Node
from od_msg.srv import ArucoId  # 커스텀 srv

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.srv = self.create_service(ArucoId, 'detect_aruco', self.handle_detect_aruco)
        self.get_logger().info("✅ ArUco 서비스 서버 준비 완료!")

    def handle_detect_aruco(self, request, response):
        self.get_logger().info("🔍 마커 인식 요청 수신!")

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("❌ 카메라 열기 실패")
            response.id = -1
            return response

        detected_ids = []
        frame_counter = 0

        try:
            while frame_counter < 30:
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn("⚠️ 프레임 캡처 실패")
                    break

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

                # ✅ OpenCV 버전에 따라 생성 방식 분기
                if hasattr(aruco, 'DetectorParameters_create'):
                    parameters = aruco.DetectorParameters_create()
                else:
                    parameters = aruco.DetectorParameters()

                corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                if ids is not None:
                    for i in range(len(ids)):
                        detected_ids.append(int(ids[i][0]))

                    # ✅ 디버깅용 (마커 그리기, 주석 해제 시 OpenCV 창 표시 가능)
                    # aruco.drawDetectedMarkers(frame, corners, ids)
                    # cv2.imshow("ArUco Detection", frame)
                    # cv2.waitKey(1)

                frame_counter += 1

        finally:
            cap.release()
            # cv2.destroyAllWindows()  # 위 imshow 쓴 경우만 필요

        if detected_ids:
            most_common_id, count = Counter(detected_ids).most_common(1)[0]
            self.get_logger().info(f"🎯 다수결 마커 ID: {most_common_id} ({count}회)")
            response.id = most_common_id
        else:
            self.get_logger().warn("❌ 마커 감지 실패")
            response.id = -1

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 종료 신호 감지 (Ctrl+C)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

