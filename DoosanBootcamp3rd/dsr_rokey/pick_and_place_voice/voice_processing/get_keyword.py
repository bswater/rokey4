# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"
import os
import asyncio
import rclpy
import pyaudio
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

import edge_tts
from playsound import playsound

############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("pick_and_place_voice")

is_load = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")


############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):
        super().__init__("get_keyword_node")

        # OpenAI LLM 초기화
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        # 도구 및 행동 추출용 프롬프트
        self.keyword_prompt = PromptTemplate(
            input_variables=["user_input"],
            template="""
             당신은 사용자의 문장에서 도구와 해당 도구에 대한 작업을 추출해야 합니다.
            <목표>
            - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
            - 도구에 대해 수행할 작업(clean, trash, eat 중 하나)을 문맥에 따라 추론하여 반드시 함께 출력하세요.
            <도구 리스트>
            - can, fruits, milk, snacks
            <행동 리스트>
            - clean, trash, eat
            <출력 형식>
            - 다음 형식을 반드시 따르세요: 도구1 도구2 ... / 행동
            - 도구는 공백으로 구분하며, 작업은 마지막에 `/` 뒤에 1개만 포함되어야 합니다.
            - 줄바꿈, 쉼표, 따옴표, 괄호 등 불필요한 문자는 절대 포함하지 마세요.
            - 반드시 한 줄로 출력하세요.
            - 사용자가 특정 수량을 지정하지 않은 경우, 각 도구는 기본적으로 2개씩 있다고 간주하고 이름을 두 번 반복해 출력하세요.
            <도구 유사 표현>
            다음과 같은 유사 표현이 등장할 경우 도구 리스트 항목으로 매핑하여 추출하세요.
            - can: 음료수, 캔음료, 파워에이드, 콜라, 사이다, 펩시, 몬스터, 음료
            - fruits: 과일, 오렌지, 사과, 바나나, 포도, 디저트, 후식
            - milk: 우유, 흰 우유, 딸기우유, 초코우유
            - snacks: 과자, 빼빼로, 스낵, 군것질
            <카테고리 표현 대응>
            다음과 같은 상위 개념이 등장하면 아래 항목들로 변환하여 추출하세요.
            - 디저트, 과일, 후식 → fruits
            - 간식, 군것질, 먹을 것, 씹을 거리 → snacks
            - 음료, 마실 것 → can
            <포괄 명령 처리>
            "다 꺼내", "전부", "모두", "있는 거 다", "다 가져와" 등의 표현이 포함되면 도구 리스트 전체를 등장 순서대로 출력하세요.
            <수량 제한 표현 처리>
            "하나", "한 개", "한 종류", "하나만", "조금만" 등의 표현이 포함된 경우,
            해당 카테고리 또는 도구군에서 등장 순서 기준으로 하나만 선택하여 출력하세요.
            <도구 제외 표현 처리>
            "~ 빼고", "~ 제외하고"가 포함될 경우, 해당 항목은 도구 리스트에서 제외하여 출력하세요.
            예: "음료수 빼고 정리해줘" → fruits fruits milk milk snacks snacks
            <행동 추출 규칙>
            문장에서 다음 표현을 기반으로 clean, trash, eat 중 하나를 반드시 추출하세요.
            - clean: 정리해줘, 치워줘, 넣어줘, 정돈해, 청소해
            - trash: 유통기한, 버려줘, 폐기, 상한, 버림
            - eat: 배고파, 먹고 싶어, 후식, 마실 것, 건네줘, 줘, 꺼내와, 챙겨줘, 가져와
            ※ 여러 행동 표현이 혼합되어 있을 경우 다음 우선순위를 따르세요: clean > eat > trash
            ※ 예외적으로 수량 제한이 있을 경우(예: "하나만 줘")는 해당 도구를 한 번만 출력하세요.
            <도구가 없을 경우>
            - 도구 리스트 중 해당 항목이 없으면 출력하지 마세요.
            <예시>
            - 입력: "빼빼로를 집어서 옮겨줘"
            출력: snacks snacks / clean
            - 입력: "파워에이드랑 우유를 옮겨줘"
            출력: can can milk milk / eat
            - 입력: "왼쪽에 있는 오렌지를 줘"
            출력: fruits / eat
            - 입력: "매운데 우유를 마시게 건네줘"
            출력: milk / eat
            - 입력: "과자를 주고 같이 마실 음료수도 집어줘"
            출력: snacks can / eat
            - 입력: "거기 있는 새우깡이랑 사과 하나 챙겨서 가져와"
            출력: snacks fruits / eat
            - 입력: "우유나 뭐 그런 거 하나 건네줘"
            출력: milk / eat
            - 입력: "디저트 꺼내와"
            출력: fruits / eat
            - 입력: "마실 거 하나 줘"
            출력: can / eat
            - 입력: "있는 거 다 꺼내줘"
            출력: can fruits milk snacks / eat
            - 입력: "정리해줘"
            출력: can can fruits fruits milk milk snacks snacks / clean
            - 입력: "유통기한 지난 거 버려줘"
            출력: can fruits milk snacks / trash
            - 입력: "후식 먹고 싶어"
            출력: fruits / eat
            - 입력: "음료수 빼고 정리해줘"
            출력: fruits fruits milk milk snacks snacks / clean
            <사용자 입력>
            "{user_input}"
        """
        )
        self.lang_chain = LLMChain(llm=self.llm, prompt=self.keyword_prompt)

        # 반응 생성용 프롬프트
        self.reaction_prompt_template = PromptTemplate(
            input_variables=["user_input", "tools", "action"],
            template="""
            사용자가 "{user_input}"라고 말했고,
            도구는 "{tools}", 행동은 "{action}" 입니다.

            이에 대한 친근하고 자연스러운 한국어 한 문장을 생성하세요.
            예: "사과 하나만 줘" → "사과는 없어요."

            예: "물건들 정리해 줘" → "알겠습니다."

            예: "과자 하나만 줘" → "네 과자 하나 준비할께요"
            
            예: "우유 하나만 줘" → "네 우유 하나 준비할께요"
            """
        )
        self.reaction_chain = LLMChain(llm=self.llm, prompt=self.reaction_prompt_template)

        # STT 및 MIC 설정
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        self.stt = STT(openai_api_key=openai_api_key)
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        # ROS 서비스 등록
        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )

    def extract_keyword(self, output_message):
        # 도구 및 행동 추출
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response["text"].strip()
        print(f"raw llm response: {result}")

        if "/" in result:
            tools_str, action = result.split(" /")
            tools = tools_str.strip().split()
        else:
            tools = []
            action = "unknown"

        # 반응 문장 생성
        reaction = self.reaction_chain.invoke({
            "user_input": output_message,
            "tools": " ".join(tools),
            "action": action
        })["text"].strip()

        print(f"llm's response: tools={tools}, action={action}")
        print(f"reaction: {reaction}")
        return tools, action, reaction

    async def speak_async(self, text, voice="ko-KR-SunHiNeural"):
        file_path = "/tmp/output.mp3"
        communicate = edge_tts.Communicate(text=text, voice=voice)
        await communicate.save(file_path)
        playsound(file_path)
        os.remove(file_path)

    def speak(self, text):
        asyncio.run(self.speak_async(text))

    def get_keyword(self, request, response):
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            response.success = False
            response.message = "오디오 스트림 열기 실패"
            return response

        # 웨이크업 대기
        while not self.wakeup_word.is_wakeup():
            pass

        # STT → Keyword Extraction → TTS
        output_message = self.stt.speech2text()
        tools, action, reaction = self.extract_keyword(output_message)

        self.get_logger().warn(f"Detected tools: {tools}")
        self.get_logger().warn(f"Detected action: {action}")
        self.get_logger().warn(f"Response message: {reaction}")

        # TTS 실행
        self.speak(reaction)

        response.success = True
        response.message = f"{' '.join(tools)} / {action}"
        return response


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
