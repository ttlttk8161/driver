# error_manager.py
# 에러 관리 모듈

import logging

class ErrorCode:
    UNKNOWN = 0
    ROS_TOPIC_TIMEOUT = 1
    ROS_NODE_INIT_FAIL = 2
    MAIN_SYSTEM_INIT_FAIL = 3
    MODULE_START_FAIL = 4
    MODULE_RUNTIME_EXCEPTION = 5
    STDOUT_REDIRECT_FAIL = 6
    # 필요시 추가

class ErrorManager:
    def __init__(self):
        self.logger = logging.getLogger("ErrorManager")

    def handle(self, code, detail=None):
        msg = self._get_message(code, detail)
        self.logger.error(msg)
        # 에러코드별 추가 동작(예: 알림, 종료 등) 구현 가능
        if code == ErrorCode.ROS_TOPIC_TIMEOUT:
            # 예: ROS 토픽 대기 실패 시
            pass
        elif code == ErrorCode.ROS_NODE_INIT_FAIL:
            pass
        elif code == ErrorCode.MAIN_SYSTEM_INIT_FAIL:
            pass
        elif code == ErrorCode.MODULE_START_FAIL:
            pass
        elif code == ErrorCode.MODULE_RUNTIME_EXCEPTION:
            pass
        elif code == ErrorCode.STDOUT_REDIRECT_FAIL:
            pass
        # ...

    def _get_message(self, code, detail):
        base = {
            ErrorCode.UNKNOWN: "알 수 없는 에러 발생.",
            ErrorCode.ROS_TOPIC_TIMEOUT: "ROS 토픽 대기 시간 초과.",
            ErrorCode.ROS_NODE_INIT_FAIL: "ROS 노드 초기화 실패.",
            ErrorCode.MAIN_SYSTEM_INIT_FAIL: "MainSystem 초기화 실패.",
            ErrorCode.MODULE_START_FAIL: "모듈 시작 실패.",
            ErrorCode.MODULE_RUNTIME_EXCEPTION: "모듈 실행 중 예외 발생.",
            ErrorCode.STDOUT_REDIRECT_FAIL: "stdout 리디렉션 실패.",
        }.get(code, "정의되지 않은 에러코드.")
        if detail:
            return f"[Error {code}] {base} 상세: {detail}"
        return f"[Error {code}] {base}"

error_manager = ErrorManager()
