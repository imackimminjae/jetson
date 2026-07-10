# log_bag.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from datetime import datetime
import os

# 저장 경로 / 토픽 / 지연시간
BASE_DIR = "/home/imac/rosbags"   # 필요 시 /home/imac/rosbags 또는 마운트 경로로 변경
TOPICS = [
    "/mavros/local_map/odom",
    "/vehicle/status_mav",
    '/vehicle/status_hils',
    "/vehicle/status_hrr",
    "/vehicle/status_cte",
    '/fsm_mode',
    '/obstacles',
]
DELAY_SEC = 2.0

def generate_launch_description():
    # 저장 경로 보장
    os.makedirs(BASE_DIR, exist_ok=True)

    bag_prefix = f"log_exp_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    cmd = ["ros2", "bag", "record", "-o", bag_prefix] + TOPICS

    # bag 기록 프로세스
    record_proc = ExecuteProcess(
        cmd=cmd,
        output="screen",
        cwd=BASE_DIR,   # 여기서 Permission 이슈가 나면 경로/권한 점검
    )

    # 지연 후 시작
    delayed = TimerAction(period=DELAY_SEC, actions=[record_proc])

    # LaunchDescription은 엔티티 리스트를 받습니다.
    return LaunchDescription([delayed])
