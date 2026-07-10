# image_bag.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from datetime import datetime
import os

# 저장 경로 / 토픽 / 지연시간
BASE_DIR = "/home/imac/rosbags"   # 필요 시 /home/imac/rosbags 또는 마운트 경로로 변경
TOPICS = [
    "/oak/rgb/image_raw",
    "/oak/depth/image_raw",
    "/oak/traffic_light_status",
    "/oak/objects_3d",
    "/red_bboxes",
    "/oak/yolo/bboxes",
]
DELAY_SEC = 2.0

def generate_launch_description():
    # 저장 경로 보장
    os.makedirs(BASE_DIR, exist_ok=True)

    bag_prefix = f"image_exp_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    cmd = ["ros2", "bag", "record", "-o", bag_prefix] + TOPICS

    # 이미지 토픽은 용량이 커서 압축 권장
    cmd += ["--compression-mode", "file", "--compression-format", "zstd"]
    
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
