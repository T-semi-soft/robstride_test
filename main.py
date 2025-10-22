import can
import time
import sys
import threading
from typing import Optional

# --- プロトコル定義 (Arduino版と同一) ---
COMM_GET_ID = 0x00  # モード0: GET_ID
COMM_SET_ID = 0x07  # モード7: SET_ID
RESP_MARKER = 0xFE  # 応答ID下位8bit
MASTER_ID = 0xFD    # 任意の上位機ID（慣例値）

# 最後にスキャンで見つかっ

def main():
    print("Hello from robstride-test!")


if __name__ == "__main__":
    main()
