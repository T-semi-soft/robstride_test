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

# 最後にスキャンで見つかったID (グローバル変数)
last_found_id = 0xFF
def build_ext_id(mode: int, data16: int, id8: int) -> int:
    """RS05の拡張IDを構築する"""
    return ((mode & 0x1F) << 24) | (data16 << 8) | id8

def send_get_id(bus: can.Bus, target_can_id: int) -> bool:
    """GET_ID (タイプ0) フレームを送信"""
    data16 = (MASTER_ID << 8) | 0x00
    arbitration_id = build_ext_id(COMM_GET_ID, data16, target_can_id)
    
    msg = can.Message(
        arbitration_id=arbitration_id,
        is_extended_id=True,
        dlc=8,
        data=[0] * 8
    )
    
    try:
        bus.send(msg)
        return True
    except can.CanError as e:
        print(f"GET_ID 送信失敗: {e}")
        return False

def send_set_id(bus: can.Bus, current_id: int, new_id: int) -> bool:
    """SET_ID (タイプ7) フレームを送信"""
    # data16: [23..16]=new_id, [15..8]=MASTER_ID
    data16 = (new_id << 8) | MASTER_ID
    arbitration_id = build_ext_id(COMM_SET_ID, data16, current_id)
    
    msg = can.Message(
        arbitration_id=arbitration_id,
        is_extended_id=True,
        dlc=8,
        data=[0] * 8
    )
    
    try:
        bus.send(msg)
        return True
    except can.CanError as e:
        print(f"SET_ID 送信失敗: {e}")
        return False

def scan_ids(bus: can.Bus, wait_ms: int = 1500):
    """IDスキャンを実行"""
    global last_found_id # グローバル変数を変更する宣言
    
    print("全IDにGET_ID(タイプ0)を送信...")
    for i in range(128):  # 0 から 127 まで
        send_get_id(bus, i)
        time.sleep(0.0002)  # delayMicroseconds(200)

    print("応答待ち...")
    t0 = time.monotonic()
    last_found_id = 0xFF
    
    # 1500ms / 10ms = 150回
    # C++版の10msポーリングに相当
    total_wait_sec = wait_ms / 1000.0
    
    while (time.monotonic() - t0) < total_wait_sec:
        # recv()のタイムアウトは応答待ちループの粒度
        # Arduino版の pdMS_TO_TICKS(10) に相当
        rx_msg = bus.recv(timeout=0.01) 
        
        if rx_msg is None:
            continue  # 10msタイムアウト、まだ応答待ち時間内

        if not rx_msg.is_extended_id:
            continue
        if rx_msg.dlc != 8:
            continue
            
        extid = rx_msg.arbitration_id
        mode = (extid >> 24) & 0x1F
        data16 = (extid >> 8) & 0xFFFF
        id8 = extid & 0xFF

        if mode == COMM_GET_ID and id8 == RESP_MARKER:
            motor_can_id = data16 & 0x00FF  # 応答中位16bitの下位8bit=CAN ID
            last_found_id = motor_can_id
            
            # f-stringで16進数フォーマット
            print(f"[RX] EXTID=0x{extid:08X} mode={mode} data16=0x{data16:04X} "
                  f"id8=0x{id8:02X} => CAN_ID=0x{motor_can_id:02X}")

    if last_found_id == 0xFF:
        print("応答なし")

def get_bus_interface(channel: str) -> str:
    """インターフェース名から 'interface' タイプを推測する"""
    if 'tty' in channel or 'COM' in channel:
        #例: /dev/ttyACM0
        return 'slcan'
    if 'can' in channel:
        #例: can0
        return 'socketcan'
    # 
    return 'socketcan'

def main():
    print("Hello from robstride-test!")


if __name__ == "__main__":
    main()
