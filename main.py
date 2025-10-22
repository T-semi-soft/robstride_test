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
    global last_found_id
    
    if len(sys.argv) != 2:
        print(f"使い方: python {sys.argv[0]} <can_interface>")
        print(f"例 (SocketCAN): python {sys.argv[0]} can0")
        print(f"例 (SLCAN):     python {sys.argv[0]} /dev/ttyACM0")
        sys.exit(1)
        
    channel_name = sys.argv[1]
    # 'can0' なら 'socketcan', '/dev/ttyACM0' なら 'slcan' などを推測
    interface_type = get_bus_interface(channel_name)

    print("\nRS05 IDツール (python-can版)")
    print("G=スキャン / S <hex>=ID変更 / Q=終了")

    bus: Optional[can.Bus] = None
    try:
        bus = can.Bus(
            interface=interface_type, 
            channel=channel_name, 
            bitrate=1000000
        )
        print(f"CAN start OK ({interface_type} @ {channel_name} @1Mbps)")
    except Exception as e:
        print(f"CANバス '{channel_name}' ({interface_type}) の初期化に失敗しました: {e}")
        if interface_type == 'socketcan':
            print(f"sudo ip link set {channel_name} type can bitrate 1000000 up")
            print("↑ を実行しましたか？")
        sys.exit(1)

    # 起動時に一発スキャン
    scan_ids(bus)

    # loop()
    try:
        while True:
            cmd_line = input("> ").strip()
            if not cmd_line:
                continue
                
            cmd_parts = cmd_line.split()
            cmd_char = cmd_parts[0].upper()

            if cmd_char == 'G':
                scan_ids(bus)
                
            elif cmd_char == 'S':
                if len(cmd_parts) < 2:
                    print("使い方: S <hex> 例) S 3B")
                    continue
                
                hex_str = cmd_parts[1]
                try:
                    new_id = int(hex_str, 16)
                    if not (0 <= new_id <= 0x7F):
                        print("ID範囲外。0x00..0x7Fだけや。")
                        continue
                        
                    if last_found_id == 0xFF:
                        print("現在見えてるIDが無い。先にGでスキャンしろ。")
                    else:
                        print(f"SET_ID: 0x{last_found_id:02X} -> 0x{new_id:02X} ...")
                        if send_set_id(bus, last_found_id, new_id):
                            time.sleep(0.05)  # delay(50)
                            # 変更直後にブロードキャスト応答が来る仕様。ついでに再スキャンで確定させる。
                            scan_ids(bus)
                        else:
                            print("送信失敗")
                            
                except ValueError:
                    print(f"16進数として無効です: '{hex_str}'")

            elif cmd_char == 'Q':
                break
            
            else:
                print("コマンド: G / S <hex> / Q")

    except KeyboardInterrupt:
        print("\n終了 (Ctrl+C)")
    finally:
        if bus:
            bus.shutdown()
            print("CANバスをシャットダウンしました。")

if __name__ == "__main__":
    main()
