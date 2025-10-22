#!/usr/bin/env python3
#
# RS05 角度指令(CSP)ツール - python-can版
#
# ・G         : 接続中モータのCAN IDスキャン（current_idを更新）
# ・L <rad_s> : CSPの速度上限 [rad/s] を設定（保持）
# ・A <deg>   : 目標角度 [deg] 指定（内部でCSPを有効化→上限適用→loc_ref書き込み）
# ・Q         : 終了
#
# -----------------------------------------------------------------
# Pythonでの事前準備 (ターミナルで実行):
# 1. 必要なライブラリのインストール:
#    $ pip install python-can
#
# 2. (SocketCANの場合) CANインターフェースを1Mbpsで起動:
#    $ sudo ip link set can0 down
#    $ sudo ip link set can0 type can bitrate 1000000
#    $ sudo ip link set can0 up
#
# 3. 実行 (インターフェース名を引数に指定):
#    $ python rs05_csp_tool.py can0
# -----------------------------------------------------------------

import can
import time
import sys
import struct  # float_to_le のため
import math
from typing import Optional

# ==== 拡張ID(29bit)レイアウト ====
# [28:24]=モード(5bit), [23:8]=data16, [7:0]=id8
def build_ext_id(mode: int, data16: int, id8: int) -> int:
    """RS05の拡張IDを構築する"""
    return ((mode & 0x1F) << 24) | (data16 << 8) | id8

# ==== モード / インデックス ====
COMM_GET_ID    = 0x00  # モード0: ID取得
COMM_RUN_EN    = 0x03  # モード3: 有効化
COMM_WRITE_ONE = 0x12  # モード18: 単一書込み
RESP_MARKER    = 0xFE
MASTER_ID      = 0xFD

IDX_runmode    = 0x7005  # 0:運控 1:PP 2:速度 3:電流 5:CSP
IDX_loc_ref    = 0x7016  # 位置指令 [rad] (float)
IDX_limit_spd  = 0x7017  # 速度上限 [rad/s] (float, CSP用)

# ==== 状態 ====
current_id: int = 0x7F   # スキャンで上書き
limit_spd: float = 10.0  # 既定のCSP速度上限[rad/s]

# ==== ユーティリティ ====

def can_send(bus: can.Bus, msg: can.Message) -> bool:
    """CAN送信ラッパー"""
    try:
        bus.send(msg)
        return True
    except can.CanError as e:
        print(f"CAN送信エラー: {e}")
        return False

def float_to_le_bytes(val: float) -> bytes:
    """floatを4バイトのリトルエンディアンbytesに変換"""
    # C++: float_to_le(val, &tx.data[4])
    return struct.pack('<f', val)

# ---- 内部送信ヘルパ ----

def send_enable(bus: can.Bus, id: int) -> bool:
    """「有効化」送信 (モード3)"""
    arbitration_id = build_ext_id(COMM_RUN_EN, (MASTER_ID << 8) | 0x00, id)
    msg = can.Message(
        arbitration_id=arbitration_id,
        is_extended_id=True,
        dlc=8,
        data=[0] * 8
    )
    return can_send(bus, msg)

def write_param_f(bus: can.Bus, index: int, val: float, id: int) -> bool:
    """float値を単一書き込み (モード18)"""
    arbitration_id = build_ext_id(COMM_WRITE_ONE, (MASTER_ID << 8) | 0x00, id)
    
    # ペイロード: [idx_lo, idx_hi, 0, 0, f0, f1, f2, f3]
    payload = bytearray(8)
    payload[0] = index & 0xFF
    payload[1] = (index >> 8) & 0xFF
    # payload[2, 3] = 0
    payload[4:8] = float_to_le_bytes(val) # 4バイトを代入
    
    msg = can.Message(
        arbitration_id=arbitration_id,
        is_extended_id=True,
        dlc=8,
        data=payload
    )
    return can_send(bus, msg)

def write_param_u8(bus: can.Bus, index: int, val: int, id: int) -> bool:
    """uint8_t値を単一書き込み (モード18)"""
    arbitration_id = build_ext_id(COMM_WRITE_ONE, (MASTER_ID << 8) | 0x00, id)
    
    # ペイロード: [idx_lo, idx_hi, 0, 0, val, 0, 0, 0]
    payload = bytearray(8)
    payload[0] = index & 0xFF
    payload[1] = (index >> 8) & 0xFF
    # payload[2, 3] = 0
    payload[4] = val & 0xFF
    # payload[5, 6, 7] = 0
    
    msg = can.Message(
        arbitration_id=arbitration_id,
        is_extended_id=True,
        dlc=8,
        data=payload
    )
    return can_send(bus, msg)

# ---- IDスキャン（G） ----
def scan_ids(bus: can.Bus, wait_ms: int = 800):
    global current_id
    
    print("全IDへGET_ID...")
    for id_to_scan in range(128): # 0..127
        arbitration_id = build_ext_id(
            COMM_GET_ID, 
            (MASTER_ID << 8) | 0x00, 
            id_to_scan
        )
        msg = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=True,
            dlc=8,
            data=[0] * 8
        )
        can_send(bus, msg)
        time.sleep(0.0007) # delayMicroseconds(700)

    t0 = time.monotonic()
    wait_sec = wait_ms / 1000.0
    found_this_scan = False

    while (time.monotonic() - t0) < wait_sec:
        # C++版の pdMS_TO_TICKS(10) に相当
        rx_msg = bus.recv(timeout=0.01) 
        
        if rx_msg is None:
            continue
        
        if not rx_msg.is_extended_id or rx_msg.dlc != 8:
            continue
            
        extid = rx_msg.arbitration_id
        mode = (extid >> 24) & 0x1F
        data16 = (extid >> 8) & 0xFFFF
        id8 = extid & 0xFF

        if mode == COMM_GET_ID and id8 == RESP_MARKER:
            id_found = data16 & 0xFF
            current_id = id_found # グローバル変数を更新
            found_this_scan = True
            print(f"[FOUND] CAN_ID=0x{id_found:02X}")
            # 複数のモータが応答する可能性もあるが、
            # C++版と同様に最後に見つかったものを採用
            
    if not found_this_scan:
        print("応答なし")
        
    print(f"current_id=0x{current_id:02X}")


# ---- 角度指令（A） ----
def csp_move_deg(bus: can.Bus, deg: float) -> bool:
    global limit_spd, current_id
    
    rad = deg * math.pi / 180.0
    ok = True
    
    # 1. CSP(5)モードに設定
    ok &= write_param_u8(bus, IDX_runmode, 5, current_id)
    # 2. 速度上限を適用
    ok &= write_param_f(bus, IDX_limit_spd, limit_spd, current_id)
    time.sleep(0.005) # delay(5)
    
    # 3. 有効化
    ok &= send_enable(bus, current_id)
    time.sleep(0.005) # delay(5)
    
    # 4. 目標角度を書き込み
    ok &= write_param_f(bus, IDX_loc_ref, rad, current_id)
    
    status = "OK" if ok else "NG"
    print(f"A: {deg:.3f} deg ({rad:.6f} rad) / limit={limit_spd:.3f} rad/s -> {status}")
    return ok

# ==== インターフェース判別 ====
def get_bus_interface(channel: str) -> str:
    """インターフェース名から 'interface' タイプを推測する"""
    if 'tty' in channel or 'COM' in channel:
        return 'slcan' # 例: /dev/ttyACM0
    return 'socketcan' # 例: can0

# ==== メイン ====
def main():
    global limit_spd, current_id # Lコマンドで変更するため
    
    if len(sys.argv) != 2:
        print(f"使い方: python {sys.argv[0]} <can_interface>")
        print(f"例 (SocketCAN): python {sys.argv[0]} can0")
        print(f"例 (SLCAN):     python {sys.argv[0]} /dev/ttyACM0")
        sys.exit(1)
        
    channel_name = sys.argv[1]
    interface_type = get_bus_interface(channel_name)

    print("\nRS05 角度指令(CSP): G=スキャン / L <rad_s>=上限設定 / A <deg>=角度指令 / Q=終了")

    bus: Optional[can.Bus] = None
    try:
        bus = can.Bus(
            interface=interface_type, 
            channel=channel_name, 
            bitrate=1000000  # 1Mbps
        )
        print(f"CAN start OK ({interface_type} @ {channel_name} @1Mbps), current_id=0x{current_id:02X}")
    except Exception as e:
        print(f"CANバス '{channel_name}' ({interface_type}) の初期化に失敗: {e}")
        if interface_type == 'socketcan':
            print(f"sudo ip link set {channel_name} type can bitrate 1000000 up")
            print("↑ を実行しましたか？")
        sys.exit(1)

    # メインループ (C++版の loop() 相当)
    try:
        while True:
            cmd_line = input("> ").strip()
            if not cmd_line:
                continue
                
            cmd_parts = cmd_line.split()
            cmd_char = cmd_parts[0].upper()

            if cmd_char == 'G':
                scan_ids(bus)
                
            elif cmd_char == 'L':
                if len(cmd_parts) < 2:
                    print(f"CSP速度上限(現在): {limit_spd:.3f} rad/s")
                    print("使い方: L <rad_s>")
                    continue
                try:
                    v = float(cmd_parts[1])
                    if v < 0: v = 0.0
                    
                    # モータに即時適用
                    ok = write_param_f(bus, IDX_limit_spd, v, current_id)
                    if ok:
                        limit_spd = v # 成功したらローカルの既定値も更新
                    
                    status = "OK" if ok else "NG"
                    print(f"CSP速度上限: {v:.3f} rad/s -> {status}")

                except ValueError:
                    print(f"無効な数値です: '{cmd_parts[1]}'")

            elif cmd_char == 'A':
                if len(cmd_parts) < 2:
                    print("使い方: A <deg>")
                    continue
                try:
                    deg = float(cmd_parts[1])
                    csp_move_deg(bus, deg)
                except ValueError:
                    print(f"無効な数値です: '{cmd_parts[1]}'")

            elif cmd_char == 'Q':
                print("終了します。")
                break
            
            else:
                print("コマンド: G / L <rad_s> / A <deg> / Q")

    except KeyboardInterrupt:
        print("\n終了 (Ctrl+C)")
    finally:
        if bus:
            bus.shutdown()
            print("CANバスをシャットダウンしました。")

if __name__ == "__main__":
    main()
    