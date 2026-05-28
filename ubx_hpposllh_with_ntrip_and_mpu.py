"""
高精度GPS測位システム - 超高速版
ZED-F9P + MPU6050 統合（処理速度最優先）
"""

from machine import UART, Pin, I2C
import socket
import network
import time
import struct
import math
import mpu6050

# ==================== ハードウェア初期化 ====================
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
mpu = mpu6050.MPU6050(i2c)

# ==================== 設定 ====================
UART_NUM = 1
UART_BAUDRATE = 921600
UART_TX_PIN = 17
UART_RX_PIN = 18

NTRIP_SERVER = "ntrip1.bizstation.jp"
NTRIP_PORT = 2101
MOUNT_POINT = "B49FE63A"

WIFI_SSID = "lab"
WIFI_PASSWORD = "aaaaaaaa"

# ==================== 定数 ====================
EARTH_RADIUS = 6371000.0
RAD = 0.017453292519943295  # math.pi/180
SPEED_WINDOW = 5
SPEED_LIMIT = 10.0
SPEED_NOISE = 0.8
SPEED_SMOOTH = 0.6

UBX_SYNC1 = 0xB5
UBX_SYNC2 = 0x62
UBX_CLASS_NAV = 0x01
UBX_ID_HPPOSLLH = 0x14

# ==================== グローバル変数（高速アクセス用）====================
uart = None

# 速度用バッファ
speed_buf = [0.0] * SPEED_WINDOW
speed_idx = 0
speed_cnt = 0
prev_speed = 0.0
filtered_speed = 0.0

# 位置用
prev_lat = 0.0
prev_lon = 0.0
prev_time = 0

# MPU用
ax = ay = az = 0.0
mpu_time = 0
MPU_INTERVAL = 20  # 20Hz

# バッファ（事前確保）
line_buf = bytearray(256)
lb_len = 0
in_ubx = False
ubx_class = 0
ubx_id = 0
ubx_len = 0

def calc_distance(lat1, lon1, lat2, lon2):
    """距離計算（インライン展開用に別関数だが最適化）"""
    lat1_r = lat1 * RAD
    lat2_r = lat2 * RAD
    dlat = (lat2 - lat1) * RAD * 0.5
    dlon = (lon2 - lon1) * RAD * 0.5
    
    a = math.sin(dlat) ** 2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon) ** 2
    return EARTH_RADIUS * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def update_mpu():
    """MPU6050更新（タイミング制御）"""
    global ax, ay, az, mpu_time
    t = time.ticks_ms()
    if time.ticks_diff(t, mpu_time) >= MPU_INTERVAL:
        mpu_time = t
        ax, ay, az = mpu.get_accel_data()
        return True
    return False

def calc_speed(lat, lon, tm):
    """速度計算"""
    global prev_lat, prev_lon, prev_time
    if prev_time == 0:
        prev_lat, prev_lon, prev_time = lat, lon, tm
        return 0.0
    
    dt = (tm - prev_time) * 0.001
    if dt <= 0 or dt > 2.0:
        prev_lat, prev_lon, prev_time = lat, lon, tm
        return 0.0
    
    dist = calc_distance(prev_lat, prev_lon, lat, lon)
    prev_lat, prev_lon, prev_time = lat, lon, tm
    return dist / dt * 3.6

def filter_speed(raw):
    """速度フィルタ"""
    global prev_speed, filtered_speed, speed_idx, speed_cnt, speed_buf
    
    if raw < SPEED_NOISE:
        raw = 0.0
    
    if prev_speed > 0:
        diff = raw - prev_speed
        if diff > SPEED_LIMIT:
            raw = prev_speed + SPEED_LIMIT
        elif diff < -SPEED_LIMIT:
            raw = prev_speed - SPEED_LIMIT
    
    speed_buf[speed_idx] = raw
    speed_idx = (speed_idx + 1) % SPEED_WINDOW
    if speed_cnt < SPEED_WINDOW:
        speed_cnt += 1
    
    total = 0.0
    for i in range(speed_cnt):
        total += speed_buf[i]
    avg = total / speed_cnt
    
    if filtered_speed == 0:
        smoothed = avg
    else:
        smoothed = SPEED_SMOOTH * filtered_speed + (1 - SPEED_SMOOTH) * avg
    
    smoothed = round(smoothed, 1) if smoothed >= 1.0 else (0.0 if smoothed < 0.5 else round(smoothed, 1))
    
    filtered_speed = smoothed
    prev_speed = raw
    return smoothed

def parse_ubx(buf, length):
    """UBX解析（高速・メモリアロケーションなし）"""
    if length < 36:
        return None
    
    # フラグチェック
    if buf[3] & 0x01:
        return None
    
    # 直接バッファから読み取り（メモリアロケーション最小化）
    itow = buf[4] | (buf[5] << 8) | (buf[6] << 16) | (buf[7] << 24)
    
    lon_raw = buf[8] | (buf[9] << 8) | (buf[10] << 16) | (buf[11] << 24)
    if lon_raw >= 0x80000000:
        lon_raw -= 0x100000000
    
    lat_raw = buf[12] | (buf[13] << 8) | (buf[14] << 16) | (buf[15] << 24)
    if lat_raw >= 0x80000000:
        lat_raw -= 0x100000000
    
    hMSL_raw = buf[20] | (buf[21] << 8) | (buf[22] << 16) | (buf[23] << 24)
    if hMSL_raw >= 0x80000000:
        hMSL_raw -= 0x100000000
    
    lon_hp = buf[24] if length > 24 else 0
    lat_hp = buf[25] if length > 25 else 0
    hMSL_hp = buf[27] if length > 27 else 0
    
    hAcc = buf[28] | (buf[29] << 8) | (buf[30] << 16) | (buf[31] << 24)
    
    # スケーリング（整数演算）
    lat_s = lat_raw * 100 + lat_hp
    lon_s = lon_raw * 100 + lon_hp
    hgt_s = hMSL_raw * 10 + hMSL_hp
    hacc_cm = hAcc * 0.01  # 0.1mm -> cm
    
    return (itow, lat_s, lon_s, hgt_s, hacc_cm)

def format_coord(v):
    """座標フォーマット（バッファリングなしで直接文字列生成）"""
    negative = v < 0
    if negative:
        v = -v
    deg = v // 1000000000
    frac = v % 1000000000
    return f"{'-' if negative else ''}{deg}.{frac:09d}"

def coord_to_float(v):
    """座標変換"""
    return v / 1000000000.0 if v >= 0 else -((-v) / 1000000000.0)

def init_uart():
    global uart
    if uart:
        return uart
    try:
        uart = UART(UART_NUM, baudrate=UART_BAUDRATE, tx=UART_TX_PIN, rx=UART_RX_PIN)
        uart.init(bits=8, parity=None, stop=1)
        return uart
    except:
        return None

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    for _ in range(30):
        if wlan.isconnected():
            return True
        time.sleep(1)
    return False

def send_ubx_cmd(cls, id, payload=b''):
    if not uart:
        return
    length = len(payload)
    header = bytes([cls, id, length & 0xFF, (length >> 8) & 0xFF])
    ck_a = ck_b = 0
    for b in header + payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    uart.write(bytes([0xB5, 0x62]) + header + payload + bytes([ck_a, ck_b]))

def run():
    global uart, prev_lat, prev_lon, prev_time, prev_speed, filtered_speed, speed_idx, speed_cnt
    global ax, ay, az, mpu_time, line_buf, lb_len, in_ubx, ubx_class, ubx_id, ubx_len
    
    # 変数初期化
    prev_lat = prev_lon = 0.0
    prev_time = 0
    prev_speed = filtered_speed = 0.0
    speed_idx = speed_cnt = 0
    ax = ay = az = 0.0
    mpu_time = 0
    lb_len = 0
    in_ubx = False
    
    print("\nGPS+MPU6050 超高速モード起動中...")
    
    uart = init_uart()
    if not uart:
        print("UARTエラー")
        return
    
    if not connect_wifi():
        print("WiFiエラー")
        return
    
    send_ubx_cmd(0x06, 0x01, bytes([0x01, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00]))
    
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((NTRIP_SERVER, NTRIP_PORT))
        sock.send(f"GET /{MOUNT_POINT} HTTP/1.1\r\nHost: {NTRIP_SERVER}\r\n\r\n".encode())
        sock.settimeout(1)
        for _ in range(20):
            if sock.readline() == b'\r\n':
                break
        sock.setblocking(False)
        
        print("Ready - ログ出力中...")
        print("Count,Latitude,Longitude,Acc_cm,Speed,AccX,AccY,AccZ")
        
        hp_cnt = 0
        rtcm_cnt = 0
        
        while True:
            # MPU更新（非同期）
            update_mpu()
            
            # GPSデータ処理（高速パーサ）
            if uart.any():
                data = uart.read(64)  # 一度に64バイト読み取り
                if data:
                    for b in data:
                        if not in_ubx:
                            if b == 0xB5:
                                line_buf[0] = b
                                lb_len = 1
                            elif b == 0x62 and lb_len == 1 and line_buf[0] == 0xB5:
                                line_buf[1] = b
                                lb_len = 2
                                in_ubx = True
                            else:
                                lb_len = 0
                        else:
                            if lb_len < len(line_buf):
                                line_buf[lb_len] = b
                                lb_len += 1
                            
                            if lb_len == 4:
                                ubx_class = line_buf[2]
                                ubx_id = line_buf[3]
                            elif lb_len == 6:
                                ubx_len = line_buf[4] | (line_buf[5] << 8)
                            elif ubx_len > 0 and lb_len == 6 + ubx_len + 2:
                                if ubx_class == 0x01 and ubx_id == 0x14:
                                    parsed = parse_ubx(line_buf[6:], ubx_len)
                                    if parsed:
                                        hp_cnt += 1
                                        tm, lat_s, lon_s, hgt_s, hacc = parsed
                                        
                                        # 速度計算
                                        lat_f = coord_to_float(lat_s)
                                        lon_f = coord_to_float(lon_s)
                                        raw_spd = calc_speed(lat_f, lon_f, tm)
                                        spd = filter_speed(raw_spd if raw_spd > 0 else 0.0)
                                        
                                        # 出力（最小限の文字列操作）
                                        print(f"{hp_cnt},{format_coord(lat_s)},{format_coord(lon_s)},    {hacc:.1f},   {spd:.1f},   {ax:.2f},{ay:.2f},{az:.2f}")
                                
                                in_ubx = False
                                lb_len = 0
            
            # RTCM転送
            if sock:
                try:
                    b = sock.recv(1)
                    if b and b[0] == 0xD3:
                        lb = sock.recv(2)
                        if len(lb) == 2:
                            msg_len = ((lb[0] & 0x03) << 8) | lb[1]
                            if 0 < msg_len < 3000:
                                d = sock.recv(msg_len + 3)
                                if len(d) >= msg_len + 3:
                                    uart.write(b'\xD3' + lb + d)
                                    rtcm_cnt += 1
                except:
                    pass
            
            # 極短待機（CPU解放）
            time.sleep_us(500)
            
    except KeyboardInterrupt:
        print(f"\n停止 - GPS:{hp_cnt}回, RTCM:{rtcm_cnt}フレーム")
    except Exception as e:
        print(f"エラー:{e}")
    finally:
        if sock:
            sock.close()

# 実行
if __name__ == "__main__":
    run()
