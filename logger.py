import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import cv2
import serial
import csv
import time
import math
from datetime import datetime
from collections import deque

# ==========================================
# 1. System Configuration
# ==========================================
COM_PORT = 'COM3'      
BAUD_RATE = 115200     

IDX_CAMERA = 1         
BASE_NAME = 'single_cam'

# ==========================================
# 2. Device Connection
# ==========================================
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    ser.setDTR(False)
    ser.setRTS(False)
    time.sleep(1) 
    ser.reset_input_buffer()
    print(f"Connected to ESP32 at {BAUD_RATE}")
except Exception as e:
    print(f"Error connecting to Serial Port: {e}")
    exit()

cap = cv2.VideoCapture(IDX_CAMERA, cv2.CAP_MSMF)
if not cap.isOpened():
    print(f"Warning: Camera {IDX_CAMERA} not found, falling back to 0")
    IDX_CAMERA = 0
    cap = cv2.VideoCapture(IDX_CAMERA, cv2.CAP_MSMF)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
REQUESTED_FPS = 20.0
cap.set(cv2.CAP_PROP_FPS, REQUESTED_FPS)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_FOCUS, 0)

reported_fps = cap.get(cv2.CAP_PROP_FPS)
FPS = reported_fps if 1.0 <= reported_fps <= 120.0 else REQUESTED_FPS
fourcc = cv2.VideoWriter_fourcc(*'mp4v')

# ==========================================
# 3. Directory Setup & Variables
# ==========================================
survey_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
survey_folder = f"Survey_{survey_time_str}"
img_folder = os.path.join(survey_folder, "images")
vid_folder = os.path.join(survey_folder, "videos")

os.makedirs(survey_folder, exist_ok=True)
os.makedirs(img_folder, exist_ok=True)
os.makedirs(vid_folder, exist_ok=True)

csv_file = os.path.join(survey_folder, f"{BASE_NAME}_{survey_time_str}.csv")
vid_filename = os.path.join(vid_folder, f"{BASE_NAME}_{survey_time_str}.mp4")

# ตัวแปรสำหรับการตรวจจับการสั่นสะเทือนและการบันทึกเหตุการณ์
VIB_THRESHOLD = 0.7      
PRE_RECORD_SECONDS = 3.0         
TARGET_PHOTO_DISTANCE = 5.0      
TARGET_POST_RECORD_DISTANCE = 15.0
MIN_POST_RECORD_TIME = 2.0       

# ตัวแปรสำหรับจัดกลุ่ม Event
EVENT_COOLDOWN = 1.5     # ระยะเวลา 1.5 วินาที ที่จะนับรวบยอดการสั่นติดๆ กันให้เป็น Event เดียว
last_event_time = 0.0    
current_event_img = "-"  

pre_record_buffer = deque()
is_recording = False
record_end_time = 0.0
pending_photos = deque()
event_counter = 0

prev_x, prev_y, prev_z = None, None, None 
current_vib_val = 0.0
curr_x, curr_y, curr_z = 0.0, 0.0, 0.0
curr_lat = "0.000000"
curr_lon = "0.000000"
curr_speed_kmh = 0.0

out_vid = cv2.VideoWriter(vid_filename, fourcc, FPS, (640, 480))

print(f"System Ready - Press 'q' to stop.")
print(f"Survey Directory: {survey_folder}")

# ==========================================
# 4. Main Loop
# ==========================================
with open(csv_file, 'w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    writer.writerow(["Timestamp", "Latitude", "Longitude", "Acc_cm", "Speed", "Vibration", "AccX", "AccY", "AccZ", "Event_ID", "Image_File"])
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.resize(frame, (640, 480))
            
        triggered = False
        trigger_vib = 0.0

        current_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        current_time = time.monotonic()

        while ser.in_waiting > 0:
            try:
                raw_line = ser.readline()
                line = raw_line.decode('utf-8', errors='ignore').strip()
                
                if line and ',' in line and not "Count" in line and not "Ready" in line:
                    data = [x.strip() for x in line.split(',')]
                    
                    if len(data) >= 8:
                        row_img_name = "-"
                        row_event_id = "-"
                        curr_lat = data[1]
                        curr_lon = data[2]
                        curr_speed_kmh = float(data[4])
                        curr_x = float(data[-3])
                        curr_y = float(data[-2])
                        curr_z = float(data[-1])
                        
                        vib = 0.0
                        if prev_x is not None:
                            diff_x = curr_x - prev_x
                            diff_y = curr_y - prev_y
                            diff_z = curr_z - prev_z
                            vib = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
                        
                        prev_x, prev_y, prev_z = curr_x, curr_y, curr_z
                        current_vib_val = vib
                        
                        # ถือว่าเป็นการสั่นใน Event เดียวกันหากเกิดใกล้เคียงกัน
                        if vib >= VIB_THRESHOLD:
                            triggered = True
                            if vib > trigger_vib:
                                trigger_vib = vib

                            # ถ้าระยะเวลาห่างจากรอยสั่นเดิม "เกิน 1.5 วินาที" จะถือว่าเป็น "หลุมใหม่" (New Event)
                            if current_time - last_event_time > EVENT_COOLDOWN:
                                event_counter += 1
                                file_ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                                current_event_img = f"E{event_counter:05d}_{file_ts}_{curr_lat}_{curr_lon}_{vib:.2f}G.jpg"

                                # จัดคิวถ่ายรูปแค่ 1 ใบ ต่อ 1 เหตุการณ์
                                speed_ms = max(curr_speed_kmh, 5.0) * (1000.0 / 3600.0)
                                dynamic_photo_delay = TARGET_PHOTO_DISTANCE / speed_ms
                                pending_photos.append({
                                    "event_id": event_counter,
                                    "target_time": current_time + dynamic_photo_delay,
                                    "filepath": os.path.join(img_folder, current_event_img),
                                    "image_name": current_event_img,
                                })
                            
                            # อัปเดตเวลาล่าสุดที่มีการสั่น (ยืดเวลา Event ออกไปอีก)
                            last_event_time = current_time
                            
                            row_event_id = event_counter
                            row_img_name = current_event_img

                        writer.writerow([current_timestamp, curr_lat, curr_lon, data[3], data[4], round(vib, 4), curr_x, curr_y, curr_z, row_event_id, row_img_name])
                        f.flush()
            except:
                continue

        color = (0, 0, 255) if current_vib_val >= VIB_THRESHOLD else (0, 255, 0)
        cv2.putText(frame, f"Time: {current_timestamp}", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"GPS: {curr_lat}, {curr_lon}", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Vib: {current_vib_val:.3f} G", (15, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(frame, f"X:{curr_x:.2f} Y:{curr_y:.2f} Z:{curr_z:.2f}", (15, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        if is_recording:
            cv2.circle(frame, (600, 30), 10, (0, 0, 255), -1)

        pre_record_buffer.append((current_time, frame.copy()))
        while pre_record_buffer and current_time - pre_record_buffer[0][0] > PRE_RECORD_SECONDS:
            pre_record_buffer.popleft()

        # ==========================================
        # 5. Event Recording System
        # ==========================================
        if triggered:
            speed_ms = max(curr_speed_kmh, 5.0) * (1000.0 / 3600.0)
            dynamic_post_time = max(TARGET_POST_RECORD_DISTANCE / speed_ms, MIN_POST_RECORD_TIME)
            record_end_time = current_time + dynamic_post_time
            
            if not is_recording:
                is_recording = True
                
                for _, buffered_frame in pre_record_buffer:
                    out_vid.write(buffered_frame)
                
                print(f"Trigger Detected: {trigger_vib:.2f}G. Resuming video recording.")

        if is_recording:
            out_vid.write(frame)

            while pending_photos and current_time >= pending_photos[0]["target_time"]:
                photo = pending_photos.popleft()
                cv2.imwrite(photo["filepath"], frame)
                print(f"Photo saved: {photo['filepath']}")
            
            if current_time >= record_end_time:
                is_recording = False
                print("Event passed. Pausing video recording.")

        cv2.imshow("RoadSense - Monitor", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
out_vid.release()
ser.close()
cv2.destroyAllWindows()
print("Survey completed!")
print(f"Data saved to: {csv_file}")
print(f"Images saved in: {os.path.abspath(img_folder)}")
print(f"Video saved as: {vid_filename}")