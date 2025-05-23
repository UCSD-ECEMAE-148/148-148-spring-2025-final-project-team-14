import cv2
import depthai as dai
import yaml
import time
import numpy as np
from adafruit_servokit import ServoKit

# -------- Configuration --------
CONFIG_FILE = 'config.yaml'

def load_config():
    try:
        with open(CONFIG_FILE, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        default = {
            'calibration': 1,
            'hsv_lower1': [0, 100, 100],
            'hsv_upper1': [10, 255, 255],
            'hsv_lower2': [160, 100, 100],
            'hsv_upper2': [179, 255, 255],
            # control params
            'DEADZONE': 30,
            'HYST': 10,            # larger slack
            'Kp': 0.5,             # slightly reduced
            'Ki': 0.0,             # disabled for now
            'Kd': 0.2,             # increased derivative weight
            'XP': 100.0,
            'YP': 100.0,
            'LOOP_DELAY': 0.03,
            'MAX_STEP': 3.0,       # tighter rate limit
            'FILTER_ALPHA': 0.1,   # heavier smoothing on d-term
            'POS_SMOOTH': 0.6,     # centroid EMA weight
            'I_MAX': 100.0
        }
        save_config(default)
        return default

def save_config(cfg):
    with open(CONFIG_FILE, 'w') as f:
        yaml.safe_dump(cfg, f)

config = load_config()

# -------- Servo setup --------
kit = ServoKit(channels=16)
kit.servo[1].set_pulse_width_range(1300, 2400); kit.servo[1].actuation_range = 180
kit.servo[0].set_pulse_width_range(500, 2500);  kit.servo[0].actuation_range = 180

pan_angle  = 90.0
tilt_angle = 90.0
kit.servo[0].angle = pan_angle
kit.servo[1].angle = tilt_angle

# -------- DepthAI pipeline --------
pipeline = dai.Pipeline()
cam = pipeline.createColorCamera()
cam.setBoardSocket(dai.CameraBoardSocket.RGB)
cam.setPreviewSize(640, 480)
cam.setInterleaved(False)
xout = pipeline.createXLinkOut(); xout.setStreamName("rgb")
cam.preview.link(xout.input)

def nothing(x): pass

# Calibration UI (unchanged)...
if config['calibration'] == 1:
    cv2.namedWindow('Calibration')
    # create trackbars...
    # [same as before]
    with dai.Device(pipeline) as device:
        q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        while True:
            frame = q.get().getCvFrame()
            # read & save trackbars...
            if cv2.waitKey(1)&0xFF==ord('q'):
                break
    save_config(config)
    print(f"Config saved to {CONFIG_FILE}")
    cv2.destroyAllWindows()
    exit(0)

# -------- Active tracking --------
DEADZONE     = config['DEADZONE']
HYST         = config['HYST']
Kp           = config['Kp']
Ki           = config['Ki']
Kd           = config['Kd']
XP           = config['XP']
YP           = config['YP']
LOOP_DELAY   = config['LOOP_DELAY']
MAX_STEP     = config['MAX_STEP']
ALPHA        = config['FILTER_ALPHA']
POS_SMOOTH   = config['POS_SMOOTH']
I_MAX        = config['I_MAX']

prev_err_x   = prev_err_y   = 0.0
prev_d_err_x = prev_d_err_y = 0.0
i_err_x      = i_err_y      = 0.0
last_time    = time.time()

# smoothed centroid state
smooth_x = smooth_y = None

with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    while True:
        now = time.time()
        dt = max(now - last_time, 1e-3)
        last_time = now

        frame = q.get().getCvFrame()
        h, w = frame.shape[:2]
        cx, cy = w//2, h//2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, tuple(config['hsv_lower1']), tuple(config['hsv_upper1']))
        mask2 = cv2.inRange(hsv, tuple(config['hsv_lower2']), tuple(config['hsv_upper2']))
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))
        mask = cv2.GaussianBlur(mask, (9,9), 0)

        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            (x, y), r = cv2.minEnclosingCircle(c)
            if r > 10:
                # initialize smoothing on first sight
                if smooth_x is None:
                    smooth_x, smooth_y = x, y
                # exponential moving average of centroid
                smooth_x = POS_SMOOTH*smooth_x + (1-POS_SMOOTH)*x
                smooth_y = POS_SMOOTH*smooth_y + (1-POS_SMOOTH)*y

                err_x = smooth_x - cx
                err_y = smooth_y - cy

                # derivative (normalized + heavy smoothing)
                d_raw_x = (err_x - prev_err_x)/dt
                d_err_x = ALPHA*d_raw_x + (1-ALPHA)*prev_d_err_x
                prev_d_err_x = d_err_x

                d_raw_y = (err_y - prev_err_y)/dt
                d_err_y = ALPHA*d_raw_y + (1-ALPHA)*prev_d_err_y
                prev_d_err_y = d_err_y

                # integral (clamped)
                i_err_x += err_x*dt; i_err_x = np.clip(i_err_x, -I_MAX, I_MAX)
                i_err_y += err_y*dt; i_err_y = np.clip(i_err_y, -I_MAX, I_MAX)

                # PD (no I for now)
                delta_x = Kp*(err_x/XP) + Kd*(d_err_x/XP)
                delta_y = Kp*(err_y/YP) + Kd*(d_err_y/YP)

                # pan
                if abs(err_x) > DEADZONE + HYST:
                    step_x = np.clip(delta_x, -MAX_STEP, MAX_STEP)
                    pan_angle = float(np.clip(pan_angle - step_x, 0, 180))
                    kit.servo[0].angle = pan_angle
                prev_err_x = err_x

                # tilt
                if abs(err_y) > DEADZONE + HYST:
                    step_y = np.clip(delta_y, -MAX_STEP, MAX_STEP)
                    tilt_angle = float(np.clip(tilt_angle - step_y, 0, 180))
                    kit.servo[1].angle = tilt_angle
                prev_err_y = err_y

        # draw feedback
        cv2.circle(frame, (cx,cy), 5, (255,255,255), -1)
        cv2.putText(frame,
                    f"Pan: {pan_angle:.1f}° Tilt: {tilt_angle:.1f}°",
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        cv2.imshow("Pan-Tilt Tracking", frame)
        if cv2.waitKey(1)&0xFF == ord('q'):
            break
        time.sleep(LOOP_DELAY)

    cv2.destroyAllWindows()

