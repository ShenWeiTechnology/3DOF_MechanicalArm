import sensor, image, time
from pyb import UART
mode = 0
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_hmirror(True)
sensor.set_vflip(True)
clock = time.clock()
uart = UART(1, 9600, timeout=10)
uart.init(9600, bits=8, parity=None, stop=1)
cmd_buf = []
red_threshold = (4, 95, 24, 200, 7, 45)
REAL_WIDTH = 33
focal_length = 272
BLACK_THRESHOLD = (3, 36, -15, 5, -7, 13)
WHITE_THRESHOLD = (50, 70, -15, 5, -7, 17)
FRAME_REAL_SIZE = 100
FRAME_F = 272
MIN_RECT_W = 10
MIN_RECT_H = 10
MAX_RECT_W = 319
MAX_RECT_H = 239
MAX_DIAGONAL_ERROR = 30
MIN_ASPECT_RATIO = 0.4
MAX_ASPECT_RATIO = 2.5
MIN_INNER_WHITE_RATE = 15
ROI_X = 40
ROI_Y = 40
ROI_W = 240
ROI_H = 180
ROI_X2 = ROI_X + ROI_W
ROI_Y2 = ROI_Y + ROI_H
MIN_ASPECT_RATIO_STRICT = 0.8
MAX_ASPECT_RATIO_STRICT = 1.2
class Kalman1D:
    def __init__(self, q=0.1, r=0.1, initial_value=0):
        self.q = q
        self.r = r
        self.x = initial_value
        self.p = 1.0
        self.k = 0.0
    def update(self, measurement):
        self.p = self.p + self.q
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (1 - self.k) * self.p
        return self.x
kf_ball_x = Kalman1D(q=0.5, r=4.0, initial_value=0)
kf_ball_y = Kalman1D(q=0.5, r=4.0, initial_value=0)
kf_ball_dis = Kalman1D(q=1.0, r=10.0, initial_value=0)
kf_p1x = Kalman1D(q=0.5, r=3)
kf_p1y = Kalman1D(q=0.5, r=3)
kf_p2x = Kalman1D(q=0.5, r=3)
kf_p2y = Kalman1D(q=0.5, r=3)
kf_p3x = Kalman1D(q=0.5, r=3)
kf_p3y = Kalman1D(q=0.5, r=3)
kf_p4x = Kalman1D(q=0.5, r=3)
kf_p4y = Kalman1D(q=0.5, r=3)
kf_dis = Kalman1D(q=1.0, r=8.0)
def send_3_data(n1, n2, n3):
    n1 = max(-32768, min(32767, n1))
    n2 = max(-32768, min(32767, n2))
    n3 = max(-32768, min(32767, n3))
    h1 = (n1 >> 8) & 0xFF
    l1 = n1 & 0xFF
    h2 = (n2 >> 8) & 0xFF
    l2 = n2 & 0xFF
    h3 = (n3 >> 8) & 0xFF
    l3 = n3 & 0xFF
    uart.write(bytearray([0xAA, h1, l1, h2, l2, h3, l3, 0xFF]))
def send_4point(x1,y1,d1, x2,y2,d2, x3,y3,d3, x4,y4,d4):
    buf = [0xAA]
    arr = [x1,y1,d1, x2,y2,d2, x3,y3,d3, x4,y4,d4]
    for n in arr:
        n = max(-32768, min(32767, n))
        buf.append((n >> 8) & 0xFF)
        buf.append(n & 0xFF)
    buf.append(0xFF)
    uart.write(bytearray(buf))
def UART_Receive_Cmd():
    global mode
    if uart.any():
        data = uart.read(1)
        if data:
            cmd_buf.append(data[0])
            if len(cmd_buf) == 3:
                if cmd_buf[0] == 0xBB and cmd_buf[2] == 0xCC:
                    if cmd_buf[1] == 1:
                        mode = 1
                    elif cmd_buf[1] == 2:
                        mode = 2
                cmd_buf.clear()
def line_len(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return (dx*dx + dy*dy)**0.5
def calc_dis(p1,p2,p3,p4):
    top = line_len(p1,p2)
    right = line_len(p2,p3)
    bottom = line_len(p3,p4)
    left = line_len(p4,p1)
    side = (top + right + bottom + left) / 4
    if side < 1: return 0
    return int((FRAME_REAL_SIZE * FRAME_F) / side)
def sort_points(pts):
    pts = sorted(pts, key=lambda p: p[0] + p[1])
    p1 = pts[0]
    p3 = pts[3]
    mid = pts[1:3]
    p2 = mid[0] if mid[0][1] < mid[1][1] else mid[1]
    p4 = mid[1] if mid[0][1] < mid[1][1] else mid[0]
    return p1, p2, p3, p4
def is_valid_black_frame(img, corners):
    if len(corners) != 4: return False
    p1, p2, p3, p4 = sort_points(corners)
    w = max(p1[0],p2[0],p3[0],p4[0]) - min(p1[0],p2[0],p3[0],p4[0])
    h = max(p1[1],p2[1],p3[1],p4[1]) - min(p1[1],p2[1],p3[1],p4[1])
    if w<MIN_RECT_W or w>MAX_RECT_W or h<MIN_RECT_H or h>MAX_RECT_H: return False
    aspect_ratio = w/h
    if aspect_ratio<MIN_ASPECT_RATIO or aspect_ratio>MAX_ASPECT_RATIO: return False
    diag1 = line_len(p1,p3)
    diag2 = line_len(p2,p4)
    mean_diag = (diag1+diag2)/2
    if abs(diag1-diag2)/mean_diag*100 > MAX_DIAGONAL_ERROR: return False
    cx = (p1[0]+p2[0]+p3[0]+p4[0])//4
    cy = (p1[1]+p2[1]+p3[1]+p4[1])//4
    if not (ROI_X<=cx<=ROI_X2 and ROI_Y<=cy<=ROI_Y2): return False
    if not (MIN_ASPECT_RATIO_STRICT<=aspect_ratio<=MAX_ASPECT_RATIO_STRICT): return False
    return True
last_p1x=0; last_p1y=0; last_p2x=0; last_p2y=0
last_p3x=0; last_p3y=0; last_p4x=0; last_p4y=0; last_dis=0
lost_cnt=0; LOST_MAX=5
def save_last(p1x,p1y,p2x,p2y,p3x,p3y,p4x,p4y,dis):
    global last_p1x,last_p1y,last_p2x,last_p2y,last_p3x,last_p3y,last_p4x,last_p4y,last_dis
    last_p1x=p1x; last_p1y=p1y; last_p2x=p2x; last_p2y=p2y
    last_p3x=p3x; last_p3y=p3y; last_p4x=p4x; last_p4y=p4y; last_dis=dis
def send_last():
    send_4point(last_p1x,last_p1y,last_dis, last_p2x,last_p2y,last_dis,
                last_p3x,last_p3y,last_dis, last_p4x,last_p4y,last_dis)
def find_max_blob(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob = blob
            max_size = blob.pixels()
    return max_blob
while True:
    clock.tick()
    img = sensor.snapshot()
    UART_Receive_Cmd()
    if mode == 1:
        blobs = img.find_blobs([red_threshold])
        filtered_blobs = []
        for b in blobs:
            if 500 < b.pixels() < 50000:
                filtered_blobs.append(b)
        if filtered_blobs:
            max_blob = find_max_blob(filtered_blobs)
            img.draw_rectangle(max_blob.rect())
            img.draw_cross(max_blob.cx(), max_blob.cy())
            cx = max_blob.cx()
            cy = max_blob.cy()
            blob_width = max_blob.w()
            distance_mm = (REAL_WIDTH * focal_length) / blob_width
            dis = int(distance_mm)
            cx_f = int(kf_ball_x.update(cx))
            cy_f = int(kf_ball_y.update(cy))
            dis_f = int(kf_ball_dis.update(dis))
            send_3_data(cx_f, cy_f, dis_f)
            img.draw_string(0, 0,  "MODE: BALL", color=(255,0,0))
            img.draw_string(0, 15, "X: %d" % cx_f, color=(255,0,0))
            img.draw_string(0, 30, "Y: %d" % cy_f, color=(255,0,0))
            img.draw_string(0, 45, "DIS: %dmm" % dis_f, color=(255,0,0))
            print("FPS:%.2f | MODE:BALL | X:%d | Y:%d | DIS:%d" %
                  (clock.fps(), cx_f, cy_f, dis_f))
        else:
            send_3_data(0, 0, 0)
            kf_ball_x.x = 0
            kf_ball_y.x = 0
            kf_ball_dis.x = 0
            img.draw_string(0,0,"NO TARGET", color=(255,0,0))
            print("FPS:%.2f | MODE:BALL | NO TARGET" % clock.fps())
    elif mode == 2:
        img.draw_rectangle(ROI_X, ROI_Y, ROI_W, ROI_H, color=(0,255,0))
        blobs = img.find_blobs([BLACK_THRESHOLD], pixels_threshold=100, area_threshold=100, merge=True)
        best = None
        best_score = 99999
        for b in blobs:
            if not (MIN_RECT_W <= b.w() <= MAX_RECT_W and MIN_RECT_H <= b.h() <= MAX_RECT_H):
                continue
            corners = b.min_corners()
            if not is_valid_black_frame(img, corners):
                continue
            p1,p2,p3,p4 = sort_points(corners)
            score = abs(line_len(p1,p3)-line_len(p2,p4))
            if score < best_score:
                best_score = score
                best = b
                best_corners = corners
        if best:
            p1,p2,p3,p4 = sort_points(best_corners)
            dis = calc_dis(p1,p2,p3,p4)
            dis = int(kf_dis.update(dis))
            p1x = int(kf_p1x.update(p1[0]))
            p1y = int(kf_p1y.update(p1[1]))
            p2x = int(kf_p2x.update(p2[0]))
            p2y = int(kf_p2y.update(p2[1]))
            p3x = int(kf_p3x.update(p3[0]))
            p3y = int(kf_p3y.update(p3[1]))
            p4x = int(kf_p4x.update(p4[0]))
            p4y = int(kf_p4y.update(p4[1]))
            send_4point(p1x,p1y,dis, p2x,p2y,dis, p3x,p3y,dis, p4x,p4y,dis)
            print("P1(%d,%d,%d) P2(%d,%d,%d) P3(%d,%d,%d) P4(%d,%d,%d)" %
                  (p1x,p1y,dis, p2x,p2y,dis, p3x,p3y,dis, p4x,p4y,dis))
            img.draw_line(p1[0],p1[1],p2[0],p2[1],color=(255,255,255))
            img.draw_line(p2[0],p2[1],p3[0],p3[1],color=(255,255,255))
            img.draw_line(p3[0],p3[1],p4[0],p4[1],color=(255,255,255))
            img.draw_line(p4[0],p4[1],p1[0],p1[1],color=(255,255,255))
            img.draw_cross(best.cx(), best.cy())
            save_last(p1x,p1y,p2x,p2y,p3x,p3y,p4x,p4y,dis)
            lost_cnt = 0
        else:
            lost_cnt += 1
            if lost_cnt < LOST_MAX:
                send_last()
                print("P1(%d,%d,%d) P2(%d,%d,%d) P3(%d,%d,%d) P4(%d,%d,%d)" %
                      (last_p1x,last_p1y,last_dis, last_p2x,last_p2y,last_dis,
                       last_p3x,last_p3y,last_dis, last_p4x,last_p4y,last_dis))
            else:
                send_4point(0,0,0,0,0,0,0,0,0,0,0,0)
                print("P1(0,0,0) P2(0,0,0) P3(0,0,0) P4(0,0,0)")
