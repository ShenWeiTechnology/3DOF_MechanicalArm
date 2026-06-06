import sensor, image, time
from pyb import UART
mode = 1
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()
uart = UART(3, 9600)
uart.init(9600, bits=8, parity=None, stop=1)
red_threshold = (4, 95, 24, 51, 7, 45)
REAL_WIDTH = 33
focal_length = 272
FRAME_ROI = (110, 40, 100, 100)
BLACK_FRAME_THRESHOLD = (0, 35, -15, 15, -15, 15)
roi_x, roi_y, roi_w, roi_h = FRAME_ROI
REF_Y = roi_y + roi_h // 2
REF_X = roi_x + roi_w // 2
CORNER_THRESHOLD = 40
current_direction = 1
last_valid_error = 0
last_valid_dir = 1
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
def find_max_blob(blobs):
	max_size = 0
	max_blob = None
	for blob in blobs:
		if blob.pixels() > max_size:
			max_blob = blob
			max_size = blob.pixels()
	return max_blob
kf_ball_x = Kalman1D(q=0.5, r=4.0, initial_value=0)
kf_ball_y = Kalman1D(q=0.5, r=4.0, initial_value=0)
kf_ball_dis = Kalman1D(q=1.0, r=10.0, initial_value=0)
kf_track_x = Kalman1D(q=0.5, r=4.0, initial_value=0)
kf_track_y = Kalman1D(q=1.0, r=10.0, initial_value=0)
judge_delay_cnt = 0
stable_frame = 8
while True:
	clock.tick()
	img = sensor.snapshot()
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
		blobs = img.find_blobs(
			[BLACK_FRAME_THRESHOLD],
			roi=FRAME_ROI,
			pixels_threshold=50,
			area_threshold=50,
			merge=True
		)
		max_blob = find_max_blob(blobs)
		current_error = 0
		new_direction = current_direction
		if max_blob:
			cx = max_blob.cx()
			cy = max_blob.cy()
			filtered_cx = kf_track_x.update(cx)
			filtered_cy = kf_track_y.update(cy)
			judge_delay_cnt += 1
			top_left = (roi_x, roi_y)
			top_right = (roi_x + roi_w, roi_y)
			bottom_left = (roi_x, roi_y + roi_h)
			bottom_right = (roi_x + roi_w, roi_y + roi_h)
			if judge_delay_cnt > stable_frame:
				if current_direction == 1:
					if filtered_cx - top_left[0] < CORNER_THRESHOLD:
						new_direction = 2
					current_error = filtered_cy - REF_Y
				elif current_direction == 2:
					if filtered_cy - top_left[1] < CORNER_THRESHOLD:
						new_direction = 3
					current_error = filtered_cx - REF_X
				elif current_direction == 3:
					if bottom_right[0]- filtered_cx < CORNER_THRESHOLD:
						new_direction = 4
					current_error = -(filtered_cy - REF_Y)
				elif current_direction == 4:
					if filtered_cy-bottom_left[1] < CORNER_THRESHOLD:
						new_direction = 1
					current_error = -(filtered_cx - REF_X)
				current_direction = new_direction
				last_valid_dir = new_direction
				last_valid_error = current_error
				img.draw_rectangle(max_blob.rect(), color=(255, 0, 0), thickness=2)
				img.draw_cross(int(filtered_cx), int(filtered_cy), color=(0,255,0), size=5, thickness=2)
			else:
				current_error = last_valid_error
				current_direction = last_valid_dir
				img.draw_string(10, 30, "NO FRAME!", color=(255, 0, 0), scale=2)
			img.draw_rectangle(FRAME_ROI, color=(255,255,0), thickness=2)
			if current_direction in [1, 3]:
				img.draw_line(roi_x, REF_Y, roi_x+roi_w, REF_Y, color=(0,0,255), thickness=2)
			else:
				img.draw_line(REF_X, roi_y, REF_X, roi_y+roi_h, color=(0,0,255), thickness=2)
			error_val = int( (current_error * 100 + 1500) / 6 )
			dir_val = current_direction
			send_3_data(0, error_val, dir_val)
			dir_str = ["STANDBY","RIGHT","DOWN","LEFT","UP"][current_direction]
			img.draw_string(0, 0,  "MODE: TRACK", color=(255,0,0))
			img.draw_string(0, 15, "Dir: %s" % dir_str, color=(255,0,0))
			img.draw_string(0, 30, "Err: %.2f" % current_error, color=(255,0,0))
			print("FPS:%.2f | MODE:TRACK | Dir:%d | Err:%.2f | Send:0 %d %d" %
				  (clock.fps(), current_direction, current_error, error_val, dir_val))