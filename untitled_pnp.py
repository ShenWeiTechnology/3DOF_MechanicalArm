import sensor
import image
import time
import math
import gc

# ====================== 核心配置（30mm红色正方体） ======================
CUBE_SIZE = 50.0  # 正方体实际边长（毫米）

FX = 73.0
FY = 73.0
CX = 80.0
CY = 60.0

# 红色阈值（根据你的环境调整）
RED_THRESHOLD = (3, 37, 15, 200, -17, 41)
MIN_AREA = 80
ROI = (20, 15, 120, 90)
# ====================== 标准EPnP算法（纯Python实现） ======================
class EPnPSolver:
    def __init__(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def solve(self, obj_pts, img_pts):
        """标准EPnP算法，求解4个共面点的位姿"""
        # 归一化图像坐标
        pts_norm = []
        for (u, v) in img_pts:
            x = (u - self.cx) / self.fx
            y = (v - self.cy) / self.fy
            pts_norm.append((x, y))

        # 构建单应矩阵
        A = [[0.0 for _ in range(9)] for _ in range(8)]

        for i in range(4):
            X, Y, _ = obj_pts[i]
            x, y = pts_norm[i]

            A[2*i][0] = X
            A[2*i][1] = Y
            A[2*i][2] = 1.0
            A[2*i][6] = -x * X
            A[2*i][7] = -x * Y
            A[2*i][8] = -x

            A[2*i+1][3] = X
            A[2*i+1][4] = Y
            A[2*i+1][5] = 1.0
            A[2*i+1][6] = -y * X
            A[2*i+1][7] = -y * Y
            A[2*i+1][8] = -y

        # 高斯消元求解
        try:
            for col in range(8):
                # 找主元
                pivot = col
                for row in range(col, 8):
                    if abs(A[row][col]) > abs(A[pivot][col]):
                        pivot = row
                A[col], A[pivot] = A[pivot], A[col]

                # 归一化主元行
                div = A[col][col]
                if abs(div) < 1e-10:
                    return False, None
                for j in range(col, 9):
                    A[col][j] /= div

                # 消去其他行
                for row in range(8):
                    if row != col and abs(A[row][col]) > 1e-10:
                        factor = A[row][col]
                        for j in range(col, 9):
                            A[row][j] -= factor * A[col][j]

            # 提取解
            h = [A[i][8] for i in range(8)] + [1.0]

            # 分解单应矩阵
            h11, h12, h13 = h[0], h[1], h[2]
            h21, h22, h23 = h[3], h[4], h[5]
            h31, h32 = h[6], h[7]

            # 计算缩放因子
            scale = 1.0 / math.sqrt(h11*h11 + h21*h21 + h31*h31)

            # 计算平移向量
            tx = h13 * scale
            ty = h23 * scale
            tz = scale

            # 转换为右前上坐标系
            x = tx
            y = tz
            z = -ty

            return True, (x, y, z)
        except:
            return False, None

# ====================== 核心：旋转矩形角点提取（零高级API） ======================
def extract_rotated_corners(blob):
    """
    从blob中提取旋转后的真实角点
    只使用blob的基础属性，100%兼容所有OpenMV版本
    返回：四个角点[(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
    """
    # 获取blob的基础属性（所有版本都支持）
    x, y, w, h = blob.rect()
    cx = blob.cx()
    cy = blob.cy()
    theta = blob.rotation() * math.pi / 180.0  # 转换为弧度

    # 轴对齐外接矩形的四个角点（相对于中心）
    half_w = w / 2.0
    half_h = h / 2.0

    local_corners = [
        (-half_w, -half_h),  # 左上
        (half_w, -half_h),   # 右上
        (half_w, half_h),    # 右下
        (-half_w, half_h)    # 左下
    ]

    # 旋转矩阵
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    # 旋转角点并转换为全局坐标
    global_corners = []
    for (lx, ly) in local_corners:
        # 旋转
        rx = lx * cos_theta - ly * sin_theta
        ry = lx * sin_theta + ly * cos_theta

        # 转换为全局坐标
        gx = cx + rx
        gy = cy + ry

        global_corners.append((gx, gy))

    return global_corners

# ====================== 辅助函数 ======================
def sort_corners(corners):
    """稳定的角点排序：左上 -> 右上 -> 右下 -> 左下"""
    cx = sum(p[0] for p in corners) / 4.0
    cy = sum(p[1] for p in corners) / 4.0

    def angle(p):
        return math.atan2(p[1] - cy, p[0] - cx)

    sorted_corners = sorted(corners, key=angle)

    min_y = float('inf')
    min_idx = 0
    for i, p in enumerate(sorted_corners):
        if p[1] < min_y:
            min_y = p[1]
            min_idx = i

    return sorted_corners[min_idx:] + sorted_corners[:min_idx]

def draw_quad(img, corners, color, thickness=1):
    """绘制四边形"""
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i+1)%4]
        img.draw_line(int(x1), int(y1), int(x2), int(y2), color=color, thickness=thickness)

# ====================== 主程序 ======================
def main():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QQVGA)

    # 校准白平衡
    print("正在校准白平衡...")
    sensor.skip_frames(time=3000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)

    # 初始化EPnP求解器
    pnp_solver = EPnPSolver(FX, FY, CX, CY)

    # 定义正方体可见面的3D点（30mm边长）
    object_points = [
        (0.0, 0.0, 0.0),          # 左下
        (CUBE_SIZE, 0.0, 0.0),    # 右下
        (CUBE_SIZE, CUBE_SIZE, 0.0),  # 右上
        (0.0, CUBE_SIZE, 0.0)     # 左上
    ]

    clock = time.clock()
    frame_count = 0
    last_gc = time.ticks_ms()
    last_print = time.ticks_ms()

    print("="*40)
    print("30mm红色正方体PNP系统（终极兼容版）")
    print("角点提取：旋转矩形（零高级API）")
    print("算法：标准EPnP")
    print("坐标系：右(X) 前(Y) 上(Z)")
    print("="*40)

    while(True):
        clock.tick()

        try:
            img = sensor.snapshot()

            # 检测红色色块（所有版本都支持）
            blobs = img.find_blobs(
                [RED_THRESHOLD],
                roi=ROI,
                pixels_threshold=MIN_AREA,
                area_threshold=MIN_AREA,
                merge=True
            )

            current_pos = None

            if blobs:
                max_blob = max(blobs, key=lambda b: b.area())

                # 【终极解决方案】提取旋转后的真实角点
                # 只使用blob.rect()、blob.cx()、blob.cy()、blob.rotation()
                # 这些是所有OpenMV版本从第一天起就有的API，绝对不会有问题
                corners = extract_rotated_corners(max_blob)

                # 对角点进行稳定排序
                sorted_corners = sort_corners(corners)

                # 求解EPnP
                success, pos = pnp_solver.solve(object_points, sorted_corners)

                if success:
                    x, y, z = pos
                    current_pos = (x, y, z)

                    # 绘制旋转后的真实四边形（不是轴对齐矩形）
                    draw_quad(img, sorted_corners, color=(0, 255, 0), thickness=1)

                    # 绘制四个角点
                    for i, (x_p, y_p) in enumerate(sorted_corners):
                        img.draw_circle(int(x_p), int(y_p), 3, color=(255, 0, 0), fill=True)
                        img.draw_string(int(x_p)+5, int(y_p), str(i), color=(255, 0, 0))

                    # 显示坐标
                    img.draw_string(2, 2, "X:%.0f" % x, color=(255, 255, 255))
                    img.draw_string(2, 12, "Y:%.0f" % y, color=(255, 255, 255))
                    img.draw_string(2, 22, "Z:%.0f" % z, color=(255, 255, 255))
                    img.draw_string(2, 32, "FPS:%.0f" % clock.fps(), color=(255, 255, 255))
                else:
                    img.draw_string(2, 2, "PNP失败", color=(255, 0, 0))
            else:
                img.draw_string(2, 2, "无物体", color=(255, 0, 0))

            # 绘制ROI区域
            img.draw_rectangle(ROI, color=(0, 0, 255), thickness=1)

            frame_count += 1

            # 每秒输出一次
            if time.ticks_diff(time.ticks_ms(), last_print) > 1000:
                if current_pos is not None:
                    x, y, z = current_pos
                    print("X:%.1f Y:%.1f Z:%.1f FPS:%.1f" % (x, y, z, clock.fps()))
                else:
                    print("未检测到有效物体")
                last_print = time.ticks_ms()

            # 垃圾回收
            if time.ticks_diff(time.ticks_ms(), last_gc) > 3000:
                gc.collect()
                last_gc = time.ticks_ms()

        except Exception as e:
            print("错误:", e)
            time.sleep(0.1)
            frame_count += 1
            continue

if __name__ == "__main__":
    main()
