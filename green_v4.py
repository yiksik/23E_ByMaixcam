'''
    绿笔控制系统：
    电控给的需求：识别红绿两个笔的光斑位置（给dx和dy）

    红、绿笔光斑位置使用找色块方法进行识别

    关键：
    1.与红笔系统一样，拉低亮度，抬高饱和度，这次不用识别矩形框可以一开始就操作
    2.限定ROI与1m*1m的背景差不多大，防止现场场景复杂导致误判
    3.找不到的时候保留上一帧，因为红绿笔重合是经常发生的，使用LAB空间很容易找不到
    4.补充了一个需求：当距离较小且红笔运动幅度很小时，dx和dy手动置零来减小振荡的情况
                    红笔运动幅度小的判断较为粗糙，可以优化。
'''


from maix import camera, display, app, pinmap, uart, image
import struct

# 串口配置
pinmap.set_pin_function("A18", "UART1_RX")
pinmap.set_pin_function("A19", "UART1_TX")
device = "/dev/ttyS1"
serial1 = uart.UART(device, 115200)

# 相机配置
cam = camera.Camera(320, 240)
cam.luma(0)		       # 设置亮度，范围[0, 100]
cam.saturation(100)     # 这一步是识别红色小色块的关键！
disp = display.Display()

# 全局变量
thresholds = [[0, 80, 40, 80, 10, 80]]
green_thresholds = [[0, 80, -120, -10, 0, 30]]
last_blob = [0,0]
record_red = []     #不知道怎么声明二维列表，有点人机
for i in range(20):
    record_red.append([0,0])
record_nums = 0
green_last_blob = [0,0]
blobs_roi = [80,60,160,120]     #感兴趣区域，可以写死，有可视化界面的情况下很容易移动摄像头角度
blob_found = 0
green_blob_found = 0
dx = 0
dy = 0

# 串口编码函数
def encode_points(dx, dy):
    '''
        两个16位有符号整数
    '''
    body = b""
    body += struct.pack("<BBhhBB", 0xaa, 0xbb, dx, dy, 0xcc, 0xdd)
    return body

# 主函数
while not app.need_exit():
    # 获取图像
    img = cam.read()
    img_cpoy = img.copy()

    # 画出ROI区域方便调整相机角度
    img_cpoy.draw_rect(blobs_roi[0], blobs_roi[1], blobs_roi[2], blobs_roi[3], color=image.Color.from_rgb(0, 255, 0), thickness=1)
    
    # 找色块
    blobs = img.find_blobs(thresholds, roi=blobs_roi, pixels_threshold=1, merge=True)
    blobs_max_area = 0

    # 这里使用环形缓冲区的办法来记录20组红笔位置数据
    # 每一次红笔位置与20帧前的比较，如果运动距离较小则粗糙的认为红笔处于静止状态
    record_red[record_nums] = [last_blob[0], last_blob[1]]
    record_nums += 1
    if record_nums == 20:
        record_nums = 0

    # 记录新的红笔数据
    for blob in blobs:
        if blob[2] * blob[3] > blobs_max_area:
            blobs_max_area = blob[2] * blob[3]
            last_blob[0] = blob[0] + blob[2]//2
            last_blob[1] = blob[1] + blob[3]//2
            blob_found = 1      # 找到过红笔

    # 绿笔同理，只是绿笔不记录是否静止，因为红笔是主动的，而绿笔是被动的
    green_blobs = img.find_blobs(green_thresholds, roi=blobs_roi, pixels_threshold=1, merge=True)
    blobs_max_area = 0
    for blob in green_blobs:
        if blob[2] * blob[3] > blobs_max_area:
            blobs_max_area = blob[2] * blob[3]
            green_last_blob[0] = blob[0] + blob[2]//2
            green_last_blob[1] = blob[1] + blob[3]//2
            green_blob_found = 1 

    # 红绿笔都找到过就计算dx和dy并发送
    if blob_found and green_blob_found:
        dx = last_blob[0] - green_last_blob[0]
        dy = last_blob[1] - green_last_blob[1]

        # 特殊操作
        dis2 = (record_red[record_nums][0]-last_blob[0])** 2 + (record_red[record_nums][1]-last_blob[1])** 2
        # print(f"{dis2,  dx ** 2 + dy ** 2}")  # 打印用于调试
        if dx ** 2 + dy ** 2 < 15 and dis2 <= 3:
            dx = 0
            dy = 0

        # 编码发送
        body = encode_points(dx = dx, dy = dy)
        serial1.write(body)

        # 绘图操作
        img_cpoy.draw_circle(last_blob[0], last_blob[1], radius = 2 ,color=image.Color.from_rgb(255, 0, 0), thickness=-1)
        img_cpoy.draw_circle(green_last_blob[0], green_last_blob[1], radius = 2 ,color=image.Color.from_rgb(0, 255, 0), thickness=-1)

    disp.show(img_cpoy)