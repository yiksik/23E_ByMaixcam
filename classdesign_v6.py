'''
    红笔控制系统：
    电控给的需求：识别矩形框四个角点以及红笔位置
    矩形使用find_rects接口进行识别
    红笔光斑位置使用找色块方法进行识别

    关键：
    1.矩形识别是四个边缘点，并不在黑胶带中间，所以矩形需要等比例缩小处理
    2.缓解光斑泛白的方法：降低亮度
    3.缓解光斑在黑胶带上很小难以识别的办法：拉高图像饱和度，降低色块识别的像素阈值
'''

from maix import image, camera, display, touchscreen, app, pinmap, uart
import struct

# 串口配置
pinmap.set_pin_function("A18", "UART1_RX")
pinmap.set_pin_function("A19", "UART1_TX")
device = "/dev/ttyS1"
serial1 = uart.UART(device, 115200)

# 摄像头配置
cam = camera.Camera(320, 240)
disp = display.Display()

# 全局变量，习惯使用c/c++但不习惯python的建议全扔全局变量
black_corners = [[0,0], [0,0], [0,0], [0,0]]
black_corners_temp = [[0,0], [0,0], [0,0], [0,0]]
black_corners_found = 0
blobs_roi = [0,0,0,0]
thresholds = [[0, 80, 40, 80, 10, 80]]
last_blob = [0,0]
blob_found = 0

# 触屏ui设计
ts = touchscreen.TouchScreen()
exit_label = "< Exit"
size = image.string_size(exit_label)
exit_btn_pos = [0, 0, 8*2 + size.width(), 12 * 2 + size.height()]
def is_in_button(x, y, btn_pos):
    return x > btn_pos[0] and x < btn_pos[0] + btn_pos[2] and y > btn_pos[1] and y < btn_pos[1] + btn_pos[3]

# 串口编码函数
def encode_points(x1,y1,x2,y2,x3,y3,x4,y4,x0,y0):
    '''
        发送16位无符号整型的10个坐标值，加上帧头帧尾一块儿打包
    '''
    body = b""
    body += struct.pack("<BBHHHHHHHHHHBB", 0xaa, 0xbb, x1,y1,x2,y2,x3,y3,x4,y4,x0,y0, 0xcc, 0xdd)
    return body

# 矩形缩小函数：这个写的有点糙因为着急完工，可以优化一下
def recul_rect(black_corners_t, ratio_x, ratio_y):
    recul_black_corners_t = [
        [black_corners_t[0][0], black_corners_t[0][1]],
        [black_corners_t[1][0], black_corners_t[1][1]],
        [black_corners_t[2][0], black_corners_t[2][1]],
        [black_corners_t[3][0], black_corners_t[3][1]]
    ]
    x0 = (black_corners_t[0][0] + black_corners_t[1][0] + black_corners_t[2][0] + black_corners_t[3][0]) / 4
    y0 = (black_corners_t[0][1] + black_corners_t[1][1] + black_corners_t[2][1] + black_corners_t[3][1]) / 4
    recul_black_corners_t[0][0] = int(ratio_x * black_corners_t[0][0] - (ratio_x - 1) * x0)
    recul_black_corners_t[1][0] = int(ratio_x * black_corners_t[1][0] - (ratio_x - 1) * x0)
    recul_black_corners_t[2][0] = int(ratio_x * black_corners_t[2][0] - (ratio_x - 1) * x0)
    recul_black_corners_t[3][0] = int(ratio_x * black_corners_t[3][0] - (ratio_x - 1) * x0)
    recul_black_corners_t[0][1] = int(ratio_y * black_corners_t[0][1] - (ratio_y - 1) * y0)
    recul_black_corners_t[1][1] = int(ratio_y * black_corners_t[1][1] - (ratio_y - 1) * y0)
    recul_black_corners_t[2][1] = int(ratio_y * black_corners_t[2][1] - (ratio_y - 1) * y0)
    recul_black_corners_t[3][1] = int(ratio_y * black_corners_t[3][1] - (ratio_y - 1) * y0)
    return recul_black_corners_t

# 主函数逻辑
while not app.need_exit():
    # 退出程序的ui检测，方便刷新识别结果
    x, y, pressed = ts.read()
    if is_in_button(x, y, exit_btn_pos):
        app.set_exit_flag(True)
    
    # 获取图像
    img = cam.read()
    img_cpoy = img.copy()

    # ui按键
    img_cpoy.draw_string(8, 12, exit_label, image.COLOR_WHITE)
    img_cpoy.draw_rect(exit_btn_pos[0], exit_btn_pos[1], exit_btn_pos[2], exit_btn_pos[3],  image.COLOR_WHITE, 2)
    img_cpoy.draw_circle(x, y, 1, image.Color.from_rgb(255, 255, 255), 2)

    # 找黑框：思路，只找一次，黑框换位置就重启应用
    if black_corners_found == 0:
        rects = img.find_rects(threshold=80000)
        max_area = 0
        for _rect in rects:
            if _rect[2] * _rect[3] > max_area:
                max_area = _rect[2] * _rect[3]
                rect = _rect
        if rects:
            black_corners_temp = rect.corners()
            black_corners_found = 1
            # 找到矩形之后拉低亮度和提高饱和度
            cam.luma(0)		       # 设置亮度，范围[0, 100]
            cam.saturation(100)    # 识别红笔光斑的关键！特别是红笔功率不够的情况下！
            blobs_roi = rect[0]-30, rect[1]-30, rect[2]+60, rect[3]+60
    else:
        # 矩形等比例缩小
        black_corners = recul_rect(black_corners_t=black_corners_temp, ratio_x=1, ratio_y = 1)

        # 一些绘图操作
        img_cpoy.draw_rect(blobs_roi[0], blobs_roi[1], blobs_roi[2], blobs_roi[3], color=image.Color.from_rgb(0, 255, 0), thickness=1)
        img_cpoy.draw_circle(black_corners[0][0], black_corners[0][1],radius = 2 ,color=image.Color.from_rgb(0, 255, 0), thickness=-1)
        img_cpoy.draw_circle(black_corners[1][0], black_corners[1][1],radius = 2 ,color=image.Color.from_rgb(0, 255, 0), thickness=-1)
        img_cpoy.draw_circle(black_corners[2][0], black_corners[2][1],radius = 2 ,color=image.Color.from_rgb(0, 255, 0), thickness=-1)
        img_cpoy.draw_circle(black_corners[3][0], black_corners[3][1],radius = 2 ,color=image.Color.from_rgb(0, 255, 0), thickness=-1)
        
        # 找红笔光斑
        blobs = img.find_blobs(thresholds, roi=blobs_roi, pixels_threshold=1)
        blobs_max_area = 0
        for blob in blobs:
            if blob[2] * blob[3] > blobs_max_area:
                blobs_max_area = blob[2] * blob[3]
                last_blob[0] = blob[0] + blob[2]//2
                last_blob[1] = blob[1] + blob[3]//2
                blob_found = 1
        if blob_found:
            img_cpoy.draw_circle(last_blob[0], last_blob[1], radius = 2 ,color=image.Color.from_rgb(255, 0, 0), thickness=-1)
            
            # 编码和发送数据
            body = encode_points(black_corners[3][0], black_corners[3][1], black_corners[2][0], black_corners[2][1],
            black_corners[1][0], black_corners[1][1], black_corners[0][0], black_corners[0][1], 
            last_blob[0], last_blob[1])
            serial1.write(body)

    disp.show(img_cpoy)