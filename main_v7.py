'''
    对红笔光斑识别做了改进：
    1. 先进行普通识别(RGB+HSV)，寻找在黑框上的红色光斑
    2. 当1没找到时,使用伽马校正预处理图像进行二次识别
'''

# 识别红笔和绿笔, 并计算dx和dy
# TODO: dx,dy用串口发出去
# TODO: 策略选择：未找到两个笔时，dx和dy是否发送默认值如0

import cv2
import numpy as np
import math
import serial
import struct
import time

find_rect = True

largest_rect = None
corner_points = []

center_record = []
red_and_green_roi = []

green_record = [0.0, 0.0]
red_record = [0.0, 0.0]

red_found = False
green_found_ones = False
red_found_ones = False

dx = 0.0
dy = 0.0

def calculate_angle(p1, p2, p3):
    """计算由三个点形成的角p2-p1-p3的角度"""
    v1 = np.array([p1[0] - p2[0], p1[1] - p2[1]])
    v2 = np.array([p1[0] - p3[0], p1[1] - p3[1]])
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    if norm_v1 == 0 or norm_v2 == 0:
        return 0
    cos_angle = dot_product / (norm_v1 * norm_v2)
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle = math.degrees(math.acos(cos_angle))
    return angle

def calculate_center(points):
    """计算一组点的中心"""
    return np.mean(points, axis=0).astype(int)

def points_close(center1, center2, threshold=20):
    """判断两个中心点是否足够接近"""
    return np.linalg.norm(np.array(center1) - np.array(center2)) < threshold

def find_green_regions():
    """更新绿笔光斑位置信息(HSV)"""
    global red_and_green_roi, frame, green_record, green_found_ones
    if len(red_and_green_roi) != 4:
        return
    x1, y1, x2, y2 = red_and_green_roi
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(frame.shape[1], x2)
    y2 = min(frame.shape[0], y2)
    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    kernel = np.ones((5, 5), np.uint8)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 使用聚类算法合并相近轮廓，目的是拟合所有光晕的色块
    if len(contours) > 1:
        centers = []
        boxes = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w/2
            center_y = y + h/2
            centers.append((center_x, center_y))
            boxes.append((x, y, w, h))
        distance_threshold = 30
        merged_contours = []
        processed_indices = set()
        for i in range(len(contours)):
            if i in processed_indices:
                continue
            cluster_indices = [i]
            for j in range(i+1, len(contours)):
                if j in processed_indices:
                    continue
                dx = centers[i][0] - centers[j][0]
                dy = centers[i][1] - centers[j][1]
                distance = np.sqrt(dx*dx + dy*dy)
                if distance < distance_threshold:
                    cluster_indices.append(j)
                    processed_indices.add(j)
            merged_points = []
            for idx in cluster_indices:
                points = contours[idx].reshape(-1, 2)
                merged_points.extend(points)
            if merged_points:
                merged_points = np.array(merged_points).reshape(-1, 1, 2).astype(np.int32)
                merged_contours.append(merged_points)
        if merged_contours:
            contours = merged_contours
    # 筛选并尝试更新绿笔光斑位置信息
    max_area_green = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 10 and area > max_area_green:  # 只保留面积大于10的区域, 且只要最大的一个
            max_area_green = area
            x, y, w, h = cv2.boundingRect(contour)
            green_record[0] = x1 + x + w/2
            green_record[1] = y1 + y + h/2
            green_found_ones = True

def find_red_regions():
    global red_and_green_roi, frame, red_record, red_found, red_found_ones
    if len(red_and_green_roi) != 4:
        return
    x1, y1, x2, y2 = red_and_green_roi
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(frame.shape[1], x2)
    y2 = min(frame.shape[0], y2)
    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return
    
    # 改进的红色激光笔检测算法
    
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = red_mask1 + red_mask2
    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    b, g, r = cv2.split(roi)
    rgb_red_mask = np.zeros_like(r)
    
    # RGB阈值参数
    red_threshold = 180       # 红色通道阈值
    non_red_threshold = 150   # 非红色通道最大值
    min_contrast = 10         # 对比度
    
    rgb_red_mask[
        (r > red_threshold) & 
        (g < non_red_threshold) & 
        (b < non_red_threshold) &
        ((r - g) > min_contrast) & 
        ((r - b) > min_contrast)
    ] = 255
    
    # 合并HSV和RGB的红色掩码
    combined_mask = cv2.bitwise_and(red_mask, rgb_red_mask)
    
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 1:
        centers = []
        boxes = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w/2
            center_y = y + h/2
            centers.append((center_x, center_y))
            boxes.append((x, y, w, h))
        distance_threshold = 50
        merged_contours = []
        processed_indices = set()
        for i in range(len(contours)):
            if i in processed_indices:
                continue
            cluster_indices = [i]
            for j in range(i+1, len(contours)):
                if j in processed_indices:
                    continue
                dx = centers[i][0] - centers[j][0]
                dy = centers[i][1] - centers[j][1]
                distance = np.sqrt(dx*dx + dy*dy)
                if distance < distance_threshold:
                    cluster_indices.append(j)
                    processed_indices.add(j)
            merged_points = []
            for idx in cluster_indices:
                points = contours[idx].reshape(-1, 2)
                merged_points.extend(points)
            if merged_points:
                merged_points = np.array(merged_points).reshape(-1, 1, 2).astype(np.int32)
                merged_contours.append(merged_points)
        if merged_contours:
            contours = merged_contours
    max_area_red = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 3 and area > max_area_red:
            max_area_red = area
            x, y, w, h = cv2.boundingRect(contour)
            red_record[0] = x1 + x + w/2
            red_record[1] = y1 + y + h/2
            red_found = True
            red_found_ones = True
            
def find_rect_func():
    global center_record, find_rect, largest_rect, corner_points, red_and_green_roi
    # 寻找矩形
    # 图像预处理：转换为灰度图并高斯模糊
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # 使用自适应阈值进行二值化，增强黑色边框
    thresh = cv2.adaptiveThreshold(
        blurred, 255, 
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 11, 2
    )
    
    # 形态学操作：消除噪点并闭合边缘
    kernel = np.ones((5, 5), np.uint8)
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    closed = cv2.erode(closed, kernel, iterations=1)
    closed = cv2.dilate(closed, kernel, iterations=1)
    
    # 查找轮廓
    contours, _ = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 筛选矩形轮廓并找到最大的矩形
    max_area = 0
    
    for contour in contours:
        # 计算轮廓周长
        perimeter = cv2.arcLength(contour, True)
        # 进行多边形逼近，获取近似的轮廓
        approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
        
        # 如果近似轮廓有4个点，则可能是矩形
        if len(approx) == 4:
            # 计算轮廓面积和边界框面积
            contour_area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(approx)
            bounding_area = w * h
            
            # 筛选条件：面积足够大且轮廓面积与边界框面积比例合理
            if contour_area > 30000 and contour_area < 80000 and contour_area / bounding_area > 0.5:
                # 检查四个角的角度是否接近90度
                is_rectangle = True
                angles = []
                
                # 确保点是按顺序排列的
                # 这里假设approx中的点已经是有序的，如果不是，需要先排序
                
                for i in range(4):
                    # 获取三个相邻点来计算角度
                    p1 = approx[i][0]
                    p2 = approx[(i-1) % 4][0]
                    p3 = approx[(i+1) % 4][0]
                    
                    # 计算角度
                    angle = calculate_angle(p1, p2, p3)
                    angles.append(angle)
                    
                    # 判断角度是否在合理范围内（例如80度到100度之间）
                    if not (80 <= angle <= 100):
                        is_rectangle = False
                        break
                
                # 如果四个角都接近90度，并且面积最大，则更新矩形信息
                if is_rectangle and contour_area > max_area:
                    max_area = contour_area
                    largest_rect = approx
                    corner_points = [tuple(p[0]) for p in approx]  # 存储角点坐标
                    if len(center_record) < 3:
                        center_record.append(calculate_center(corner_points))
                    else:
                        center_record[0] = center_record[1]
                        center_record[1] = center_record[2]
                        center_record[2] = calculate_center(corner_points)
                    if len(center_record) >= 3:
                        # 已经找到超过三次矩形
                        if (points_close(center_record[0], center_record[1]) and
                        points_close(center_record[1], center_record[2]) and
                        points_close(center_record[0], center_record[2])):
                            # 且最新的三个矩形距离较近，本次识别到的矩形作为最终结果
                            find_rect = False
                            xmin = min(corner_points[0][0], corner_points[1][0],corner_points[2][0], corner_points[3][0])
                            ymin = min(corner_points[0][1], corner_points[1][1],corner_points[2][1], corner_points[3][1])
                            xmax = max(corner_points[0][0], corner_points[1][0],corner_points[2][0], corner_points[3][0])
                            ymax = max(corner_points[0][1], corner_points[1][1],corner_points[2][1], corner_points[3][1])
                            red_and_green_roi.append(xmin - 100)
                            red_and_green_roi.append(ymin - 100)
                            red_and_green_roi.append(xmax + 100)
                            red_and_green_roi.append(ymax + 100)

def reduce_brightness_gamma(image, gamma=1.5):
    """伽马校正"""
    # 创建伽马校正查找表
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")
    
    # 应用查找表进行伽马校正
    return cv2.LUT(image, table)

def main():
    # 打开摄像头，在不同系统对应索引可能不同，需要修改。

    global frame, output, red_found, dx, dy, center_record, find_rect
    cap = cv2.VideoCapture(4)

    # 串口初始化配置
    port = '/dev/ttyUSB0'
    FRAME_HEADER = bytes([0xAA, 0xBB])
    FRAME_FOOTER = bytes([0xCC, 0xDD])
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开摄像头")
        exit()

    while True:
        # 读取数据, 重置程序
        # buffer = bytearray()
        # start_time = time.time()
        
        # while (time.time() - start_time) < 0.03:
        #     if ser.in_waiting:
        #         data = ser.read(ser.in_waiting)
        #         buffer.extend(data)
                
        #         # 查找帧头
        #         header_pos = buffer.find(FRAME_HEADER)
        #         if header_pos != -1:
        #             # 移除帧头之前的数据
        #             buffer = buffer[header_pos:]
        #             # 检查是否有完整的帧
        #             if len(buffer) >= 6:  # 帧头(2) + 两个字节(2) + 帧尾(2)
        #                 # 检查帧尾
        #                 if bytes(buffer[4:6]) == FRAME_FOOTER:
        #                     # 解析无符号八位整数
        #                     int1, int2 = struct.unpack('<BB', buffer[2:4])
        #                     print(f"{int1, int2}")
        #                     if int1 == 10 and int2 == 11:
        #                         int1 = 0
        #                         int2 = 0
        #                         find_rect = 1
        #                         center_record = []
        #                     break
        #                 # 帧尾不匹配，移除帧头继续查找
        #                 buffer = buffer[2:]

        # 逐帧捕获
        ret, frame = cap.read()

        # 如果正确读取帧，ret 为 True
        if not ret:
            print("无法获取帧")
            break
         
        output = frame.copy()

        if find_rect:
            find_rect_func()
        
        else:
            # 矩形已经找到，开始寻找色块
            red_found = False
            find_green_regions()
            find_red_regions()
            if red_found == False:
                frame = reduce_brightness_gamma(frame, 0.5)
                find_red_regions()
            if green_found_ones and red_found_ones:
                # 红笔和绿笔都找到了，计算dx和dy
                dx = red_record[0] - green_record[0]
                dy = red_record[1] - green_record[1]
                # TODO: 策略选择：未找到两个笔时，dx和dy是否发送默认值如0
                
                # 在图像左上角显示dx和dy
                cv2.putText(output, f"dx: {dx:.2f}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(output, f"dy: {dy:.2f}", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 串口发送
        data = struct.pack('<ff', dx, dy)
        frame_uart = FRAME_HEADER + data + FRAME_FOOTER
        _ = ser.write(frame_uart)

        # 显示结果
        cv2.imshow('原始图像', frame)
        if find_rect == 0:
            for point in largest_rect:
                x, y = point[0]
                cv2.circle(output, (x, y), 5, (0, 0, 255), -1)
            # for i, point in enumerate(corner_points):
            #     x, y = point
                # point_text = f"Point {i+1}: ({x}, {y})"
                # # 在不同行显示每个角点的坐标
                # cv2.putText(output, point_text, (20, 90 + i*25),
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # cv2.rectangle(output, (red_and_green_roi[0], red_and_green_roi[1]), (red_and_green_roi[2], red_and_green_roi[3]), (0, 255, 0), 2)  # 绿色矩形
            # 绘制绿色中心
            cv2.circle(output, (int(green_record[0]), int(green_record[1])), 5, (0, 255, 0), -1)
            # 绘制红色中心
            cv2.circle(output, (int(red_record[0]), int(red_record[1])), 5, (0, 0, 255), -1)
            cv2.imshow('处理后图像', output)

        # 按 'q' 键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 完成所有操作后，释放捕获器并关闭窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()