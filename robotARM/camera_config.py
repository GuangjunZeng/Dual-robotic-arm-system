import PySpin
import cv2
import numpy as np


def init_camera(camera_index):
    system = PySpin.System.GetInstance()
    camera_list = system.GetCameras()
    if camera_list.GetSize() == 0:
        print("No camera detected!")
        return None, None, None
    camera = camera_list.GetByIndex(camera_index)
    camera.Init()
    return camera, camera_list, system


def configure_camera(camera, exposure_time_value, gain_value, frame_rate_value):
    try:
        # 关闭自动曝光并设置曝光时间（单位：微秒）
        exposure_auto = camera.ExposureAuto
        exposure_auto.SetValue(PySpin.ExposureAuto_Off)
        exposure_time = camera.ExposureTime
        exposure_time.SetValue(exposure_time_value)
        print(f"曝光时间已设置为 {exposure_time.GetValue()} 微秒。")

        # 关闭自动增益并设置增益（单位：dB）
        gain_auto = camera.GainAuto
        gain_auto.SetValue(PySpin.GainAuto_Off)
        gain = camera.Gain
        gain.SetValue(gain_value)
        print(f"增益已设置为 {gain.GetValue()} dB。")

        # 关闭自动帧率并设置帧率（FPS）
        nodemap = camera.GetNodeMap()
        acquisition_frame_rate_auto = PySpin.CEnumerationPtr(nodemap.GetNode("AcquisitionFrameRateAuto"))
        acquisition_frame_rate_auto_off = acquisition_frame_rate_auto.GetEntryByName("Off")
        acquisition_frame_rate_auto.SetIntValue(acquisition_frame_rate_auto_off.GetValue())
        print("自动帧率已关闭。")
        acquisition_frame_rate = PySpin.CFloatPtr(nodemap.GetNode("AcquisitionFrameRate"))
        acquisition_frame_rate.SetValue(frame_rate_value)
        print(f"帧率已设置为 {acquisition_frame_rate.GetValue()} FPS。")

        # 设置图像格式为 Mono8
        pixel_format = PySpin.CEnumerationPtr(nodemap.GetNode("PixelFormat"))
        pixel_format_mono8 = pixel_format.GetEntryByName("Mono8")
        pixel_format.SetIntValue(pixel_format_mono8.GetValue())
        print("图像格式已设置为 Mono8。")

    except PySpin.SpinnakerException as ex:
        print(f"配置相机参数时出错: {ex}")
        return False
    return True


def setup_camera(camera_index=0, exposure_time_value=10000, gain_value=0, frame_rate_value=25):
    """
    封装相机初始化和配置的接口函数：
    1. 根据 camera_index 初始化相机。
    2. 配置相机的曝光、增益和帧率参数。
    返回 (camera, camera_list, system) 三元组。
    """
    camera, camera_list, system = init_camera(camera_index)
    if camera is None:
        return None, None, None
    if not configure_camera(camera, exposure_time_value, gain_value, frame_rate_value):
        return None, None, None
    return camera, camera_list, system


def get_rectangle_params(image_data, roi_x, roi_y, roi_width, roi_height, area_threshold=100):
    """
    从输入的灰度图 image_data 中提取指定 ROI 区域，
    检测黑色矩形对象，并在图像上直接绘制旋转矩形框和中心标记。

    参数：
        image_data: 输入的灰度图像数据
        roi_x, roi_y, roi_width, roi_height: ROI 参数
        area_threshold: 用于过滤噪声的轮廓面积阈值

    返回：
        元组 (drawn_image, rect_info)：
            drawn_image: 在原图上绘制了矩形框的 BGR 图像
            rect_info: 如果检测到目标，返回字典 {'center': (x, y), 'angle': angle}，否则为 None
    """
    # 将灰度图转为 BGR 图（用于绘制）
    drawn_image = cv2.cvtColor(image_data, cv2.COLOR_GRAY2BGR)
    # 对图像取反后进行二值化处理
    _, binary_image = cv2.threshold(~image_data, 160, 255, cv2.THRESH_BINARY)
    # 提取 ROI 区域
    roi = binary_image[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
    # 查找轮廓
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return drawn_image, None

    # 选择面积最大的轮廓
    max_contour = None
    max_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > area_threshold and area > max_area:
            max_area = area
            max_contour = contour

    if max_contour is None:
        return drawn_image, None

    # 计算最小外接旋转矩形（坐标为 ROI 内坐标）
    rect = cv2.minAreaRect(max_contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    # 将 ROI 内坐标转换为整幅图像坐标
    box += np.array([roi_x, roi_y])
    # 在图像上绘制旋转矩形框（红色，线宽2）
    cv2.polylines(drawn_image, [box], isClosed=True, color=(0, 0, 255), thickness=2)
    # 计算中心点，并加上 ROI 偏移（minAreaRect 返回的中心为 ROI 内坐标）
    (cx, cy) = rect[0]
    center_coord = (int(cx + roi_x), int(cy + roi_y))
    # 绘制中心标记（蓝色十字）
    cv2.drawMarker(drawn_image, center_coord, (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
    # 获取旋转角度
    angle = int(rect[2])
    rect_info = {'center': center_coord, 'angle': angle}
    # 绘制 ROI 框（绿色）
    cv2.rectangle(drawn_image, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (0, 255, 0), thickness=2)
    return drawn_image, rect_info
