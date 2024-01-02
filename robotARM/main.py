import scipy.io
import cv2
import time
import serial
import PySpin
import socket
from coordinate_convert import get_magnet_pose_from_flange_pose, get_flange_pose_from_magnet_pose, image_to_platform, \
    compute_homography, build_fixed_matrices
from camera_config import get_rectangle_params, setup_camera
import os
import threading

# 全局变量，用于保存最新坐标，初始值为 None
latest_target_coord = None
# 线程锁，保护最新坐标的读写
coord_lock = threading.Lock()
camera = None
camera_list = None
system = None
running = True 
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'
#scipy和numpy的版本须注意，分别1.12和1.26

ip = "192.168.1.6"
ports = [29999, 30003, 30004]
serial_port = 'COM6'  # 串口名称
baud_rate = 9600  # 波特率

# 导入地形信息    #计算旋转矩阵    #计算相机转换矩阵    # 设置 ROI 参数
terrain_data = scipy.io.loadmat('data03.mat')
Z = terrain_data['Z']
T_tool_magnet, T_world_platform, T_platform_world = build_fixed_matrices()
H = compute_homography()
roi = {
    'x': 10,
    'y': 10,
    'width': 2000,
    'height': 2000
}

def create_socket(ip, port):
    """创建并返回与指定IP和端口的socket连接"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ip, port))
    return s


def extract_coordinates(data):
    """
    从返回的字节数据中提取坐标信息，
    并转换为电机坐标（通过get_magnet_pose_from_flange_pose）
    """
    try:
        data_str = data.decode('utf-8')
        start = data_str.find('{') + 1
        end = data_str.find('}', start)
        if start == 0 or end == -1:
            return None
        values = data_str[start:end].split(',')
        if len(values) < 6:
            return None
        # 提取世界坐标系中的坐标
        x, y, z, rx, ry, rz = map(float, values[:6])
        return x, y, z, rx, ry, rz
        # 转换到电机坐标
        # return get_magnet_pose_from_flange_pose(x, y, z, rx, ry, rz, T_tool_magnet, T_platform_world)
    except Exception as e:
        print("坐标提取错误:", e)
        return None


def Dobot_Initialize():
    """机械臂初始化，发送一系列命令"""
    init_commands = [
        b'SpeedL(2)',
        b'SetCollisionLevel(1)',
        b'PayLoad(3,1)',
        b'Tool(0)',
        b'EnableRobot()'
    ]
    for cmd in init_commands[:-1]:
        sockets[29999].send(cmd)
        _ = sockets[29999].recv(100)
        time.sleep(0.001)
    # 最后一个命令等待较长时间
    sockets[29999].send(init_commands[-1])
    _ = sockets[29999].recv(100)
    time.sleep(3)


def MovL_Command(element):
    """
    根据输入的运动元素（电机坐标），先转换为法兰坐标，
    然后生成 MovL 指令
    """
    x, y, z, rx, ry, rz, speed = element
    x, y, z, rx, ry, rz = get_flange_pose_from_magnet_pose(x, y, z, rx, ry, rz, T_tool_magnet, T_world_platform)
    x, y, z, rx, ry, rz = round(x), round(y), round(z), round(rx), round(ry), round(rz)
    command = f'MovL({x},{y},{z},{rx},{ry},{rz}, SpeedL={speed})'
    return command.encode('utf-8')


def Motor_Initialize():
    """等待串口接收 'S'，收到后发送 'R' 完成电机初始化"""
    while True:
        if ser.in_waiting > 0:
            incoming = ser.read(1)
            if incoming == b'S':
                print("接收到 'S'，发送 'R'")
                ser.write(b'R')
                time.sleep(0.1)
                break
            else:
                print("接收到:", incoming)
        time.sleep(0.1)


def send_motorcommand(command):
    """
    发送电机命令，等待含 'R' 的响应返回；
    若在超时时间内未收到则返回 None
    """
    encoded_command = command.encode() if isinstance(command, str) else command
    timeout = 1
    end_time = time.time() + timeout
    while time.time() < end_time:
        ser.write(encoded_command)
        time.sleep(0.005)
        if ser.in_waiting:
            response = ser.read(ser.in_waiting).decode()
            if 'R' in response:
                time.sleep(0.5)
                return response
    return None


def get_magnet_position():
    sockets[30003].send(b'GetPose()')
    data = sockets[30003].recv(100)
    coords = extract_coordinates(data)
    return coords





def process_image(image_data, roi):
    """对图像进行处理，绘制ROI和矩形框，返回处理后的图像及矩形参数"""
    drawn_image, rect_info = get_rectangle_params(image_data, roi['x'], roi['y'], roi['width'], roi['height'])
    if rect_info:
        cv2.putText(drawn_image, f"Center: {rect_info['center']}, Angle: {rect_info['angle']:.2f}",
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
    return drawn_image, rect_info


def compute_target(rect_info, H, Z):
    # 根据矩形中心点计算在平台坐标系中的位置
    x, y = rect_info['center']
    print('xiangsu x:', x, 'y:', y)
    x, y = image_to_platform(x, y, H)
    x, y = int(x), int(y)
    print('pingtaix:', x, 'y:', y)
    z = Z[x - 1, y - 1]  # 根据地形信息获得z值
    target_position = [x, y, z, 168.31, 89.8, -10.2, 1]
    # print(target_position)
    return target_position


def send_target_command(target_position):
    # 将机械臂目标位置转换至世界坐标，并发送移动命令
    target_x, target_y, target_z, target_rx, target_ry, target_rz, _ = target_position
    world_target = get_flange_pose_from_magnet_pose(target_x, target_y, target_z, target_rx, target_ry, target_rz,
                                                    T_tool_magnet, T_world_platform)
    command = MovL_Command(target_position)
    sockets[30003].send(command)
    # print('target_position fasongle:', target_position)

    _ = sockets[30003].recv(100)  # 清除返回信息
    return world_target


def check_if_reached(world_target):
    coords = get_magnet_position()

    # print("coords",coords)
    #
    # print("world_target",world_target)
    if coords:
        x, y, z, rx, ry, rz = coords
        if (abs(world_target[0] - x) < 5 and
                abs(world_target[1] - y) < 5 and
                abs(world_target[2] - z) < 5 and
                abs(world_target[3] - rx) < 1 and
                abs(world_target[4] - ry) < 1 and
                abs(world_target[5] - rz) < 1):
            return True


def initialize_system():
    # 初始化全局通信
    global sockets, ser
    sockets = {port: create_socket(ip, port) for port in ports}
    ser = serial.Serial(serial_port, baud_rate, timeout=0.1)  # 设置超时，避免阻塞

    Dobot_Initialize()
    Motor_Initialize()
    send_motorcommand('R5,0')
    print("机械臂与电机初始化完成，磁铁开始旋转")


def cleanup():
    global camera, camera_list, system, sockets, ser
    try:
        send_motorcommand('R0,0')  # 关闭电机
        if 29999 in sockets:
            sockets[29999].send(b'DisableRobot()')  # 机械臂下使能

        if camera is not None:
            camera.EndAcquisition()
            camera.DeInit()
        if camera_list is not None:
            camera_list.Clear()
        if system is not None:
            system.ReleaseInstance()
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"资源释放过程中出现异常: {e}")
    finally:
        for s in sockets.values():
            s.close()
        if ser:
            ser.close()
        print("所有资源均已释放，程序退出")

    
def camera_thread():
    """相机线程，不断获取最新坐标并更新全局变量"""
    global camera, camera_list, system, latest_target_coord, running

    try:
        # 初始化相机
        camera, camera_list, system = setup_camera(camera_index=0, exposure_time_value=30000,
                                                     gain_value=5, frame_rate_value=25)
        if camera is None:
            print("相机初始化失败，退出相机线程")
            running = False
            return
        print("相机线程启动")
        camera.BeginAcquisition()
        cv2.namedWindow('raw', cv2.WINDOW_NORMAL)  # WINDOW_NORMAL 允许调整窗口大小
        cv2.resizeWindow('raw', 600, 600)  # 设置窗口大小为 800x600
        cv2.namedWindow('Processed Camera', cv2.WINDOW_NORMAL)  # WINDOW_NORMAL 允许调整窗口大小
        cv2.resizeWindow('Processed Camera', 600, 600)  # 设置窗口大小为 800x600

        while running:
            image_result = camera.GetNextImage()
            image_data = image_result.GetNDArray()
            cv2.imshow('raw', image_data)

            # 处理图像：绘制矩形框并获取参数
            drawn_image, rect_info = process_image(image_data, roi)
            cv2.imshow('Processed Camera', drawn_image)
            with coord_lock:
                    latest_target_coord = rect_info

            # if drawn_image is not None and rect_info is not None:
            #     cv2.imshow('Processed Camera', drawn_image)
            #     with coord_lock:
            #         latest_target_coord = rect_info

            image_result.Release()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                running = False
                break
            time.sleep(0.01)  # 根据实际情况调整采集频率
    except Exception as e:
        print(f"相机线程异常: {e}")
    finally:
        print("相机线程退出")


def motion_thread():
    """运动线程，从最新坐标计算目标位置、发送指令并等待到达"""
    global latest_target_coord, running
    # 初始化机械臂与电机

    print("运动线程启动")
    try:
        while running:
            # print("运动线程运行中...")
            with coord_lock:
                rect_info = latest_target_coord
            # print('rect_info:', rect_info)
            
            if rect_info:
                target_position = compute_target(rect_info, H, Z)
                # print("目标位置:", target_position)
                world_target = send_target_command(target_position)
                # print("世界坐标:", world_target)
                # 等待机械臂到达目标位置
                while running and not check_if_reached(world_target):
                    # print('3')
                    time.sleep(0.01)
                print("机械臂到达目标位置，等待robot移动")
                
            time.sleep(1)  # 根据实际情况调整检测频率
    except Exception as e:
        print(f"运动线程异常: {e}")
    finally:
        print("运动线程退出")

def main():
    """
    执行流程
    0、计算必要信息
    1、初始化相机
    2、初始化机械臂与电机
    3、从相机中获得毛毛虫位姿
    4、确保移动机械臂中心位置至毛毛虫位置---若没有，则循环获取位置数据，直至机械臂到达目标位置
    6、根据毛毛虫位姿调整机械臂位姿
    7、使用键盘中断，当q按下时启动结束程序：电机停、机械臂下使能、关闭串口、关闭相机
    """

    global running
    try:
        #初始化系统
        initialize_system()
        print("系统初始化完毕，主程序开始运行...")

        # 启动相机线程和运动线程（只启动一次）
        cam_thread = threading.Thread(target=camera_thread, daemon=True)
        cam_thread.start()
        time.sleep(5)  # 等待相机初始化完成
        mot_thread = threading.Thread(target=motion_thread, daemon=True)
        mot_thread.start()
        while running:
            # 主循环只负责监听退出信号
            if input("按下 'q' 键退出程序: ") == 'q':
                print("接收到退出信号，准备退出程序...")
                running = False
                break
            time.sleep(0.05)  # 降低 CPU 占用
    except Exception as e:
        print(f"运行过程中发生异常: {e}")
    finally:
        # 等待线程结束
        cam_thread.join()
        mot_thread.join()
        cleanup()
        print("程序安全退出")


if __name__ == "__main__":
    main()
