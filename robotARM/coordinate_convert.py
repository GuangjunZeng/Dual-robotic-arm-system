import cv2
from scipy.spatial.transform import Rotation as R
import numpy as np

def build_fixed_matrices():
    """
    构造固定的变换矩阵：
      1. T_tool_magnet：法兰盘(tool)坐标系到电机(magnet)坐标系的固定变换矩阵
      2. T_platform_world：平台坐标系到世界坐标系的变换矩阵（即 T_platform_world = inv(T_world_platform)）
    
    全局常量（需外部定义）：
      RX_MAGNET, RY_MAGNET, RZ_MAGNET       —— 电机相对于法兰盘的欧拉角（'xyz'顺序，单位：度）
      DX_TOOL_TO_MAGNET, DY_TOOL_TO_MAGNET, DZ_TOOL_TO_MAGNET  —— 电机相对于法兰盘的平移（单位保持一致）
      DX_PLATFORM, DY_PLATFORM, DZ_PLATFORM  —— 平台相对于世界坐标系的平移（单位保持一致）
    """

    # 固定参数：法兰盘到电机的固定变换参数（单位：度，采用 'xyz' 顺序）
    DX_TOOL_TO_MAGNET = 0
    DY_TOOL_TO_MAGNET = 40
    DZ_TOOL_TO_MAGNET = 60
    RX_MAGNET = 1  # 电机相对于法兰盘的旋转角度
    RY_MAGNET = 1
    RZ_MAGNET = 1

     # 固定参数：平台原点在世界坐标系下的平移
    DX_PLATFORM = -750
    DY_PLATFORM = -270
    DZ_PLATFORM = 53
    
    # 构造 T_tool_magnet
    R_tool_magnet = R.from_euler('xyz', [RX_MAGNET, RY_MAGNET, RZ_MAGNET], degrees=True).as_matrix()
    T_tool_magnet = np.eye(4)
    T_tool_magnet[:3, :3] = R_tool_magnet
    T_tool_magnet[:3, 3] = [DX_TOOL_TO_MAGNET, DY_TOOL_TO_MAGNET, DZ_TOOL_TO_MAGNET]
    
    # 构造平台在世界坐标系下的平移矩阵 T_world_platform
    T_world_platform = np.eye(4)
    T_world_platform[:3, 3] = [DX_PLATFORM, DY_PLATFORM, DZ_PLATFORM]
    # 求其逆，得到从世界坐标系转换到平台坐标系的矩阵 T_platform_world
    T_platform_world = np.linalg.inv(T_world_platform)
    
    return T_tool_magnet, T_world_platform, T_platform_world


def get_magnet_pose_from_flange_pose(x, y, z, rx, ry, rz, T_tool_magnet, T_platform_world):
    """
    根据法兰盘在世界坐标系下的位姿，计算电机在平台坐标系下的位姿。
    
    参数：
      x, y, z, rx, ry, rz:
          法兰盘在世界坐标系下的位姿，其中 rx, ry, rz 为 'xyz' 顺序的欧拉角（单位：度）
      T_tool_magnet:
          法兰盘到电机的固定齐次变换矩阵
      T_platform_world:
          从世界坐标系转换到平台坐标系的齐次变换矩阵（即由平台平移构造的 T_platform_world）
          
    返回：
      x_magnet, y_magnet, z_magnet, rx_magnet, ry_magnet, rz_magnet
          电机在平台坐标系下的位姿，旋转部分依然以 'xyz' 顺序的欧拉角（单位：度）
    """
    # 1. 构造法兰盘在世界坐标系下的齐次变换矩阵 T_world_tool
    R_tool = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
    T_world_tool = np.eye(4)
    T_world_tool[:3, :3] = R_tool
    T_world_tool[:3, 3] = [x, y, z]
    
    # 2. 计算电机在世界坐标系下的位姿： T_world_magnet = T_world_tool @ T_tool_magnet
    T_world_magnet = np.dot(T_world_tool , T_tool_magnet)
    
    # 3. 转换到平台坐标系下： T_platform_magnet = T_platform_world @ T_world_magnet
    T_platform_magnet = np.dot(T_platform_world , T_world_magnet)
    
    # 4. 提取平移和旋转部分
    translation = T_platform_magnet[:3, 3]
    R_magnet = T_platform_magnet[:3, :3]
    magnet_euler = R.from_matrix(R_magnet).as_euler('xyz', degrees=True)
    
    return translation[0], translation[1], translation[2], magnet_euler[0], magnet_euler[1], magnet_euler[2]

def get_flange_pose_from_magnet_pose(x, y, z, rx, ry, rz, T_tool_magnet, T_world_platform):
    """
    根据电机在平台坐标系下的位姿，反推出法兰盘在世界坐标系下的位姿。
    
    参数：
      x, y, z, rx, ry, rz:
          电机在平台坐标系下的位姿，其中 rx, ry, rz 为 'xyz' 顺序的欧拉角（单位：度）
      T_tool_magnet:
          法兰盘到电机的固定齐次变换矩阵
      T_world_platform:
          平台相对于世界坐标系的平移矩阵（用于将平台坐标转换回世界坐标系）
    
    返回：
      x_flange, y_flange, z_flange, rx_flange, ry_flange, rz_flange
          法兰盘在世界坐标系下的位姿，旋转部分为 'xyz' 顺序的欧拉角（单位：度）
    """
    # 1. 构造电机在平台坐标系下的齐次变换矩阵 T_platform_magnet
    R_magnet_pose = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
    T_platform_magnet = np.eye(4)
    T_platform_magnet[:3, :3] = R_magnet_pose
    T_platform_magnet[:3, 3] = [x, y, z]
    
    # 2. 将电机位姿从平台坐标系转换到世界坐标系： T_world_magnet = T_world_platform @ T_platform_magnet
    T_world_magnet = np.dot(T_world_platform , T_platform_magnet)
    
    # 3. 根据 T_world_magnet = T_world_tool @ T_tool_magnet，反推出法兰盘在世界坐标系下的位姿：
    #    T_world_tool = T_world_magnet @ inv(T_tool_magnet)
    T_tool_magnet_inv = np.linalg.inv(T_tool_magnet)
    T_world_tool = np.dot(T_world_magnet , T_tool_magnet_inv)
    
    # 4. 提取平移和旋转部分
    translation = T_world_tool[:3, 3]
    flange_euler = R.from_matrix(T_world_tool[:3, :3]).as_euler('xyz', degrees=True)
    
    return translation[0], translation[1], translation[2], flange_euler[0], flange_euler[1], flange_euler[2]

def compute_homography():
    """
    计算从图像坐标到平台坐标的单应性矩阵

    参数:
      image_points: 平台在图像中的四个角点坐标 [(u1,v1), (u2,v2), (u3,v3), (u4,v4)]
      platform_points: 对应平台坐标系下的坐标 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
    
    返回:
      H: 3x3 单应性矩阵
    """
    # 示例：平台在图像中的四个角点（单位：像素）
    image_points = [(0, 0), (2048, 0), (0, 2048), (2048, 2048)]
    # 示例：对应平台坐标（单位：例如米），假设平台为1米×1米的正方形
    platform_points = [(0, 100), (0, 0), (-100, 100), (-100, 0)]

    src = np.array(image_points, dtype=np.float32)
    dst = np.array(platform_points, dtype=np.float32)
    H, status = cv2.findHomography(src, dst)
    return H


def image_to_platform(u, v, H):
    """
    利用单应性矩阵 H 将图像坐标 (u,v) 转换到平台坐标 (x,y)
    """
    point = np.array([u, v, 1], dtype=np.float32)
    platform_point = H.dot(point)
    platform_point /= platform_point[2]  # 归一化
    return platform_point[0], platform_point[1]