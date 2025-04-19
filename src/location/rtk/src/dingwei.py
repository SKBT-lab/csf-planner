import numpy as np
import math

np.set_printoptions(precision=20, suppress=True)
# step1:目标位置转为相机坐标系
'''
function:carmera_system
相机坐标系，云台坐标系：飞机抖动造成的测量误差 时间误差
相机云台安装误差 
input:agle_fangwei 方位角
      agle_fuyang 俯仰角
      distance 测距值
'''


def carmera_system(agle_fangwei, agle_fuyang, distance):
    agle_fangwei = (agle_fangwei * math.pi) / 180
    agle_fuyang = (agle_fuyang * math.pi) / 180
    camera_cor = np.empty([1, 4], dtype=float)  # 相机坐标系坐标定义
    camera_cor[0][0] = distance * np.cos(agle_fuyang) * np.cos(agle_fangwei)
    camera_cor[0][1] = distance * np.cos(agle_fuyang) * np.sin(agle_fangwei)
    camera_cor[0][2] = distance * np.sin(agle_fuyang)
    camera_cor[0][3] = 1
    return camera_cor


# step2:相机坐标系转为机体坐标系
'''
function:plane_system
机体坐标系：
input:agle_yaw 航向角
      agle_pitch 俯仰角
      agle_roll 横滚角
      carmera_cor 相机坐标系坐标矩阵

'''


def plane_system(agle_yaw, agle_pitch, agle_roll, carmera_cor):
    agle_yaw = (agle_yaw * math.pi) / 180
    agle_pitch = (agle_pitch * math.pi) / 180
    agle_roll = (agle_roll * math.pi) / 180
    T_x1 = np.array([[1, 0, 0, 0],
                     [0, np.cos(agle_roll), -1 * np.sin(agle_roll), 0],
                     [0, np.sin(agle_roll), np.cos(agle_roll), 0],
                     [0, 0, 0, 1]])  # 绕x轴旋转
    T_y1 = np.array([[np.cos(agle_pitch), 0, np.sin(agle_pitch), 0],
                     [0, 1, 0, 0],
                     [-1 * np.sin(agle_pitch), 0, np.cos(agle_pitch), 0],
                     [0, 0, 0, 1]])  # 绕y轴旋转
    T_z1 = np.array([[np.cos(agle_yaw), -1 * np.sin(agle_yaw), 0, 0],
                     [np.sin(agle_yaw), np.cos(agle_yaw), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])  # 绕z轴旋转
    T_all_1 = np.dot(T_z1, T_y1)
    T_all = np.dot(T_all_1, T_x1)
    plane_cor = np.dot(T_all, carmera_cor)
    return plane_cor


# step3:机体坐标系转为地理坐标系
'''
function:geo_system
地理坐标系：
input:agle_yaw 飞机航向角 单位角度
      agle_pitch 飞机俯仰角
      agle_roll 飞机横滚角
      plane_cor 机体坐标系坐标
'''


def geo_system(agle_yaw, agle_pitch, agle_roll, plane_cor):
    # 角度转化
    agle_yaw = (agle_yaw * math.pi) / 180
    agle_pitch = (agle_pitch * math.pi) / 180
    agle_roll = (agle_roll * math.pi) / 180
    T_x2 = np.array([[1, 0, 0, 0],
                     [0, np.cos(agle_roll), -1 * np.sin(agle_roll), 0],
                     [0, np.sin(agle_roll), np.cos(agle_roll), 0],
                     [0, 0, 0, 1]])  # 绕x轴旋转
    T_y2 = np.array([[np.cos(agle_pitch), 0, np.sin(agle_pitch), 0],
                     [0, 1, 0, 0],
                     [-1 * np.sin(agle_pitch), 0, np.cos(agle_pitch), 0],
                     [0, 0, 0, 1]])  # 绕y轴旋转
    T_z2 = np.array([[np.cos(agle_yaw), -1 * np.sin(agle_yaw), 0, 0],
                     [np.sin(agle_yaw), np.cos(agle_yaw), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])  # 绕z轴旋转
    T_all_1 = np.dot(T_z2, T_y2)
    T_all = np.dot(T_all_1, T_x2)
    geo_cor = np.dot(T_all, plane_cor)
    return geo_cor


# step4:地理坐标系转为空间直角坐标系
'''
function:SpacRec_system
空间直角坐标系：
input:plane_h 飞机高度
      plane_lat 飞机纬度
      plane_lon 飞机经度
      geo_cor 地理坐标系坐标
'''


def SpacRec_system(plane_h, plane_lat, plane_lon, geo_cor):
    if plane_lon < 0:
        plane_lon = plane_lon + 180
    plane_lat = (plane_lat * math.pi) / 180
    # print(plane_lat)
    plane_lon = (plane_lon * math.pi) / 180
    e_2 = 0.00669437999013  # 第一偏心率的平方
    a = 6378137  # 地球长半轴
    N = a / np.sqrt(1 - e_2 * ((np.sin(plane_lat)) ** 2))
    X = (N + plane_h) * np.cos(plane_lat) * np.cos(plane_lon)
    Y = (N + plane_h) * np.cos(plane_lat) * np.sin(plane_lon)
    Z = (N * (1 - e_2) + plane_h) * np.sin(plane_lat)
    agle_fai = 0.5 * math.pi - plane_lat  # latitude 纬度
    agle_theta = 0.5 * math.pi + plane_lon  # longitude 经度

    # 旋转矩阵
    T_1 = np.array([[1, 0, 0, 0],
                    [0, np.cos(agle_fai), -1 * np.sin(agle_fai), 0],
                    [0, np.sin(agle_fai), np.cos(agle_fai), 0],
                    [0, 0, 0, 1]])  # 绕纬度
    T_2 = np.array([[np.cos(agle_theta), -1 * np.sin(agle_theta), 0, 0],
                    [np.sin(agle_theta), np.cos(agle_theta), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])  # 绕经度旋转

    # 平移矩阵
    T_3 = np.array([[1, 0, 0, X],
                    [0, 1, 0, Y],
                    [0, 0, 1, Z],
                    [0, 0, 0, 1]])
    T_all_1 = np.dot(T_3, T_2)
    T_all = np.dot(T_all_1, T_1)
    SpacRec_cor = np.dot(T_all, geo_cor)
    # print(T_3)
    # print(T_2)
    # print(T_1)
    '''
    plane_lat = (90-)
    # 旋转矩阵
    T_1 = np.array([[1, 0, 0, 0],
                    [0, np.sin(plane_lat), np.cos(plane_lat), 0],
                    [0, -1*np.cos(plane_lat), np.sin(plane_lat), 0],
                    [0, 0, 0, 1]])  # 绕纬度
    print(T_1)
    T_2 = np.array([[-1*np.sin(plane_lon), np.cos(plane_lon), 0, 0],
                    [-1*np.cos(plane_lon), -1*np.sin(plane_lon), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])  # 绕经度旋转
    print(T_2)
    # 平移矩阵
    T_3 = np.array([[1, 0, 0, -1*X],
                    [0, 1, 0, -1*Y],
                    [0, 0, 1, -1*Z],
                    [0, 0, 0, 1]])
    print(T_3)
    T_all_1 = np.dot(T_3, T_2)
    T_all_2 = np.dot(T_all_1, T_1)
    print(T_all_2)
    T_all = np.linalg.inv(T_all_2)
    print(T_all)
    SpacRec_cor = np.dot(T_all,geo_cor)
    '''
    return SpacRec_cor


# step5:空间直角坐标系转为大地坐标系
'''
function:ground_system
空间直角坐标系：
近似法
input:SpacRec_cor 空间直角坐标系坐标
'''


def ground_system(SpacRec_cor):
    ex2 = 0.00673949674227  # 第二偏心率
    e2 = 0.00669437999013  # 第一偏心率
    a = 6378137
    b = 6356752.3
    d = np.sqrt(SpacRec_cor[0] ** 2 + SpacRec_cor[1] ** 2)
    z = SpacRec_cor[2]
    a1 = math.atan(a * z / (b * d))
    L = math.atan(SpacRec_cor[1] / SpacRec_cor[0])
    L = np.rad2deg(L)
    B = math.atan((z + b * ex2 * ((np.sin(a1)) ** 3)) / (d - a * e2 * ((np.cos(a1)) ** 3)))
    if B < 0:
        B = -B
    # print(np.rad2deg(B))
    R = a / (np.sqrt(1 - e2 * (np.sin(B)) ** 2))
    # print(R)
    print(d / np.cos(B))
    H = d / np.cos(B) - R
    B = np.rad2deg(B)
    if L < 0:
        L = L + 180
    ground_cor = np.array([L, B, H[0]])
    return ground_cor


if __name__ == '__main__':
    # 传感器测量参数赋值
    # 载荷参数
    agle_fangwei = 30 # 载荷方位角 deg 30
    agle_fuyang = 0    # 载荷俯仰角 deg -50
    distance = 1000 # 激光测距 m 6000

    # 飞机参数
    agle_yaw = 10# 飞机航向角 deg 10
    agle_pitch = 0 # 飞机俯仰角 deg 3
    agle_roll = 0  # 飞机翻滚角 deg 4
    plane_h = 20  # 飞机飞行高度 m 9000
    plane_lat = 40  # 飞机纬度 deg
    plane_lon = 120  # 飞机经度 deg

    camera_sys_matric = carmera_system(agle_fangwei, agle_fuyang, distance).T
    print("相机坐标系矩阵为：")
    print(camera_sys_matric)

    plane_sys_matric = plane_system(0, 0, 0, camera_sys_matric)
    print("机体坐标系矩阵为：")
    print(plane_sys_matric)

    geo_sys_matric = geo_system(-1 * agle_yaw, -1 * agle_pitch, -1 * agle_roll, plane_sys_matric)
    print("地理坐标系矩阵为：")
    print(geo_sys_matric)

    SpacRec_sys_matric = SpacRec_system(plane_h, plane_lat, plane_lon, geo_sys_matric)
    print("空间直角坐标系矩阵为：")
    print(SpacRec_sys_matric)

    Geound_sys_matric = ground_system(SpacRec_sys_matric)
    print("大地坐标系矩阵为：")
    print(Geound_sys_matric)

#    Geound_sys_matric=Geound_sys_matric * math.pow(10,3)
#    Geound_sys_matric1=np.array([108.953636,34.347376,400])
#    Geound_sys_matric1=Geound_sys_matric1 * math.pow(10,3)
#    Geound_sys_matric2=Geound_sys_matric-Geound_sys_matric1
#    print(Geound_sys_matric2)