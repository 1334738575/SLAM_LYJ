# extract_kitti_params.py（Windows 可直接运行）
import numpy as np
import json

def parse_calib(calib_path):
    calib = {}
    with open(calib_path, 'r') as f:
        for line in f:
            key, val = line.strip().split(':', 1)
            calib[key] = np.array([float(x) for x in val.split()]).reshape(3, 4)
    # 分解 P0（左相机，参考传感器）内参 K
    P0 = calib['P0']
    K0 = P0[:3, :3]
    fx, fy, cx, cy = K0[0,0], K0[1,1], K0[0,2], K0[1,2]
    # 分解 P1 得到右相机相对左相机的位姿（R, t）
    P1 = calib['P1']
    R = P1[:3, :3] @ np.linalg.inv(K0)
    t = P1[:3, 3:4] / fx  # 归一化平移
    # 转换 R 为四元数（COLMAP 格式：wxyz）
    from scipy.spatial.transform import Rotation as Rt
    q = Rt.from_matrix(R).as_quat()  # [x,y,z,w] → 转 [w,x,y,z]
    q_colmap = [q[3], q[0], q[1], q[2]]
    return {
        "K0": [fx, fy, cx, cy],
        "cam1_rel": {"q": q_colmap, "t": t.flatten().tolist()}
    }


def kitti_pose_to_colmap(kitti_pose_line, frame_id, sensor_id=0):
    """
    KITTI 6×12 外参 → COLMAP images.txt 格式
    sensor_id=0: 参考传感器（左相机），sensor_id=1: 右相机
    """
    pose = np.array(kitti_pose_line.split()).reshape(3, 4).astype(np.float64)
    R = pose[:3, :3]
    t = pose[:3, 3]
    # 转换 R 为四元数（wxyz）
    from scipy.spatial.transform import Rotation as Rt
    q = Rt.from_matrix(R).as_quat()  # [x,y,z,w] → [w,x,y,z]
    q_colmap = [q[3], q[0], q[1], q[2]]
    # COLMAP images.txt 一行：image_id, qw, qx, qy, qz, tx, ty, tz, camera_id, name
    image_id = frame_id * 2 + sensor_id  # 按帧+传感器编号
    camera_id = sensor_id
    image_name = f"cam{sensor_id}/{frame_id:06d}.jpg"
    return f"{image_id} " + " ".join(map(str, q_colmap)) + " " + " ".join(map(str, t)) + f" {camera_id} {image_name}"



if __name__ == "__main__":
    # 示例调用
    homePath = r"D:\colmap-x64-windows-cuda\data\KITTI\00"
    params = parse_calib(homePath + "/calib.txt")
    print(json.dumps(params, indent=2))


    # 示例：转换第0序列的 poses.txt
    with open(homePath + "/00.txt", "r") as f, open(homePath + "poses.txt", "w") as out:
        for frame_id, line in enumerate(f):
            out.write(kitti_pose_to_colmap(line, frame_id, 0) + "\n")  # 左相机
            out.write(kitti_pose_to_colmap(line, frame_id, 1) + "\n")  # 右相机
            out.write("\n")  # 空行分隔