import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.insert(0, parent_dir)
# sys.path.append(parent_dir)

def printSystemPath():
    # 获取系统路径列表
    system_paths = sys.path
    # 打印系统路径
    print("系统路径列表:")
    for path in system_paths:
        print(path)

import build.example.Release.examplePy as exPy


print(exPy.addPy(1,2))
testStruct = exPy.PyStruct(9)
print(testStruct.version)
print(exPy.SLAM_LYJ_VERSION())
