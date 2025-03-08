import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.insert(0, parent_dir)

import build.example.Release.example_Py as exPy
import build.example.Release.SLAM_LYJ_Py as SLAMPy
import build.example.Release.QT_LYJ_Py as QTPy


print(exPy.addPy(1,2))
testStruct = exPy.PyStruct(9)
print(testStruct.version)

print(SLAMPy.SLAM_LYJ_VERSION())
SLAMPy.SLAM_LYJ_VULKAN()

QTPy.testQT()
