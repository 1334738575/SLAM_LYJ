import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.insert(0, parent_dir)

import build.example.Release.examplePy as exPy


print(exPy.addPy(1,2))
testStruct = exPy.PyStruct(9)
print(testStruct.version)
print(exPy.SLAM_LYJ_VERSION())
exPy.SLAM_LYJ_VULKAN()
