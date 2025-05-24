import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.insert(0, parent_dir)

import build.example.Release.example_Py as exPy
import build.example.Release.SLAM_LYJ_Py as SLAMPy
import build.example.Release.QT_LYJ_Py as QTPy

def testExample():
    print("add:", exPy.addPy(1,2))
    testStruct = exPy.PyStruct(9)
    print("struct:", testStruct.version)
    testStruct.version = 99
    print("struct:", testStruct.print())
    vec3d = exPy.Vector3d(1, 1, 1)
    print("vector3d:", vec3d.__repr__())
    print("vector3d x:", vec3d.x())
    vv3d = []
    vv3d.append(vec3d)
    vv3d2 = exPy.testVector(vv3d)
    print("testVector:", vv3d2[0].__repr__())
    vv = [exPy.PyStruct(0), exPy.PyStruct(1)]
    vv2 = exPy.testPtr(vv)
    print("vvPtr:", vv2[0].version)


def testSLAM():
    print(SLAMPy.SLAM_LYJ_VERSION())
    SLAMPy.SLAM_LYJ_VULKAN()

def testQT():
    QTPy.testQT()

if __name__ == '__main__':
    testExample()
