import subprocess
import sys
import os

def generate_stubs(pyd_file, output_dir=None):
    command = ['stubgen']
    command.extend(['-m', pyd_file])
    if output_dir:
        command.extend(['-o', output_dir])
    try:
        result = subprocess.run(command, check=True, text=True, capture_output=True)
        print("stub file generate successfully.")
        print(result.stdout)
    except subprocess.CalledProcessError as e:
        print("error occurred while generating stub file:")
        print(e.stderr)
        sys.exit(1)


if __name__ == "__main__":
    #path is relative to workspace of vscode, current open directory path,
    # workspace:D:/testPybind11, example_name:D:/testPybind11/build/example/Release/example_Py.pyd
    example_name = 'build.example.Release.example_Py'
    #this path is append to example_name's directory
    # output_dir_name:D:/testPybind11/build/example/Release/./
    output_dir_name = '.'
    generate_stubs(example_name, output_dir_name)

    SLAM_name = 'build.example.Release.SLAM_LYJ_Py'
    generate_stubs(SLAM_name, output_dir_name)

    QT_name = 'build.example.Release.QT_LYJ_Py'
    generate_stubs(QT_name, output_dir_name)