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
    example_name = 'build.example.Release.example_Py'
    output_dir_name = '.'
    generate_stubs(example_name, output_dir_name)

    SLAM_name = 'build.example.Release.SLAM_LYJ_Py'
    generate_stubs(SLAM_name, output_dir_name)

    QT_name = 'build.example.Release.QT_LYJ_Py'
    generate_stubs(QT_name, output_dir_name)