import subprocess
import sys


def cmake_project(source_dir='.',
                buid_dir='build',
                generator_name='Visual Studio 17 2022',
                build_type="Release"):
    command = ['cmake']
    command.extend(['-S', source_dir])
    command.extend(['-B', buid_dir])
    command.extend(['-G', generator_name])
    command.extend(['-DCMAKE_BUILD_TYPE=' + build_type])
    try:
        result = subprocess.run(command, check=True, text=True, capture_output=True)
        print("cmake generate successfully.")
        print(result.stdout)
    except subprocess.CalledProcessError as e:
        print("error occurred while generating:")
        print(e.stderr)
        sys.exit(1)


if __name__ == "__main__":
    cmake_project()