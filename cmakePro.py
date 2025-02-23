import subprocess
import sys
import argparse

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
    parser = argparse.ArgumentParser(description="cmake parameters")
    parser.add_argument('--src', type=str, default='.', help='source directory')
    parser.add_argument('--build', type=str, default='build', help='build directory')
    parser.add_argument('--gen', type=str, default='Visual Studio 17 2022', help='generator')
    parser.add_argument('--type', type=str, default='Release', help='build type')
    args = parser.parse_args()
    cmake_project(args.src, args.build, args.gen, args.type)