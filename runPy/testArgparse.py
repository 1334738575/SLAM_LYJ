import argparse

parser = argparse.ArgumentParser(description="cmake parameters")
parser.add_argument('--src', type=str, default='.', help='source directory') # must
parser.add_argument('--build', type=str, default='build', help='build directory') #-- means optional
parser.add_argument('--generator', type=str, default='Visual Studio 17 2022', help='generator')
parser.add_argument('--type', type=str, default='Release', help='build type')
parser.add_argument('--verbose', action='store_true', help='verbose is bool') # verbose is bool
parser.add_argument('--numbers', type=int, nargs='+', help='A list of numbers.') # multi-value --numbers 1 2 3 4
parser.add_argument('--mode', choices=['easy', 'medium', 'hard'], default='easy', help='Game mode.') # choose
args = parser.parse_args()
# -h or --help to print info
print(args.mode)