# !/usr/bin/env python3
import argparse
import os
import platform
import time
import subprocess


def wslpath(path: str, opts: str = "", distro: str = None) -> str:
    # Assumes that windows root is always 'C:\' not '/'
    if path.startswith('/'):
        return path

    path = path.replace('\\', '\\\\')
    cmd = (f"wsl -d {distro} -e wslpath {opts} {path}" if distro else f"wsl wslpath {opts} {path}").split()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
    except subprocess.CalledProcessError as err:
        # NOTE: WSL errors uses UTF-16LE encoding on stdout for whatever reason
        if err.returncode == 0xFFFFFFFF:
            err_msg = '\n' + err.stdout.encode('UTF-8').decode('UTF-16LE')
            raise RuntimeError(err_msg) from err
        else:
            raise RuntimeError(err.stderr) from err

    return proc.stdout.strip()


if __name__ == "__main__":
    if platform.system() != 'Windows':
        raise NotImplementedError('')

    script_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description='')

    parser.add_argument('--world', type=str, default=os.path.join(script_dir,'../templates/ardupilot-examples/worlds/iris.wbt'), help='')
    parser.add_argument('--param', type=str, default=os.path.join(script_dir, '../templates/ardupilot-examples/params/iris.parm'), help='')
    parser.add_argument('--entrypoint', type=str, default=os.path.join(script_dir, './main.py'), help='')
    parser.add_argument('--distro', type=str, default='drone-container', help='')
    parser.add_argument('--launcher', type=str, default=os.path.join(script_dir, 'launcher.sh'), help='')

    args = parser.parse_args()

    treat = lambda path: f"'{path}'" if ' ' in path else path

    args.world = treat(args.world if args.world.startswith('/') else os.path.abspath(args.world))
    args.param = treat(args.param if args.world.startswith('/') else os.path.abspath(args.param))
    args.entrypoint = treat(args.entrypoint if args.world.startswith('/') else os.path.abspath(args.entrypoint))
    args.launcher = treat(args.launcher if args.world.startswith('/') else os.path.abspath(args.launcher))

    sim_vehicle_cmd = ["wsl", "-d", args.distro, wslpath(args.launcher), wslpath(args.param)]
    webots_cmd = ["wsl", "-d", args.distro, "webots", wslpath(args.world)]
    vehicle_ctrl_cmd = ["wsl", "-d", args.distro, "python3", wslpath(args.entrypoint)]

    print(" ".join(sim_vehicle_cmd))
    print(" ".join(webots_cmd))
    print(" ".join(vehicle_ctrl_cmd))

    sim_vehicle_p = subprocess.Popen(sim_vehicle_cmd, text=True)
    webots_p = subprocess.Popen(webots_cmd, text=True)
    vehicle_ctrl_p = subprocess.Popen(vehicle_ctrl_cmd, text=True)

    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        sim_vehicle_p.terminate()
        webots_p.terminate()
        vehicle_ctrl_p.terminate()


## Docker Stuff
# containerization_dir = os.path.join(script_dir, '../containerization')
# os.system(f"cp -r {args.project_directory} {treat(os.path.join(containerization_dir, 'mnt/'))}")
# os.system(f"cp {args.launcher} {treat(os.path.join(containerization_dir, 'mnt/launcher.sh'))}")
# os.system(f"cp {args.param} {treat(os.path.join(containerization_dir, 'mnt/vehicle.parm'))}")
# sim_vehicle_cmd = ["docker", "compose", "--project-directory", treat(containerization_dir), "up"]
# webots_cmd = [args.webots, args.world]
# vehicle_ctrl_cmd = [sys.executable, os.path.join(args.project_directory, args.entrypoint)]

## IP Utils
# def win_ipaddrs(interface_regex: str = 'vEthernet*') -> List[str]:
#     cmd = ["powershell", "(Get-NetIPAddress -InterfaceAlias '%s' | Where-Object { $_.AddressFamily -eq 'IPv4' }).IPAddress" % interface_regex]
#     try:
#         proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
#     except subprocess.CalledProcessError as err:
#         raise RuntimeError(err.stderr) from err
#
#     return proc.stdout.strip().split()
#
# def wsl_ipaddrs(distro: str = None) -> List[str]:
#     cmd = (f"wsl -d {distro} -e hostname -I" if distro else "wsl hostname -I").split()
#     try:
#         proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
#     except subprocess.CalledProcessError as err:
#         if err.returncode == 0xFFFFFFFF:
#             # NOTE: WSL errors uses UTF-16LE encoding on stdout for whatever reason
#             err_msg = '\n' + err.stdout.encode('UTF-8').decode('UTF-16LE')
#             raise RuntimeError(err_msg) from err
#         else:
#             raise RuntimeError(err.stderr) from err
#
#     return proc.stdout.strip().split()