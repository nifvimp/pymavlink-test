# !/usr/bin/env python3
import argparse
import os
import platform
import time
import subprocess
import sys


def wslpath(path: str, opts: str = "", distro: str = None) -> str:
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
    containerization_dir = os.path.join(script_dir, '../containerization')
    parser = argparse.ArgumentParser(description='')

    parser.add_argument('--world', type=str, default=os.path.join(script_dir,'../templates/ardupilot-examples/worlds/iris.wbt'), help='')
    parser.add_argument('--param', type=str, default=os.path.join(script_dir, '../templates/ardupilot-examples/params/iris.parm'), help='')
    parser.add_argument('--entrypoint', type=str, default=os.path.join(script_dir, './main.py'), help='')
    parser.add_argument('--container', type=str, default='drone-container', help='')
    parser.add_argument('--launcher', type=str, default=os.path.join(script_dir, 'launcher.sh'), help='')
    parser.add_argument('--webots', type=str, default='C:\\Program Files\\Webots\\Webots.lnk', help='')

    args = parser.parse_args()

    treat = lambda path: f"'{path}'" if ' ' in path else path

    args.world = treat(os.path.abspath(args.world))
    args.param = treat(os.path.abspath(args.param))
    args.entrypoint = treat(os.path.abspath(args.entrypoint))
    args.launcher = treat(os.path.abspath(args.launcher))
    args.webots = treat(os.path.abspath(args.webots))

    sim_vehicle_cmd = ["wsl", "-d", args.container, wslpath(args.launcher), wslpath(args.param)]
    webots_cmd = ["wsl", "-d", args.container, "webots", wslpath(args.world)]
    vehicle_ctrl_cmd = [sys.executable, os.path.join(args.project_directory, args.entrypoint)]

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
    # os.system(f"cp -r {args.project_directory} {treat(os.path.join(containerization_dir, 'mnt/'))}")
    # os.system(f"cp {args.launcher} {treat(os.path.join(containerization_dir, 'mnt/launcher.sh'))}")
    # os.system(f"cp {args.param} {treat(os.path.join(containerization_dir, 'mnt/vehicle.parm'))}")
    # sim_vehicle_cmd = ["docker", "compose", "--project-directory", treat(containerization_dir), "up"]
    # webots_cmd = [args.webots, args.world]
    # vehicle_ctrl_cmd = [sys.executable, os.path.join(args.project_directory, args.entrypoint)]