# !/usr/bin/env python3
import argparse, time, subprocess, platform, os
import signal
from typing import List


def wsl_ipaddrs(distro: str = None) -> List[str]:
    cmd = (f"wsl -d {distro} -e hostname -I" if distro else "wsl hostname -I").split()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
    except subprocess.CalledProcessError as err:
        if err.returncode == 0xFFFFFFFF:
            # NOTE: WSL errors uses UTF-16LE encoding on stdout for whatever reason
            err_msg = '\n' + err.stdout.encode('UTF-8').decode('UTF-16LE')
            raise RuntimeError(err_msg) from err
        else:
            raise RuntimeError(err.stderr) from err

    return proc.stdout.strip().split()


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


def win_ipaddrs(interface_regex: str = 'vEthernet (WSL (Hyper-V firewall))') -> List[str]:
    cmd = ["powershell", "(Get-NetIPAddress -InterfaceAlias '%s' | Where-Object { $_.AddressFamily -eq 'IPv4' }).IPAddress" % interface_regex]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
    except subprocess.CalledProcessError as err:
        raise RuntimeError(err.stderr) from err

    return proc.stdout.strip().split()


def unix_ipaddrs() -> List[str]:
    try:
        proc = subprocess.run(["hostname", "-I"], capture_output=True, text=True, check=True)
    except subprocess.CalledProcessError as err:
        raise RuntimeError(err.stderr) from err

    return proc.stdout.strip().split()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.realpath(__file__))
    parser = argparse.ArgumentParser(description='')

    parser.add_argument('--world', type=str, default=os.path.join(script_dir, '../templates/ardupilot-examples/worlds/iris.wbt'), help='')
    parser.add_argument('--param', type=str, default=os.path.join(script_dir, '../templates/ardupilot-examples/params/iris.parm'), help='')
    parser.add_argument('--entrypoint', type=str, default=os.path.join(script_dir, 'main.py'), help='')
    parser.add_argument('--container', type=str, default='drone-container', help='')
    parser.add_argument('--launcher', type=str, default=os.path.join(script_dir, 'launcher.sh'), help='')
    parser.add_argument('--webots', type=str, default="C:\\Program Files\\Webots\\Webots.lnk", help='')
    args = parser.parse_args()

    # tcp: 127.0.0.1: 5760 - default master
    if platform.system() != 'Windows':
        raise RuntimeError('Unsupported operating system.')

    treat = lambda path: f"'{path}'" if ' ' in path else path

    args.world = treat(os.path.abspath(args.world))
    args.param = treat(os.path.abspath(args.param))
    args.entrypoint = treat(os.path.abspath(args.entrypoint))
    args.launcher = treat(os.path.abspath(args.launcher))
    args.webots = treat(os.path.abspath(args.webots))

    sim_vehicle_cmd = ["wsl", "-d", args.container, "bash", wslpath(args.launcher), wslpath(args.param), win_ipaddrs()[0]]
    vehicle_ctrl_cmd = ["python3", args.entrypoint]
    webots_cmd = ["powershell", ".", args.webots, args.world]

    sim_vehicle_p = subprocess.Popen(sim_vehicle_cmd, text=True)
    drone_ctrl_p = subprocess.Popen(vehicle_ctrl_cmd, text=True)
    webots_p = subprocess.Popen(webots_cmd, text=True)
