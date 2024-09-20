import subprocess

from connection import *
from utils import *

ARDUPILOT_PORT = 14550

def win_ipaddrs(interface_regex: str = 'vEthernet*') -> List[str]:
    cmd = ["powershell", "(Get-NetIPAddress -InterfaceAlias '%s' | Where-Object { $_.AddressFamily -eq 'IPv4' }).IPAddress" % interface_regex]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
    except subprocess.CalledProcessError as err:
        raise RuntimeError(err.stderr) from err

    return proc.stdout.strip().split()


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


if __name__ == '__main__':
    connection_string = f'{win_ipaddrs()[0]}:{ARDUPILOT_PORT}'
    drone = DroneBase(socket=connection_string)
    # thread = Debug(['SYS_STATUS'])
    # thread.start()]
    while 1:
        drone.wait_healthy()
        drone.arm()
        drone.guided()
        drone.takeoff(20)
        drone.wait_altitude(19.5)
        drone.land()
        drone.wait_altitude(0.5)
        drone.disarm()
        drone.stabilize()
