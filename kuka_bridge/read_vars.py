"""Lee variables KRL por KUKAVARPROXY (solo lectura).

    ros2 run kuka_bridge kuka_vars                 # estado de ros_server / ros_stream
    ros2 run kuka_bridge kuka_vars '$OV_PRO' COM_BUF[1]
"""

import sys

from kuka_bridge.kvp_client import KvpClient, KvpError

DEFAULT = ['$MODE_OP', '$PRO_NAME1[]', '$PRO_STATE1', '$OV_PRO', 'COM_ACTION',
           'COM_GEN', 'COM_WR', 'COM_RD', 'COM_LAST', '$AXIS_ACT']


def main():
    names = sys.argv[1:] or DEFAULT
    ip = '172.31.1.147'
    client = KvpClient(ip)
    try:
        for name in names:
            try:
                print(f'{name:14s} {client.read(name).decode(errors="replace")}')
            except KvpError as e:
                print(f'{name:14s} ERROR: {e}')
    finally:
        client.close()


if __name__ == '__main__':
    main()
