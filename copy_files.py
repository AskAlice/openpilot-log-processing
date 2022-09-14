import os

from main import COMMA_IP


def main():
    ping_response = os.system(f'ping -c 1 {COMMA_IP}')
    if ping_response != 0:
        return

if __name__ == '__main__':
    main()