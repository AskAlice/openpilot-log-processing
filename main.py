import bz2
import os
import subprocess
from datetime import datetime

import capnp
from tqdm import tqdm

capnp.remove_import_hook()
log_capnp = capnp.load('log.capnp')

# Will clean up original files
PRODUCTION = False
BASE_PATH = r"/mnt/c/temp/comma/realdata"
REUSE_FILES = True
MIN_VIDEO_LENGTH_MINS = 2  # minute video length minimum, must be > this many minutes long
COMMA_IP = '192.168.1.100'
SORT_BY_TIME = True


def main():
    for root, dirs, files in os.walk(BASE_PATH):
        continue  ## REMOVE ME
        if len(files):
            for file in files:
                if file[-4:] == '.bz2':
                    if file[-4:] not in files:  # skip if already decompressed
                        subprocess.call(["bzip2", f'-d{"" if PRODUCTION else "k"}', os.path.join(root, file)],
                                        stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)

    for root, dirs, files in os.walk(BASE_PATH):
        print(files)
        if len(files):
            for file in files:
                route = root[len(BASE_PATH) + 1:len(BASE_PATH) + 20 + 1]

                # validate that the route is actually a route
                try:
                    get_unix_time_from_route(route)
                except:
                    continue

                if 'rlog' in file:
                    print(route)
                    print(root, file)
                    print(os.path.join(root, file))
                    with open(os.path.join(root, file), 'rb') as f:
                        data = f.read()

                        if file[-4:] == '.bz2':
                            data = bz2.decompress(data)
                            logs = log_capnp.Event.read_multiple_bytes(data)
                        else:
                            logs = log_capnp.Event.read_multiple_bytes(data)

                        events = list(sorted(logs, key=lambda x: x.logMonoTime) if SORT_BY_TIME else logs)
                        cnt = 0
                        for event in events:
                            cnt = cnt + 1
                            # if cnt > 100:
                            #     return
                            # event.which()
                            d = event.to_dict()
                            if 'peripheralState' in d:
                                print(event.logMonoTime, d)
                        # print(len(events), 'logs')

                        # file_path_json = f"{os.path.join(root, f'{os.path.basename(root)}-qlog.json')}"
                        # file_path_gz = f"{os.path.join(root, f'{os.path.basename(root)}-qlog.json.gz')}"
                        # os.system(
                        #     f"capnp convert binary:json log.capnp Event < {os.path.join(root, file)} > {file_path_json};"  # Decode
                        #     f"gzip --best -c {file_path_json} > {file_path_gz};"  # Compress
                        #     f"rm {file_path_json};"  # Remove json file
                        #     f"aws s3 cp {file_path_gz} s3://openpilot-logs;")  # Upload


def get_unix_time_from_route(route):
    start_time = datetime.strptime(route, '%Y-%m-%d--%H-%M-%S')


## UPLOAD TO S3
# find . -name "*-qlog.json" -exec bash -c "aws s3 cp {} s3://openpilot-logs" \;

if __name__ == '__main__':
    main()
