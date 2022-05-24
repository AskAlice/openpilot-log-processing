import bz2
import os
import subprocess
import time
from tqdm import tqdm

# Will cleanup original files
PRODUCTION = False
BASE_PATH = r"/mnt/c/temp/nyc_drive"


def main():
    for folder in tqdm(list(os.walk(r"/mnt/c/temp/nyc_drive")), desc='Decompressing'):
        continue  ## REMOVE ME
        if len(folder[2]):
            for file in folder[2]:
                if file[-4:] == '.bz2':
                    if file[-4:] not in folder[2]:  # skip if already decompressed
                        subprocess.call(["bzip2", f'-d{"" if PRODUCTION else "k"}', os.path.join(folder[0], file)],
                                        stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)

    for folder in tqdm(list(os.walk(r"/mnt/c/temp/nyc_drive")), desc='Decoding'):
        if len(folder[2]):
            for file in folder[2]:
                route = folder[0][len(BASE_PATH) + 1:len(BASE_PATH) + 20 + 1]
                # if file == 'qlog.json':
                #     os.rename(f"{os.path.join(folder[0], file)}", f"{os.path.join(folder[0], f'{route}-qlog.json')}")
                if file == 'qlog':
                    file_path_json = f"{os.path.join(folder[0], f'{os.path.basename(folder[0])}-qlog.json')}"
                    file_path_gz = f"{os.path.join(folder[0], f'{os.path.basename(folder[0])}-qlog.json.gz')}"
                    os.system(
                        f"capnp convert binary:json log.capnp Event < {os.path.join(folder[0], file)} > {file_path_json};" # Decode
                        f"gzip --best -c {file_path_json} > {file_path_gz};" # Compress
                        f"rm {file_path_json};" # Remove json file
                        f"aws s3 cp {file_path_gz} s3://openpilot-logs;")  # Upload


## UPLOAD TO S3
# find . -name "*-qlog.json" -exec bash -c "aws s3 cp {} s3://openpilot-logs" \;

if __name__ == '__main__':
    main()
