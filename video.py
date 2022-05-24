import os
import re
import subprocess
import time

from tqdm import tqdm

from main import BASE_PATH, PRODUCTION


def atoi(text):
    return int(text) if text.isdigit() else text


def natural_keys(text):
    sort_key = text[0]
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [atoi(c) for c in re.split(r'(\d+)', sort_key)]


def main():
    ecamera_filenames = {}
    fcamera_filenames = {}
    for folder in (sorted(os.walk(BASE_PATH), key=natural_keys)):
        route_name = folder[0][len(BASE_PATH) + 1:len(BASE_PATH) + 20 + 1]
        if len(folder[2]):  # there are files
            for file in folder[2]:
                if file == 'ecamera.hevc':
                    if route_name in ecamera_filenames:
                        ecamera_filenames[route_name].append(os.path.join(folder[0], file))
                    else:
                        ecamera_filenames[route_name] = [os.path.join(folder[0], file)]
                if file == 'fcamera.hevc':
                    if route_name in fcamera_filenames:
                        fcamera_filenames[route_name].append(os.path.join(folder[0], file))
                    else:
                        fcamera_filenames[route_name] = [os.path.join(folder[0], file)]
    # print(ecamera_filenames)
    # time.sleep(100000)

    print('---- FCAMERA ----')
    for route in fcamera_filenames:
        print(route)
        file_path_hevc = f"{os.path.join(BASE_PATH, route)}-fcamera.hevc"
        file_path_mp4 = f"{os.path.join(BASE_PATH, route)}-fcamera.mp4"
        os.system(f"cat {' '.join(fcamera_filenames[route])} > {file_path_hevc}")  # concat hevc files
        os.system(f"ffmpeg -r 20 -i {file_path_hevc} -c copy {file_path_mp4}")  # move to mp4
        if PRODUCTION:  # Remove original files only if production
            os.system(f"rm {' '.join(fcamera_filenames[route])}")
        os.system(f"rm {file_path_hevc}")  # remove merged hevc file regardless

    print('---- ECAMERA ----')
    for route in ecamera_filenames:
        print(route)
        file_path_hevc = f"{os.path.join(BASE_PATH, route)}-ecamera.hevc"
        file_path_mp4 = f"{os.path.join(BASE_PATH, route)}-ecamera.mp4"
        os.system(f"cat {' '.join(ecamera_filenames[route])} > {file_path_hevc}")  # concat hevc files
        os.system(f"ffmpeg -r 20 -i {file_path_hevc} -c copy {file_path_mp4}")  # move to mp4
        if PRODUCTION:  # Remove original files only if production
            os.system(f"rm {' '.join(ecamera_filenames[route])}")
        os.system(f"rm {file_path_hevc}")  # remove merged hevc file regardless

    # Upload to YouTube
    # Compress to 720p or something


if __name__ == '__main__':
    main()
