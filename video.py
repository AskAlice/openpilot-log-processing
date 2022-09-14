import os
import random
import re
import time
from datetime import datetime

from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from googleapiclient.http import MediaFileUpload
from dateutil.parser import *

from main import BASE_PATH, PRODUCTION, REUSE_FILES, MIN_VIDEO_LENGTH_MINS

OAUTH_SCOPES = ['https://www.googleapis.com/auth/youtube.upload']


# ffmpeg -i 2022-05-14--01-54-24-ecamera.mp4 -i 2022-05-14--01-54-24-fcamera.mp4 -filter_complex "[0:v]crop=iw:400:50:400[v0];[v0][1:v]vstack" test.mp4
def main():
    youtube = google_auth()

    camera_views = ['e', 'f']
    camera_filenames = get_filenames(camera_views)
    for route in camera_filenames:
        # if not is_route_long_enough(route, camera_filenames, camera_views):
        #     continue

        for camera_prefix in camera_filenames[route]:
            print(f'---- {route} {camera_prefix}camera ----')
            new_video_mp4 = merge_videos(camera_prefix, route, camera_filenames)
            return

            print(f'---- uploading ----')
            continue
            initialize_upload(youtube, {
                'file': new_video_mp4,
                'title': f'{route} Dashcam {camera_prefix}camera',
                'description': f'Dashcam video uploaded from a comma 3.',
                'category': '1',
                'keywords': 'dashcam, car, self driving',
                'privacyStatus': 'private',
                'selfDeclaredMadeForKids': True
            })
            # 930 pixels (width) to work with to render gauges and stuff
            # Compress to 720p or something


def is_route_long_enough(route, camera_filenames, camera_views):
    for camera_prefix in camera_views:
        if len(camera_filenames[route][camera_prefix]) <= MIN_VIDEO_LENGTH_MINS:
            return False
    return True


def get_filenames(camera_views):
    camera_filenames = {}

    for folder, sub_dirs, files in (sorted(os.walk(BASE_PATH), key=natural_keys)):
        if folder == BASE_PATH:
            continue
        if not len(files):  # there are files
            continue

        route_name = folder[len(BASE_PATH) + 1:len(BASE_PATH) + 20 + 1]

        if route_name not in camera_filenames:  # prep the object
            camera_filenames[route_name] = {}
        for camera_prefix in camera_views:
            if camera_prefix not in camera_filenames[route_name]:
                camera_filenames[route_name][camera_prefix] = []

            for file in files:
                if file == f'{camera_prefix}camera.hevc':
                    # if route_name in camera_filenames:
                    #     camera_filenames[route_name][camera_prefix].append(os.path.join(folder[0], file))
                    # else:
                    camera_filenames[route_name][camera_prefix].append(os.path.join(folder, file))
    return camera_filenames


def merge_videos(camera_prefix, route, camera_filenames):
    start_time = get_unix_time_from_route(route)
    print(start_time)
    return
    file_path_hevc = f"{os.path.join(BASE_PATH, route)}-{camera_prefix}camera.hevc"
    file_path_mp4 = f"{os.path.join(BASE_PATH, route)}-{camera_prefix}camera.mp4"

    if REUSE_FILES and (os.path.exists(file_path_mp4) and os.path.getsize(file_path_mp4) > 0):
        print(file_path_mp4, 'already exists and is not empty, skipping')
        return file_path_mp4

    os.system(
        f"cat {' '.join(camera_filenames[route][camera_prefix])} > {file_path_hevc};"  # concat hevc files (step 1)
        f"ffmpeg -r 20 -i {file_path_hevc} -c copy {file_path_mp4};"  # repackage as mp4
        f"rm {file_path_hevc}")  # remove concat'ed hevc file from step 1
    if PRODUCTION:  # Remove original files only if production
        os.system(f"rm {' '.join(camera_filenames[route][camera_prefix])}")
    return file_path_mp4


def get_unix_time_from_route(route):
    print(route)
    start_time = datetime.strptime(route, '%Y-%m-%d--%H-%M-%S')
    print(start_time.isoformat())


def google_auth():
    creds = None
    if os.path.exists('token.json'):
        creds = Credentials.from_authorized_user_file('token.json', OAUTH_SCOPES)
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(
                'oauth_secret.json',
                OAUTH_SCOPES)
            creds = flow.run_local_server(port=0)
        with open('token.json', 'w') as token:
            token.write(creds.to_json())
    return build('youtube', 'v3', credentials=creds)


def initialize_upload(youtube, options):
    print('Uploading', options['title'])
    tags = None
    if options['keywords']:
        tags = options['keywords'].split(",")

    body = dict(
        snippet=dict(
            title=options['title'],
            description=options['description'],
            tags=tags,
            categoryId=options['category']
        ),
        status=dict(
            privacyStatus=options['privacyStatus']
        )
    )

    # Call the API's videos.insert method to create and upload the video.
    insert_request = youtube.videos().insert(
        part=",".join(body.keys()),
        body=body,
        # The chunksize parameter specifies the size of each chunk of data, in
        # bytes, that will be uploaded at a time. Set a higher value for
        # reliable connections as fewer chunks lead to faster uploads. Set a lower
        # value for better recovery on less reliable connections.
        #
        # Setting "chunksize" equal to -1 in the code below means that the entire
        # file will be uploaded in a single HTTP request. (If the upload fails,
        # it will still be retried where it left off.) This is usually a best
        # practice, but if you're using Python older than 2.6 or if you're
        # running on App Engine, you should set the chunksize to something like
        # 1024 * 1024 (1 megabyte).
        media_body=MediaFileUpload(options['file'], chunksize=-1, resumable=True)
    ).execute()

    # resumable_upload(insert_request)


# This method implements an exponential backoff strategy to resume a
# failed upload.
def resumable_upload(insert_request):
    MAX_RETRIES = 10
    RETRIABLE_STATUS_CODES = [500, 502, 503, 504]

    response = None
    error = None
    retry = 0
    media_file = MediaFileUpload()
    while response is None:
        try:
            "Uploading file..."
            status, response = insert_request.next_chunk()
            if response is not None:
                if 'id' in response:
                    print("Video id '%s' was successfully uploaded." % response['id'])
                else:
                    exit("The upload failed with an unexpected response: %s" % response)
        except HttpError:
            error = "A retriable HTTP error %d occurred:\n%s"

        if error is not None:
            print(error)
            retry += 1
            if retry > MAX_RETRIES:
                exit("No longer attempting to retry.")

            max_sleep = 2 ** retry
            sleep_seconds = random.random() * max_sleep
            print("Sleeping %f seconds and then retrying..." % sleep_seconds)
            time.sleep(sleep_seconds)


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


if __name__ == '__main__':
    main()
