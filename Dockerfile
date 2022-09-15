FROM python:3.10

RUN apt-get update -y
RUN apt-get upgrade -y
RUN apt-get install python3-dev capnproto ffmpeg -y

RUN pip install --root-user-action=ignore -U cython
RUN pip install --root-user-action=ignore -U setuptools

WORKDIR /app

COPY requirements.txt ./

RUN pip install --root-user-action=ignore -r requirements.txt

COPY oauth_secret.json ./
COPY token.json ./
COPY log.capnp ./
COPY video.py ./
COPY main.py ./

CMD ["/app/video.py"]
ENTRYPOINT ["python"]