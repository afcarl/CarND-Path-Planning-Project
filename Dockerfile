FROM ubuntu:16.04

RUN apt-get update \
      && apt-get dist-upgrade -y

RUN apt-get install -y build-essential software-properties-common \
    && apt-get install -y cmake make gcc git sudo openssl libssl-dev curl \
    && apt-get clean autoclean \
    && apt-get autoremove -y

# install libuv
RUN apt-get -y install libuv1-dev

# install uWebSockets
RUN mkdir -p /project
COPY install-ubuntu.sh /project/
RUN cd /project && bash install-ubuntu.sh

COPY . /project
RUN mkdir -p /project/build \
  && cd /project/build \
  && cmake .. \
  && make

EXPOSE 4567

WORKDIR /project/build
CMD ["./path_planning"]
