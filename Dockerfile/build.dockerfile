FROM ubuntu:20.04

MAINTAINER hyuny223
ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt update -y

RUN echo "== Start to install Dependencies =="
RUN apt install -y apt-utils
RUN apt install -y build-essential
RUN apt install -y ca-certificates
RUN apt install -y cmake
RUN apt install -y git
RUN apt install -y sudo
RUN apt install -y wget
RUN apt install -y ninja-build
RUN apt install -y python3-dev python-dev
RUN apt install -y python3-pip python-pip
RUN apt install -y python3-numpy python-numpy

RUN echo "== OpenCV Dependencies =="
RUN apt install -y pkg-config
RUN apt install -y ffmpeg libavcodec-dev libavformat-dev libswscale-dev libxvidcore-dev libx264-dev libxine2-dev
RUN apt install -y libv4l-dev v4l-utils
RUN apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
RUN apt install -y libgtk-3-dev
RUN apt install -y mesa-utils libgl1-mesa-dri libgtkgl2.0-dev libgtkglext1-dev
RUN apt install -y libatlas-base-dev gfortran
RUN apt install -y libglew-dev
RUN apt install -y x11-apps x11-utils

RUN cd && mkdir workspace && cd workspace
RUN git clone -b developing https://github.com/hyuny223/team-SLAM.git
RUN cd team-SLAM && ./build.sh
