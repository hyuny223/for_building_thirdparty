FROM ubuntu:focal

MAINTAINER hyuny223

RUN apt update && apt updage -y


RUN echo "== Start to install Dependencies ==" && \
apt install -y build-essential && \
apt install -y cmake && \
apt install -y git && \
apt install -y sudo && \
apt install -y wget && \
apt install -y ninja-build && \
apt install -y python3 && \
apt install -y python3-pip && \


RUN echo "== OpenCV Dependencies ==" && \ 
apt install -y pkg-config && \
apt install -y ffmpeg libavcodec-dev libavformat-dev libswscale-dev libxvidcore-dev libx264-dev libxine2-dev && \
apt install -y libv4l-dev v4l-utils && \
apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev && \
apt install -y libgtk-3-dev libgtk2.0-dev, libqt4-dev, libqt5-dev && \
apt install -y mesa-utils libgl1-mesa-dri libgtkgl2.0-dev libgtkglext1-dev && \
apt install -y libatlas-base-dev gfortran && \
apt install -y glew


RUN cd && \
mkdir workspace && cd workspace && \
git clone https://github.com/hyuny223/team-SLAM.git && \
cd team-SLAM && \
./build.sh
