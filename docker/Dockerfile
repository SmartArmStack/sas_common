#FROM murilomarinho/sas:jazzy
FROM ghcr.io/marinholab/ros:lyrical
LABEL authors="murilomarinho"
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

RUN apt-get update && apt-get upgrade -y
RUN python3 -m pip install --upgrade dqrobotics --break-system-packages
#RUN sudo apt-get remove -y ros-jazzy-sas-common
RUN mkdir -p /root/sas_common_devel/src/