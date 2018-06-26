# start from base
FROM osrf/ros:kinetic-desktop-xenial

# install system-wide deps for python and node
RUN apt-get -yqq update
RUN apt-get -yqq install wget iputils-ping htop software-properties-common python-software-properties git vim-gui-common vim-runtime ffmpeg tmux

# install python3.6 and use it to replace the original python2.7
RUN add-apt-repository -y ppa:deadsnakes/ppa
RUN apt-get -yqq update
RUN apt-get -yqq install python3.6
# set up pip
RUN curl https://bootstrap.pypa.io/get-pip.py |  python3.6

# install cuda 9.0
RUN apt-get install module-init-tools kmod
RUN wget https://developer.nvidia.com/compute/cuda/9.0/Prod/local_installers/cuda_9.0.176_384.81_linux-run
RUN bash cuda_9.0.176_384.81_linux-run --silent --toolkit

# append paths
RUN echo "export PATH=/usr/local/cuda-9.0/bin:\$PATH" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=/usr/local/cuda-9.0/lib64:/usr/local/cuda-9.0/lib64/stubs:\$LD_LIBRARY_PATH" >> ~/.bashrc
# fix the libcuda.so.1 problem
RUN ln -s /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libcuda.so.1

ADD libcudnn7-runtime.deb ~/
ADD libcudnn7-dev.deb ~/
WORKDIR ~/
RUN dpkg -i libcudnn7-runtime.deb
RUN dpkg -i libcudnn7-dev.deb

RUN pip install --ignore-installed --upgrade https://storage.googleapis.com/tensorflow/linux/gpu/tensorflow_gpu-1.8.0-cp36-cp36m-linux_x86_64.whl
