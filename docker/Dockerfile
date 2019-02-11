# start from base
FROM osrf/ros:kinetic-desktop-xenial

####################CUDA CUDNN###################
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates apt-transport-https gnupg-curl && \
    rm -rf /var/lib/apt/lists/* && \
    NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5 && \
    NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80 && \
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub && \
    apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | tail -n +5 > cudasign.pub && \
    echo "$NVIDIA_GPGKEY_SUM  cudasign.pub" | sha256sum -c --strict - && rm cudasign.pub && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDA_VERSION 9.0.176

ENV CUDA_PKG_VERSION 9-0=$CUDA_VERSION-1
RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-cudart-$CUDA_PKG_VERSION && \
    ln -s cuda-9.0 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# nvidia-docker 1.0
LABEL com.nvidia.volumes.needed="nvidia_driver"
LABEL com.nvidia.cuda.version="${CUDA_VERSION}"

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=9.0"

###############Runtime################

ENV NCCL_VERSION 2.2.12

RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-libraries-$CUDA_PKG_VERSION \
        cuda-cublas-9-0=9.0.176.3-1 \
        libnccl2=$NCCL_VERSION-1+cuda9.0 && \
    rm -rf /var/lib/apt/lists/*
############Dev#####################

RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-libraries-dev-$CUDA_PKG_VERSION \
        cuda-nvml-dev-$CUDA_PKG_VERSION \
        cuda-minimal-build-$CUDA_PKG_VERSION \
        cuda-command-line-tools-$CUDA_PKG_VERSION \
        cuda-core-9-0=9.0.176.3-1 \
        cuda-cublas-dev-9-0=9.0.176.3-1 \
        libnccl-dev=$NCCL_VERSION-1+cuda9.0 && \
    rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs

##########CUDNN###################

ENV CUDNN_VERSION 7.1.4.18

RUN apt-get update && apt-get install -y --no-install-recommends \
            libcudnn7=$CUDNN_VERSION-1+cuda9.0 \
            libcudnn7-dev=$CUDNN_VERSION-1+cuda9.0 && \
    rm -rf /var/lib/apt/lists/*
##########End of installing cuda related stuff ###############


# install system-wide deps for python and node
RUN apt-get -yqq update
RUN apt-get -yqq install wget iputils-ping htop software-properties-common python-software-properties git vim-gui-common vim-runtime ffmpeg tmux python-pip

RUN pip install tensorflow-gpu

# install LinkNet dependencies
# torch
WORKDIR /root/
RUN git clone https://github.com/torch/distro.git ~/torch --recursive
WORKDIR /root/torch
RUN sed -i -e 's/sudo//g' install-deps
RUN bash install-deps
# this fix the half precison compile error
ENV TORCH_NVCC_FLAGS "-D__CUDA_NO_HALF_OPERATORS__"
WORKDIR /root/torch
RUN bash install.sh

# fix the original torch could only use cudnn v5 issue
WORKDIR /root/
RUN bash /root/torch/install/bin/torch-activate && git clone https://github.com/soumith/cudnn.torch.git -b R7 && cd cudnn.torch && /root/torch/install/bin/luarocks make cudnn-scm-1.rockspec

# lutorpy: call lua function from python
RUN apt-get -yqq install python-dev
WORKDIR /root/
RUN git clone https://github.com/imodpasteur/lutorpy.git
WORKDIR /root/lutorpy
RUN python setup.py install
WORKDIR /root/

# monocular depth estimation
RUN git clone https://github.com/mrharicot/monodepth.git

RUN git clone --recursive https://github.com/gy20073/aws.git
###darknet####
# we need this line below
RUN apt-get install -yqq libopencv-dev
WORKDIR /root/aws/yolo/darknet
RUN make -j
###pydarknet###
WORKDIR /root/aws/yolo/pydarknet
RUN pip install idna --upgrade
ENV DARKNET_HOME "/root/aws/yolo/darknet"
ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:$DARKNET_HOME
RUN apt-get install -yqq pkg-config
RUN pip install cython
ENV GPU 1
ENV OPENCV 0
RUN pip install .
WORKDIR /root/


# Download the models from GDrive
WORKDIR /root
RUN git clone https://github.com/circulosmeos/gdown.pl.git
RUN bash /root/aws/download_models.sh

RUN pip install scipy scikit-image
RUN apt-get update
RUN apt-get install python-tk

# ship it with a sample video
RUN /root/gdown.pl/gdown.pl https://drive.google.com/file/d/1SxXW-7ACUq3n9An_OwqHwGVhCP9TCOKu/view video_lowres.mkv

RUN git config --global user.email "gy20073@gmail.com"
RUN git config --global user.name "Yang Gao"

RUN apt-get install -yqq ros-kinetic-image-view

# TODO: Carla, the training code
RUN pip install h5py imgaug pygame

RUN apt-get install -y openssh-client

RUN /root/gdown.pl/gdown.pl https://drive.google.com/file/d/1_0xQQxe_ygrY5qoPK85Hz2NNpEW9d0dV/view /root/models/seg_stat_v2.t7
RUN /root/gdown.pl/gdown.pl https://drive.google.com/file/d/1ZRki61mogUfdrixf9Lh75Ja6jEqL5Fpn/view /root/models/seg_v2.net

RUN pip install torch==0.4.1 torchvision tensorboardX pillow
RUN apt-get install -y ninja-build
RUN /root/gdown.pl/gdown.pl  https://drive.google.com/file/d/1SJJx5-LFG3J3M99TrPMU-z6ZmgWynxo-/view /root/models/abn_wideres38.pth.tar

ENV CPLUS_INCLUDE_PATH "/usr/include/python2.7"

# TODO: finish the intersection detection support
#https://drive.google.com/file/d/1FUP222BcbLCKAD-tcYewODMlIIlKSuNg/view model_74.pth
#https://drive.google.com/file/d/1VpYSZgzYNvuYVBpMahdUV5m-QBqizvfd/view train_mean_42140.tensor
RUN pip install pynmea2
RUN apt-get update
RUN apt-get install ros-kinetic-nmea-msgs

RUN /root/gdown.pl/gdown.pl https://drive.google.com/open?id=1fDpj_V5xHdNVmahNcHC7tBinVZfRxtPw /root/models/drivable.pth
RUN /root/gdown.pl/gdown.pl https://drive.google.com/file/d/1HUV3eQXLpE5HkN7XPQkpik2e8xl9lk-S/view?usp=sharing /root/models/drivable_python2.pth

