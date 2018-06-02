# anaconda for python 3.6
wget https://repo.anaconda.com/archive/Anaconda3-5.1.0-Linux-x86_64.sh
bash Anaconda3-5.1.0-Linux-x86_64.sh

# tensorflow 1.8 for python 3.6 and GPU version
sudo $HOME/anaconda3/bin/pip install --ignore-installed --upgrade https://storage.googleapis.com/tensorflow/linux/gpu/tensorflow_gpu-1.8.0-cp36-cp36m-linux_x86_64.whl

# CUDA 9.0 install, only if not originally installed. Only could work if with version 9.0, 9.2 doesn't work
#wget https://developer.nvidia.com/compute/cuda/9.0/Prod/local_installers/cuda_9.0.176_384.81_linux-run
#bash cuda_9.0.176_384.81_linux.run
# follow the instruction here: https://askubuntu.com/questions/799184/how-can-i-install-cuda-on-ubuntu-16-04
# If looping the login window, then do this:
# https://askubuntu.com/questions/762831/ubuntu-16-stuck-in-login-loop-after-installing-nvidia-364-drivers

# remember to "export" a bunch of things for cuda!
# export PATH="/usr/local/cuda/bin:$HOME/bin:$HOME/.local/bin:$PATH"
# export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"

# install cudnn, this will goes to /usr/lib/x86_64-linux-gnu, not /usr/local/cuda/lib64
sudo dpkg -i libcudnn7_7.1.4.18-1+cuda9.0_amd64.deb
sudo dpkg -i libcudnn7-dev_7.1.4.18-1+cuda9.0_amd64.deb
# if failed, cp files manually

# some basic software dependencies
sudo apt -y install git vim-gui-common vim-runtime ffmpeg
# if vim not compatible, add following lines to ~/.vimrc
#set nocompatible
#set backspace=indent,eol,start
#syntax on
#colorscheme desert


BASE_DIR="/data2/yang/code/aws/"
#####################LinkNet Dependencies##############################
# torch
git clone https://github.com/torch/distro.git ~/torch --recursive
cd ~/torch; bash install-deps;
# this fix the half precison compile error
export TORCH_NVCC_FLAGS="-D__CUDA_NO_HALF_OPERATORS__" 
./install.sh

# fix the original torch could only use cudnn v5 issue
git clone https://github.com/soumith/cudnn.torch.git -b R7 && cd cudnn.torch && luarocks make cudnn-scm-1.rockspec

# the LinkNet codebase
cd $BASE_DIR; mkdir linknet; cd linknet
# the pretrained models
wget https://github.com/e-lab/LinkNet/releases/download/v1.0/v1_0.zip
unzip v1_0.zip
# the code
git clone https://github.com/gy20073/LinkNet.git

# lutorpy: call lua function from python
cd ~/
git clone https://github.com/imodpasteur/lutorpy.git
cd lutorpy
python setup.py install


DATA_BASE="/data/yang/data/aws_data"
#####################YoloV3 Dependencies##############################
cd ~/
# darknet install
git clone https://github.com/pjreddie/darknet
cd darknet
make
# download the pretrained weight
cd $DATA_BASE
wget https://pjreddie.com/media/files/yolov3.weights

