# start from base
FROM nvidia/cuda:9.0-cudnn7-devel

RUN apt-get -yqq update
RUN apt-get install -yqq git

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

# try above on a new machine
