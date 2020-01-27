# ref. https://hub.docker.com/r/youyu/ubuntu/dockerfile 
FROM ubuntu:16.04
MAINTAINER Yu You <youyu.youyu@gmail.com>

# old one is 2.4.11
ENV OPENCV 3.1.0

# Initial update and install of dependency that can add apt-repos
RUN apt-get -y update && apt-get install -y software-properties-common python-software-properties

# Add global apt repos
RUN add-apt-repository -y "deb http://archive.ubuntu.com/ubuntu precise universe" && \
    add-apt-repository -y "deb http://archive.ubuntu.com/ubuntu precise main restricted universe multiverse" && \
    add-apt-repository -y "deb http://archive.ubuntu.com/ubuntu precise-updates main restricted universe multiverse" && \
    add-apt-repository -y "deb http://archive.ubuntu.com/ubuntu precise-backports main restricted universe multiverse"

# Get dependencies
RUN apt-get update && apt-get install -y \
	libgtk2.0-dev \
	libjpeg-dev \
	libjasper-dev \
	libopenexr-dev cmake python-dev \
	python-numpy python-tk libtbb-dev \
	libeigen2-dev yasm libfaac-dev \
	libopencore-amrnb-dev libopencore-amrwb-dev \
	libtheora-dev libvorbis-dev libxvidcore-dev \
	libx264-dev libqt4-dev libqt4-opengl-dev \
	sphinx-common libv4l-dev libdc1394-22-dev \
	libavcodec-dev libavformat-dev libswscale-dev \
	libglew-dev libboost-dev libboost-python-dev libboost-serialization-dev \
	htop nano wget git unzip \
&& apt-get clean \
&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# abuse opt directory.
WORKDIR /opt
# install miniconda.
RUN wget --quiet --no-check-certificate https://repo.continuum.io/miniconda/Miniconda3-4.6.14-Linux-x86_64.sh --no-check-certificate -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda
    
# create and activate python virtual env with desired version
RUN /opt/conda/bin/conda create -n py python=3.7.2
RUN echo "source /opt/conda/bin/activate py" > ~/.bashrc
ENV PATH /opt/conda/envs/py/bin:$PATH
RUN /bin/bash -c "source /opt/conda/bin/activate py"
RUN pip install scikit-build

RUN cd /opt && wget http://bitbucket.org/eigen/eigen/get/3.2.10.tar.gz -O eigen3.tgz \
&& tar zxvf eigen3.tgz && cd  eigen-eigen-b9cd8366d4e8 \
&& mkdir build && cd build \
&& cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=RELEASE .. \
&& make install 

RUN cd /opt && wget https://codeload.github.com/opencv/opencv/zip/$OPENCV -O opencv.zip \
&& unzip opencv.zip \
&& cd opencv-$OPENCV && mkdir build && cd build \
&& cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_CUDA=ON -D WITH_OPENGL=ON .. \
&& make -j4 && make install \
&& ldconfig


# DRR repos
# search term: reconstructed radiograph site:github.com
# https://github.com/PayamZandiyeh/DRR-Studies
#
#

# docker build -t drr .
# docker run -it -v /media/external/scisoft/clones/DRR-renderer:/mycode drr /bin/bash
# cd /mycode
##
# rm -rf _skbuild;python setup.py build
## 
# cmake .
# make -j6 install
##
# DRRgenerator ../lola11-55.mhd ../lola11-55.raw out.png