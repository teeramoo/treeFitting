#!/usr/bin/env bash

sudo apt-get -y update && sudo apt-get dist-upgrade && sudo apt-get autoremove

sudo apt-get install -y build-essential

sudo apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

sudo apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev

sudo apt-get install -y libxvidcore-dev libx264-dev

sudo apt-get install -y libgtk-3-dev

sudo apt-get install -y libatlas-base-dev gfortran

sudo apt-get install -y python2.7-dev python3.5-dev

sudo apt install -y qtdeclarative5-dev

sudo apt-get install -y libboost-all-dev

sudo apt install -y libeigen3-dev

sudo apt-get install -y v4l-utils

sudo apt install -y libboost-all-dev

sudo apt install -y libsuitesparse-dev

sudo apt install -y libeigen3-dev

cd ~

home_dir=`pwd`
libraries_dir=$home_dir'/Libraries'
opencv_dir=$libraries_dir'/opencv3.2.0/'

mkdir -p $opencv_dir

cd $opencv_dir

wget -O opencv.zip https://github.com/opencv/opencv/archive/3.2.0.zip

unzip opencv.zip

cd $opencv_dir

wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.2.0.zip

unzip opencv_contrib.zip

cd $opencv_dir'/opencv-3.2.0/'

mkdir build

cd build

cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON .. -DCMAKE_BUILD_TYPE=RELEASE -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.2.0/modules ..

make -j4

sudo make install

echo '/usr/local/lib' | sudo tee --append /etc/ld.so.conf.d/opencv.conf

sudo ldconfig

echo 'PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig' | sudo tee --append ~/.bashrc

echo 'export PKG_CONFIG_PATH' | sudo tee --append ~/.bashrc

source ~/.bashrc

