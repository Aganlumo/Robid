echo "Installing dependencies for ORB_SLAM2"
mkdir src
cd src
sudo apt install libgl1-mesa-dev
sudo apt install libglew-dev
sudo apt install cmake
sudo apt install libpython2.7-dev
sudo python -mpip install numpy pyopengl Pillow pybind11
sudo apt install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
sudo apt install libdc1394-22-dev libraw1394-dev
sudo apt install libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev

echo "Installing Pangolin"
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
sudo make install

echo "Installing Opencv 3.4.3"
cd ../../
mv ../opencv.sh ./
chmod +x opencv.sh
./opencv.sh

echo "Installing Eigen 3.2.10"
wget https://gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.tar.gz
tar -xvf eigen-3.2.10.tar.gz
cd eigen-3.2.10
mkdir build
cd build
cmake ..
cmake --build .
cmake make install

echo "Building ORB_SLAM2"
cd ../../../ORB_SLAM2
chmod +x build.sh
./build.sh

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:home/david/Robid/ORB_SLAM2/Examples/ROS
echo "Building for ROS"
chmod +x build_ros.sh
./build_ros.sh

echo "Success!"
