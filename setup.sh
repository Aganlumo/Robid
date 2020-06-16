echo "Installing dependencies for ORB_SLAM2"
cd src
sudo apt install libgl1-mesa-dev
sudo apt install libglew-dev
sudo apt install cmake
sudo apt install libpython2.7-dev
sudo python -mpip install numpy pyopengl Pillow pybind11
sudo apt install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
sudo apt install libdc1394-22-dev libraw1394-dev
sudo apt install libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev

echo "Installing dependencies for Turtlebot3"
sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers

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

echo "Building for turtlebot3"
cd ../../
catkin_make_isolated

echo "source ~/Robid/devel_isolated/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc

echo "Building ORB_SLAM2"
cd ../ORB_SLAM2
chmod +x build.sh
./build.sh

echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Robid/ORB_SLAM2/Examples/ROS" >> ~/.bashrc
source ~/.bashrc
echo "Building for ROS"
chmod +x build_ros.sh
./build_ros.sh

echo

echo "Success!"
