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
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers

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

# echo "source ~/Robid/devel_isolated/setup.bash" >> ~/.bashrc
# echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc

echo "Building ORB_SLAM2"
cd ../ORB_SLAM2
chmod +x build.sh
./build.sh

# echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Robid/ORB_SLAM2/Examples/ROS" >> ~/.bashrc
source ~/.bashrc
echo "Building for ROS"
chmod +x build_ros.sh
./build_ros.sh

echo

echo "Success!"
