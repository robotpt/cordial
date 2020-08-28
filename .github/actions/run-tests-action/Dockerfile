FROM ros:kinetic-ros-base-xenial

# Setup new ROS key + install ros packages
RUN \
	apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116 \
	&& apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
	&& apt-get update -y && apt-get upgrade -y \
	&& apt-get install -y \
		ros-$ROS_DISTRO-ros-control \
		ros-$ROS_DISTRO-ros-controllers \
		ros-$ROS_DISTRO-gazebo-ros-pkgs \
		ros-$ROS_DISTRO-gazebo-ros-control \
		ros-$ROS_DISTRO-rosbridge-server \
		ros-$ROS_DISTRO-audio-common \
		ros-$ROS_DISTRO-mongodb-store \
		ros-$ROS_DISTRO-mongodb-log \
	&& rm -rf /var/lib/apt/lists/*

# Setup Python2 for cordial
RUN \
	apt-get update -y && apt-get upgrade -y \
	&& apt-get install -y \
		alsa \
		gstreamer1.0-alsa \
		vorbis-tools \
	&& apt-get install -y \
		python \
		python-pip \
		python-scipy \
		python-gst0.10 \
		python-mock \
		python-pyaudio \
	&& python2 -m pip install --upgrade pip \
	&& python2 -m pip install \
		rospkg \
		catkin_pkg \
		boto3==1.12.11 \
		soundfile==0.10.3.post1 \
		speedtest-cli==2.1.2 \
	&& rm -rf /var/lib/apt/lists/*

# Setup HTTP server
RUN \
	apt-get update -y && apt-get upgrade -y \
	&& apt-get install -y \
		nodejs \
		npm \
	&& npm install http-server -g \
	&& ln -s /usr/bin/nodejs /usr/bin/node \
	&& rm -rf /var/lib/apt/lists/*

# Setup AWS credentials
RUN \
	apt-get update -y && apt-get upgrade -y \
	&& apt-get install -y awscli \
	&& rm -rf /var/lib/apt/lists/*

RUN \
	mkdir -p /root/catkin_ws/src \
	&& cd /root/catkin_ws/src \
	&& git clone https://github.com/RobotWebTools/rosbridge_suite \
	&& git clone https://github.com/robotpt/cordial.git

COPY run_tests.sh /root/catkin_ws/src

RUN \
        cd /root/catkin_ws \
        && /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"

RUN chmod +x /root/catkin_ws/src/run_tests.sh

ENTRYPOINT ["/root/catkin_ws/src/run_tests.sh"]
