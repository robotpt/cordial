FROM exzhou/minimal_cordial_setup:latest

COPY . /root/catkin_ws/src/cordial

RUN \
	cd /root/catkin_ws \
	&& /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"

RUN chmod +x /root/catkin_ws/src/cordial/run_tests.sh

ENTRYPOINT ["/root/catkin_ws/src/cordial/run_tests.sh"]