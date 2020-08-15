FROM exzhou/minimal_cordial_setup:cordial

COPY run_tests.sh /root/catkin_ws/src

RUN \
        cd /root/catkin_ws \
        && /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"

RUN chmod +x /root/catkin_ws/src/run_tests.sh

ENTRYPOINT ["/root/catkin_ws/src/run_tests.sh"]
