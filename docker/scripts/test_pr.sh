#!/bin/sh

/bin/bash -c "source /root/catkin_ws/devel/setup.bash && rostest cordial_gui test_cordial_gui_actions.test && rostest cordial_gui test_cordial_gui_pubs_and_subs.test && rostest cordial_manager test_cordial_manager_pubs_and_subs.test"
