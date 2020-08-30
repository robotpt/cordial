#!/bin/sh

mkdir ~/.aws
/bin/bash -c "echo '[default]
region = $1
output = $2' > ~/.aws/config" \
    && /bin/bash -c "echo '[default]
aws_access_key_id = $3
aws_secret_access_key = $4' > ~/.aws/credentials"

/bin/bash -c "source /root/catkin_ws/devel/setup.bash && rostest cordial_gui test_cordial_gui_actions.test && rostest cordial_gui test_cordial_gui_pubs_and_subs.test && rostest cordial_manager test_cordial_manager_actions.test && rostest cordial_manager test_cordial_manager_pubs_and_subs.test && rostest cordial_manager test_cordial_manager_services.test"
