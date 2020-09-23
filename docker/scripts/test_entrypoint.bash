#!/bin/bash

mkdir -p ~/.aws

echo "[default]
region = ${AWS_REGION}
output = ${AWS_OUTPUT_FORMAT}" > ~/.aws/config
echo "[default]
aws_access_key_id = ${AWS_SECRET_KEY_ID}
aws_secret_access_key = ${AWS_SECRET_KEY}" > ~/.aws/credentials

source /root/catkin_ws/devel/setup.bash
rostest cordial_gui test_cordial_gui_actions.test && rostest cordial_gui test_cordial_gui_pubs_and_subs.test && rostest cordial_manager test_cordial_manager_actions.test && rostest cordial_manager test_cordial_manager_pubs_and_subs.test && rostest cordial_manager test_cordial_manager_services.test

