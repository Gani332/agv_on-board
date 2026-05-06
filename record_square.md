Robot A: 10.23.63.80

Terminal 1, logging:

ssh ubuntu@10.23.63.80
cd ~/slam_project
git pull --ff-only
export REQUIRE_GT=false
export REQUIRE_IMU=true
bash scripts/logging/start_session.sh robotA scenario1_square

Terminal 2, square drive:

ssh ubuntu@10.23.63.80
cd ~/slam_project
source /opt/ros/melodic/setup.bash
source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/drive_square.py --side 0.75 --linear 0.22 --angular 0.28 --turn-deg 94 --cycles 2 --no-prompt --verbose --clockwise

Robot B: 10.23.50.211
Terminal 1, logging:

ssh ubuntu@10.23.50.211
cd ~/slam_project
git pull --ff-only
export REQUIRE_GT=false
export REQUIRE_IMU=true
bash scripts/logging/start_session.sh robotB scenario1_square

Terminal 2, square drive:

ssh ubuntu@10.23.50.211
cd ~/slam_project
source /opt/ros/melodic/setup.bash
source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/drive_square.py --side 0.75 --linear 0.22 --angular 0.28 --turn-deg 94 --cycles 2 --no-prompt --verbose --clockwise