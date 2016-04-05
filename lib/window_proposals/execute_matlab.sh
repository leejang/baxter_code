#/bin/sh
matlab -nodisplay -nojvm -r '
run /home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/lib/window_proposals/genWindowProposals(1000); exit;'
