#!/bin/bash
gnome-terminal --geometry 50x10+10+10 -- bash camA_Estimation.sh & sleep 1
gnome-terminal --geometry 50x10+10+230 -- bash camB_Estimation.sh & sleep 1
gnome-terminal --geometry 50x10+10+450 -- bash camC_Estimation.sh & sleep 1
gnome-terminal --geometry 50x10+10+670 -- bash camD_Estimation.sh & sleep 1
gnome-terminal --geometry 50x5+10+890 -- bash base2servogp_broadcaster.sh & sleep 1
gnome-terminal --geometry 50x20+530+10 -- bash multi_cam_cooperation.sh & sleep 1

