roscore &

gnome-terminal --window --geometry=80x25+10+10 --title="tf_broadcaster" -- bash -c "sleep 2s;rosrun tfexample tf_broadcaster;exec bash;" &
gnome-terminal --window --geometry=80x25+1250+10 --title="pointpipe" -- bash -c "sleep 2s;rosrun laser_geometry pointpipe;exec bash;" &
gnome-terminal --window --geometry=80x25+1250+400 --title="pointreceiver" -- bash -c "sleep 2s;rosrun pointcloud pointreceiver;exec bash;" &
gnome-terminal --window --geometry=80x25+1250+700 --title="pointhandle" -- bash -c "sleep 2s;rosrun pointcloud pointhandle;exec bash;" &
gnome-terminal --window --geometry=80x25+10+400 -- bash -c "sleep 2s;rosrun rqt_graph rqt_graph;exec bash;" &
gnome-terminal --window --geometry=80x25+10+700 -- bash -c "sleep 2s;rosrun rviz rviz;exec bash;" &

wait
echo "done"
