# Installation / Deps

sudo apt-get install build-essential cmake libboost-all-dev libprotoc-dev protobuf-compiler libssl-dev mplayer

# Run benchmarking box

roslaunch vicon_bridge vicon.launch

roslaunch roah_rsbb roah_rsbb.launch rsbb_host:=192.168.0.255

## Only for FBM2:
roslaunch rockin_scoring fbm2h_vicon.launch team_name:=identity