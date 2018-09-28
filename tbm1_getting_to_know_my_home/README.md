# Getting to know my home

Dependencies 

sudo pip install SpeechRecognition

sudo apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0

sudo apt-get install ffmpeg libav-tools

Install pyaudio from https://pypi.python.org/pypi/PyAudio v2.11 $ sudo python setup.py install

sudo pip install textrazor

sudo pip install google-api-python-client

sudo apt-get install ros-kinetic-sound-play

catkin_make -DCATKIN_BLACKLIST_PACKAGES="hearts_face_uniform"

roslaunch tbm1_getting_to_know_my_home know_my_home.launch



# Setting up the map 

     create a map:
        roslaunch hearts_navigation hearts_navigation_map_begin.launch
       edit map_name parameter in hearts_navigation_map_end.launch (optional)
        roslaunch hearts_navigation hearts_navigation_map_end.launch
       edit map_name parameter in hearts_navigation_navigate.launch to match hearts_navigation_map_end.launch (TODO: remove parameter from know_my_home.launch) 
    
     input coordinates of points of interest to locations.json:
        rosrun hearts_navigation qeconverter.py
        roslaunch hearts_navigation hearts_navigation_navigate.launch
        set 2D pose estimate
       ensure joystick has control of TIAGo, then set 2D nav goal at point of interest
       copy coordinates into locations.json
       repeat steps 4 and 5 for all points of interest 
    set benchmarking box parameters, edit rsbb_key and rsbb_host parameters of roah_rsbb_comm node in know_my_home.launch 
    run launch file:
        roslaunch tbm1_getting_to_know_my_home know_my_home.launch 

