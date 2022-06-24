#sudo chmod 777 /dev/ttyUSB0
#sudo chmod 777 /dev/ttyUSB1
sudo chmod 777 /dev/ttyUSB_radarCfg
sudo chmod 777 /dev/ttyUSB_radarData

roslaunch aev_pkg start_all.launch
