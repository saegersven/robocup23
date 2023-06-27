#clear # clears terminal
python3 /home/pi/robocup23/stop.py # stops motors from simple python script
cd /home/pi/robocup23/build
if ninja -j4 ;
then
  /home/pi/robocup23/build/robocup ;
fi
cd ..
