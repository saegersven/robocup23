python3 stop.py # stops motors from simple python script
cd build
if ninja -j2 ;
then
  ./robocup ;
fi
cd ..
