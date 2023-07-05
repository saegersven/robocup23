rm -r build
mkdir -p build/default
cd build/default
cmake -GNinja ../..
ninja
./capture_from_camera
