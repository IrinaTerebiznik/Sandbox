## Primera vez corriendo el docker:
'''bash
chmod +x build.sh
./build.sh
'''


docker run -it --rm \
  --privileged \
  -v /dev:/dev \
  -v /sys:/sys \
  ros-ciie