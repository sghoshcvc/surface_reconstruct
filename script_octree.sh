!/usr/bin/sh
cmake .
make
for file in /home/pragna/Documents/Documents/New_Pcd/*.pcd
do
  ./project_octree "$file" -dump
done
