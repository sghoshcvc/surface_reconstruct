!/usr/bin/sh
cmake .
make
for file in /media/pragna/6F7B71345EEEFC2B/pcd2/*.pcd
do
  ./project_region_growing "$file" -dump
  ./project_octree "$file" -dump
done
