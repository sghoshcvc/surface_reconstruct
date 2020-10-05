!/usr/bin/sh
cmake .
make
for file in /home/pragna/recorded_pcd_fg/*.pcd
do
  ./project_region_growing "$file" -dump
done
