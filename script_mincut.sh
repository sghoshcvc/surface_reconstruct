!/usr/bin/sh
cmake .
make
for file in /home/pragna/recorded_pcd/*.pcd
do
  ./min_cut_seg "$file"
done
