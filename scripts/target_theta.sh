#name: target_theta.sh
# Author: Leo
# mail: yli97@jmc.com.cn
# Created Time: 二  6/22 13:49:58 2021
#####################################################

#!/bin/bash

#grep -E 'lat_controller.cc:625' $1 >> target_x.csv
#cat target_x.csv |awk '{print $11}' >> target_x1.csv
#mv target_x1.csv target_x.csv
#sed '/^[  ]*$/d' target_x.csv

#grep -E -w 'lat_controller.cc:625' $1 >> target_y.csv
#cat target_y.csv |awk '{print $13}' >> target_y1.csv
#mv target_y1.csv target_y.csv
#sed '/^[  ]*$/d' target_y.csv
grep -E -w 'lat_controller.cc:642' $1 >> target_theta.csv
cat target_theta.csv |awk '{print $10}' >> target_theta1.csv
mv target_theta1.csv target_theta.csv
sed '/^[  ]*$/d' target_theta.csv

grep -E -w 'lat_controller.cc:642' $1 >> theta.csv
cat theta.csv |awk '{print $7}' >> theta1.csv
mv theta1.csv theta.csv
sed '/^[  ]*$/d' theta.csv

