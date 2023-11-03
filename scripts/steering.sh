#####################################################
# File Name: 1.sh
# Author: Leo
# mail: yli97@jmc.com.cn
# Created Time: 二  6/22 13:49:58 2021
#####################################################

#!/bin/bash

grep -E 'steering angle command' $1 >> steering_angle.csv
cat steering_angle.csv |awk '{print $9}' >> steering_angle1.csv
mv steering_angle1.csv steering_angle.csv
sed '/^[  ]*$/d' steering_angle.csv
