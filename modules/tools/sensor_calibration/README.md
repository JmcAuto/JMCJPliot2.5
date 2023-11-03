# Sensor Calibration

## ins_stat_publisher.py

This tool is used to publish a fake "/jmc_auto/sensor/gnss/ins_stat"

Run the following command from your JmcAuto root dir:

```
python modules/tools/sensor_calibration/ins_stat_publisher.py
```

## odom_publisher.py

This tool is used to publish topic "/jmc_auto/sensor/gnss/odometry" through subscribe topic "/jmc_auto/localization/pose" 

Run the following command from your JmcAuto root dir:

```
python modules/tools/sensor_calibration/odom_publisher.py
```

