/******************************************************************************
 * Copyright 2017 The JmcAuto Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "velodyne_pointcloud/pointcloud_dump.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_dump");

  ros::NodeHandle hn("~");
  ros::NodeHandle n;

  jmc_auto::drivers::velodyne::PointCloudDump pc_dump(n, hn);
  ros::spin();

  return 0;
}
