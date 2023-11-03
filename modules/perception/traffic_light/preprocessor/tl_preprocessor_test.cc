/******************************************************************************
 * Copyright 2018 The JmcAuto Authors. All Rights Reserved.
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
#include "modules/perception/traffic_light/preprocessor/tl_preprocessor.h"

#include "gtest/gtest.h"

#include "modules/perception/traffic_light/projection/projection.h"

namespace jmc_auto {
namespace perception {
namespace traffic_light {

TEST(MultiCameraProjectionTest, load_proto) {
  RegisterFactoryBoundaryProjection();
  TLPreprocessor tlp;
  EXPECT_TRUE(tlp.Init());
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace jmc_auto
