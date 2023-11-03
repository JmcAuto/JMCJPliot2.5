/******************************************************************************
 * Copyright 2017 The jmc_auto Authors. All Rights Reserved.
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

#include "modules/localization/localization.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/localization/common/localization_gflags.h"
// #ifdef __x86_64__
// #include "modules/localization/msf/msf_localization.h"
// #endif
#include "modules/localization/rtk/rtk_localization.h"
//#include "modules/localization/vins/vins_localization.h"
//#include "modules/localization/rtk_m2/rtk_localization.h"

namespace jmc_auto {
namespace localization {

using jmc_auto::common::ErrorCode;
using jmc_auto::common::Status;


std::string Localization::Name() const {
  return FLAGS_localization_module_name;
}

void Localization::RegisterLocalizationMethods() {
  localization_factory_.Register(
      LocalizationConfig::RTK,
      []() -> LocalizationBase* { return new RTKLocalization(); });

// #ifdef __x86_64__
//   localization_factory_.Register(
//       LocalizationConfig::MSF,
//       []() -> LocalizationBase* { return new MSFLocalization(); });
// #endif
//  localization_factory_.Register(
//      LocalizationConfig::VINS,
//      []() -> LocalizationBase* { return new VINSLocalization(); });

//  localization_factory_.Register(
//      LocalizationConfig::RTKM2,
//      []() -> LocalizationBase* { return new RTKM2Localization(); });

}

Status Localization::Init() {
  RegisterLocalizationMethods();
  if (!jmc_auto::common::util::GetProtoFromFile(FLAGS_localization_config_file,
                                              &config_)) {
    AERROR << "failed to load localization config file "
           << FLAGS_localization_config_file;
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "failed to load localization config file: " +
                      FLAGS_localization_config_file);
  }

  return Status::OK();
}

Status Localization::Start() {
  localization_ =
      localization_factory_.CreateObject(config_.localization_type());
  if (!localization_) {
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "localization is not initialized with config : " +
                      config_.DebugString());
  }
  localization_->Start();

  return Status::OK();
}

void Localization::Stop() { localization_->Stop(); }

}  // namespace localization
}  // namespace jmc_auto
