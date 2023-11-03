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

/**
 * @file localization.h
 * @brief The class of Localization
 */

#ifndef MODULES_LOCALIZATION_LOCALIZATION_H_
#define MODULES_LOCALIZATION_LOCALIZATION_H_

#include <memory>
#include <string>

#include "modules/localization/proto/localization_config.pb.h"

#include "modules/common/jmc_auto_app.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/localization/localization_base.h"

/**
 * @namespace jmc_auto::localization
 * @brief jmc_auto::localization
 */
namespace jmc_auto {
namespace localization {

/**
 * @class Localization
 *
 * @brief Localization module main class. It processes GPS and IMU as input,
 * to generate localization info.
 */
class Localization : public jmc_auto::common::JmcAutoApp {
 public:
  /**
   * @brief module name
   * @return module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  jmc_auto::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  jmc_auto::common::Status Start() override;

  /**
   * @brief module stop function
   * @return stop status
   */
  void Stop() override;
  

 private:
  void RegisterLocalizationMethods();

  std::unique_ptr<LocalizationBase> localization_;
  jmc_auto::common::util::Factory<LocalizationConfig::LocalizationType,
                                LocalizationBase>
      localization_factory_;
  LocalizationConfig config_;
};

}  // namespace localization
}  // namespace jmc_auto

#endif  // MODULES_LOCALIZATION_LOCALIZATION_H_
