#include "cyber/common/log.h"
/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/inference/inference.h"

namespace apollo {
namespace perception {
namespace inference {

void Inference::set_max_batch_size(const int &batch_size) {
AINFO<<"(DMCZP) EnteringMethod: Inference::set_max_batch_size";
  max_batch_size_ = batch_size;
}

void Inference::set_gpu_id(const int &gpu_id) { gpu_id_ = gpu_id; }
AINFO<<"(DMCZP) EnteringMethod: Inference::set_gpu_id";

}  // namespace inference
}  // namespace perception
}  // namespace apollo
