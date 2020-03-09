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

#include "modules/perception/common/graph/disjoint_set.h"

namespace apollo {
namespace perception {
namespace common {

Universe::Universe(const int elements_num)
    : elts_(elements_num), sets_num_(elements_num) {
AINFO<<"(DMCZP) EnteringMethod: Universe::Universe";
  for (int i = 0; i < elements_num; ++i) {
    elts_[i].rank = 0;
    elts_[i].size = 1;
    elts_[i].p = i;
  }

  AINFO<<"(DMCZP) LeaveMethod: Universe::Universe";
 }

void Universe::Reset(const int elements_num) {
AINFO<<"(DMCZP) EnteringMethod: Universe::Reset";
  sets_num_ = elements_num;
  elts_.resize(elements_num);
  for (int i = 0; i < elements_num; ++i) {
    elts_[i].rank = 0;
    elts_[i].size = 1;
    elts_[i].p = i;
  }

  AINFO<<"(DMCZP) LeaveMethod: Universe::Reset";
 }

int Universe::Find(const int x) {
AINFO<<"(DMCZP) EnteringMethod: Universe::Find";
  int y = x;
  while (y != elts_[y].p) {
    y = elts_[y].p;
  }
  int w = x;
  while (true) {
    const int z = elts_[w].p;
    if (z == w) {
      break;
    }
    elts_[w].p = y;
    w = z;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: Universe::Find";
  return y;
}

void Universe::Join(const int x, const int y) {
AINFO<<"(DMCZP) EnteringMethod: Universe::Join";
  if (elts_[x].rank > elts_[y].rank) {
    elts_[y].p = x;
    elts_[x].size += elts_[y].size;
  } else {
    elts_[x].p = y;
    elts_[y].size += elts_[x].size;
    if (elts_[x].rank == elts_[y].rank) {
      ++elts_[y].rank;
    }
  }
  --sets_num_;

  AINFO<<"(DMCZP) LeaveMethod: Universe::Join";
 }

}  // namespace common
}  // namespace perception
}  // namespace apollo
