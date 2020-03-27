#include "cyber/common/log.h"
/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/common/prediction_thread_pool.h"

namespace apollo {
namespace prediction {

thread_local int PredictionThreadPool::s_thread_pool_level = 0;
std::vector<int> BaseThreadPool::THREAD_POOL_CAPACITY = {20, 20, 20};

BaseThreadPool::BaseThreadPool(int thread_num, int next_thread_pool_level)
    : stopped_(false) {
  AINFO<<"(DMCZP) EnteringMethod: BaseThreadPool::BaseThreadPool";

  if (!task_queue_.Init(thread_num,
                        new apollo::cyber::base::BlockWaitStrategy())) {
    throw std::runtime_error("Task queue init failed.");
  }
  for (int i = 0; i < thread_num; ++i) {
    workers_.emplace_back([this, next_thread_pool_level, i] {
      PredictionThreadPool::s_thread_pool_level = next_thread_pool_level;
      while (!stopped_) {
        std::function<void()> task;
        if (task_queue_.WaitDequeue(&task)) {
          task();
        }
      }
    });
  }

  AINFO<<"(DMCZP) LeaveMethod: BaseThreadPool::BaseThreadPool";
 }

void BaseThreadPool::Stop() {
  AINFO<<"(DMCZP) EnteringMethod: BaseThreadPool::Stop";

  task_queue_.BreakAllWait();
  for (std::thread& worker : workers_) {
    worker.join();
  }
  stopped_ = true;

  AINFO<<"(DMCZP) LeaveMethod: BaseThreadPool::Stop";
 }

BaseThreadPool::~BaseThreadPool() {
  if (stopped_.exchange(true)) {
    return;
  }
  task_queue_.BreakAllWait();
  for (std::thread& worker : workers_) {
    worker.join();
  }
}

BaseThreadPool* PredictionThreadPool::Instance() {
  AINFO<<"(DMCZP) EnteringMethod: PredictionThreadPool::Instance";

  int pool_level = s_thread_pool_level;
  int max_thread_pool_level =
      static_cast<int>(BaseThreadPool::THREAD_POOL_CAPACITY.size());
  CHECK_LT(pool_level, max_thread_pool_level);
  int index = s_thread_pool_level;
  switch (index) {
    case 0: {
      
  AINFO<<"(DMCZP) (return) LeaveMethod: PredictionThreadPool::Instance";
  return LevelThreadPool<0>::Instance();
    }
    case 1: {
      
  AINFO<<"(DMCZP) (return) LeaveMethod: PredictionThreadPool::Instance";
  return LevelThreadPool<1>::Instance();
    }
    case 2: {
      
  AINFO<<"(DMCZP) (return) LeaveMethod: PredictionThreadPool::Instance";
  return LevelThreadPool<2>::Instance();
    }
  }
  AERROR << "Should not hit here";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PredictionThreadPool::Instance";
  return LevelThreadPool<0>::Instance();

 }

}  // namespace prediction
}  // namespace apollo
