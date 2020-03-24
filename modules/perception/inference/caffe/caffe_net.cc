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

#include "modules/perception/inference/caffe/caffe_net.h"

#include "cyber/common/log.h"
#include<thread>

namespace apollo {
namespace perception {
namespace inference {

CaffeNet::CaffeNet(const std::string &net_file, const std::string &model_file,
                   const std::vector<std::string> &outputs)
    : net_file_(net_file), model_file_(model_file), output_names_(outputs) {
  AINFO<<"(DMCZP) EnteringMethod: CaffeNet::CaffeNet";

  AINFO<<"(DMCZP) LeaveMethod: CaffeNet::CaffeNet";
 
  AINFO<<"(DMCZP) EnteringMethod: CaffeNet::CaffeNet";

  AINFO<<"(DMCZP) LeaveMethod: CaffeNet::CaffeNet";
 }

bool CaffeNet::Init(const std::map<std::string, std::vector<int>> &shapes) {
AINFO<<"(DMCZP) EnteringMethod: CaffeNet::Init";

  AINFO << "(pengzi)in method: CaffeNet::Init(const std::map<std::string, std::vector<int>> &shapes). thread:" << std::this_thread::get_id();

  if (gpu_id_ >= 0) {
    caffe::Caffe::SetDevice(gpu_id_);
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
    caffe::Caffe::DeviceQuery();
  } else {
    caffe::Caffe::set_mode(caffe::Caffe::CPU);
  }

  // init Net
  net_.reset(new caffe::Net<float>(net_file_, caffe::TEST));
  if (net_ == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: CaffeNet::Init";
  return false;
  }
  net_->CopyTrainedLayersFrom(model_file_);
  for (auto tmp : shapes) {
    auto blob = net_->blob_by_name(tmp.first);
    if (blob != nullptr) {
      blob->Reshape(tmp.second);
    }
  }
  net_->Reshape();
  for (auto name : output_names_) {
    auto caffe_blob = net_->blob_by_name(name);
    if (caffe_blob == nullptr) {
      continue;
    }
    std::shared_ptr<apollo::perception::base::Blob<float>> blob;
    blob.reset(new apollo::perception::base::Blob<float>(caffe_blob->shape()));
    blobs_.insert(std::make_pair(name, blob));
  }
  for (auto name : input_names_) {
    auto caffe_blob = net_->blob_by_name(name);
    if (caffe_blob == nullptr) {
      continue;
    }
    std::shared_ptr<apollo::perception::base::Blob<float>> blob;
    blob.reset(new apollo::perception::base::Blob<float>(caffe_blob->shape()));
    blobs_.insert(std::make_pair(name, blob));
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: CaffeNet::Init";
  return true;
}

CaffeNet::CaffeNet(const std::string &net_file, const std::string &model_file,
                   const std::vector<std::string> &outputs,
                   const std::vector<std::string> &inputs)
    : net_file_(net_file),
      model_file_(model_file),
      output_names_(outputs),
      input_names_(inputs) {


AINFO << "(pengzi)CaffeNet. model_file:"<<&model_file
      << " thread:" << std::this_thread::get_id();

        
      }

std::shared_ptr<apollo::perception::base::Blob<float>> CaffeNet::get_blob(
    const std::string &name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

bool CaffeNet::reshape() {
AINFO<<"(DMCZP) EnteringMethod: CaffeNet::reshape";
  for (auto name : input_names_) {
    auto blob = this->get_blob(name);
    auto caffe_blob = net_->blob_by_name(name);
    if (caffe_blob != nullptr && blob != nullptr) {
      caffe_blob->Reshape(blob->shape());
      cudaMemcpy(caffe_blob->mutable_gpu_data(), blob->gpu_data(),
                 caffe_blob->count() * sizeof(float), cudaMemcpyDeviceToDevice);
    }
  }
  net_->Reshape();

  
  AINFO<<"(DMCZP) (return) LeaveMethod: CaffeNet::reshape";
  return true;
}

void CaffeNet::Infer() {
AINFO<<"(DMCZP) EnteringMethod: CaffeNet::Infer";

  AINFO << "(pengzi)begin Caffe net infer. in method: CaffeNet::Infer()  thread:" << std::this_thread::get_id();

  if (gpu_id_ >= 0) {
    caffe::Caffe::SetDevice(gpu_id_);
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
  }
  this->reshape();
  // If `out_blob->mutable_cpu_data()` is invoked outside,
  // HEAD will be set to CPU, and `out_blob->mutable_gpu_data()`
  // after `enqueue` will copy data from CPU to GPU,
  // which will overwrite the `inference` results.
  // `out_blob->gpu_data()` will set HEAD to SYNCED,
  // then no copy happends after `enqueue`.
  for (auto name : output_names_) {
    auto blob = get_blob(name);
    if (blob != nullptr) {
      blob->gpu_data();
    }
  }

  net_->Forward();
  for (auto name : output_names_) {
    auto blob = get_blob(name);
    auto caffe_blob = net_->blob_by_name(name);
    if (caffe_blob != nullptr && blob != nullptr) {
      blob->Reshape(caffe_blob->shape());
      cudaMemcpy(blob->mutable_gpu_data(), caffe_blob->gpu_data(),
                 caffe_blob->count() * sizeof(float), cudaMemcpyDeviceToDevice);
    }
  }


   /* pengzi add code here*/
//  neuron_coverage(net_);
   AINFO<<"( pengzipengzi) Calculate Neuron Coverage Here!!!!!"; 
   /* pengzi add code finish here*/

  AINFO << "(pengzi)finish Caffe net infer. thread:" << std::this_thread::get_id();

  AINFO<<"(DMCZP) LeaveMethod: CaffeNet::Infer";
 }




/* add function 
void neuron_coverage(std::shared_ptr<caffe::Net<float>> net_){
  AINFO << "(pengzi) calculate neuron_coverage";
  //std::vector<std::vector<Blob<Dtype>*>>  top_vec = net_->top_vecs_ ;
  std::cout << net_->top_vecs() .size() << ' ';
  AINFO << "(pengzi) total number of layers in DAG: " << net_->top_vecs() .size();
  int total_neuron = 0;
  for (std::vector<std::vector<caffe::Blob<Dtype>*>>::const_iterator i = net_->top_vecs().begin(); i != net_->top_vecs() .end(); ++i){
    AINFO<<"(pengzi) number of neurons in one layer: "<< *i.size();
    total_neuron = total_neuron + *i.size();
    for(std::vector<caffe::Blob<Dtype>*>::const_iterator j = *i.begin(); j!= *i.end(); ++j){
        std::cout << *j << ' ';       
        AINFO << "(pengzi) neuron:" << *j;
    }
  }
  AINFO << "(pengzi) Total neurons: " << total_neuron;
}
finish add function */




bool CaffeNet::shape(const std::string &name, std::vector<int> *res) {
AINFO<<"(DMCZP) EnteringMethod: CaffeNet::shape";
  AINFO << "(pengzi)in method: CaffeNet::shape(const std::string &name, std::vector<int> *res) thread:" << std::this_thread::get_id();

  auto blob = net_->blob_by_name(name);
  if (blob == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: CaffeNet::shape";
  return false;
  }
  *res = blob->shape();
  
  AINFO<<"(DMCZP) (return) LeaveMethod: CaffeNet::shape";
  return true;
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
