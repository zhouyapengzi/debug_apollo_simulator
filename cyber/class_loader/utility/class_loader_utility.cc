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

#include "cyber/class_loader/utility/class_loader_utility.h"

#include "cyber/class_loader/class_loader.h"

namespace apollo {
namespace cyber {
namespace class_loader {
namespace utility {

std::recursive_mutex& GetClassFactoryMapMapMutex() {
AINFO<<"(DMCZP) EnteringMethod: GetClassFactoryMapMapMutex";
  static std::recursive_mutex m;
  return m;
}

std::recursive_mutex& GetLibPathPocoShareLibMutex() {
AINFO<<"(DMCZP) EnteringMethod: GetLibPathPocoShareLibMutex";
  static std::recursive_mutex m;
  return m;
}

BaseToClassFactoryMapMap& GetClassFactoryMapMap() {
AINFO<<"(DMCZP) EnteringMethod: GetClassFactoryMapMap";
  static BaseToClassFactoryMapMap instance;
  return instance;
}

LibpathPocolibVector& GetLibPathPocoShareLibVector() {
AINFO<<"(DMCZP) EnteringMethod: GetLibPathPocoShareLibVector";
  static LibpathPocolibVector instance;
  return instance;
}

ClassClassFactoryMap& GetClassFactoryMapByBaseClass(
    const std::string& typeid_base_class_name) {
AINFO<<"(DMCZP) EnteringMethod: GetClassFactoryMapByBaseClass";
  BaseToClassFactoryMapMap& factoryMapMap = GetClassFactoryMapMap();
  std::string base_class_name = typeid_base_class_name;
  if (factoryMapMap.find(base_class_name) == factoryMapMap.end()) {
    factoryMapMap[base_class_name] = ClassClassFactoryMap();
  }

  return factoryMapMap[base_class_name];
}

std::string& GetCurLoadingLibraryNameReference() {
AINFO<<"(DMCZP) EnteringMethod: GetCurLoadingLibraryNameReference";
  static std::string library_name;
  return library_name;
}

std::string GetCurLoadingLibraryName() {
AINFO<<"(DMCZP) EnteringMethod: GetCurLoadingLibraryName";
  return GetCurLoadingLibraryNameReference();
}

void SetCurLoadingLibraryName(const std::string& library_name) {
AINFO<<"(DMCZP) EnteringMethod: SetCurLoadingLibraryName";
  std::string& library_name_ref = GetCurLoadingLibraryNameReference();
  library_name_ref = library_name;
}

ClassLoader*& GetCurActiveClassLoaderReference() {
AINFO<<"(DMCZP) EnteringMethod: GetCurActiveClassLoaderReference";
  static ClassLoader* loader = nullptr;
  return loader;
}

ClassLoader* GetCurActiveClassLoader() {
AINFO<<"(DMCZP) EnteringMethod: GetCurActiveClassLoader";
  return (GetCurActiveClassLoaderReference());
}

void SetCurActiveClassLoader(ClassLoader* loader) {
AINFO<<"(DMCZP) EnteringMethod: SetCurActiveClassLoader";
  ClassLoader*& loader_ref = GetCurActiveClassLoaderReference();
  loader_ref = loader;
}

ClassFactoryVector GetAllClassFactoryObjects(
    const ClassClassFactoryMap& factories) {
AINFO<<"(DMCZP) EnteringMethod: GetAllClassFactoryObjects";
  ClassFactoryVector all_class_factory_objs;
  for (auto& class_factory : factories) {
    all_class_factory_objs.emplace_back(class_factory.second);
  }

  return all_class_factory_objs;
}

ClassFactoryVector GetAllClassFactoryObjects() {
AINFO<<"(DMCZP) EnteringMethod: GetAllClassFactoryObjects";
  std::lock_guard<std::recursive_mutex> lck(GetClassFactoryMapMapMutex());

  ClassFactoryVector all_class_factory_objs;
  BaseToClassFactoryMapMap& factory_map_map = GetClassFactoryMapMap();
  for (auto& baseclass_map : factory_map_map) {
    ClassFactoryVector objs = GetAllClassFactoryObjects(baseclass_map.second);
    all_class_factory_objs.insert(all_class_factory_objs.end(), objs.begin(),
                                  objs.end());
  }

  return all_class_factory_objs;
}

ClassFactoryVector GetAllClassFactoryObjectsOfLibrary(
    const std::string& library_path) {
AINFO<<"(DMCZP) EnteringMethod: GetAllClassFactoryObjectsOfLibrary";
  ClassFactoryVector all_class_factory_objs = GetAllClassFactoryObjects();
  ClassFactoryVector library_class_factory_objs;
AINFO<<"(DMCZP) EnteringMethod: FindLoadedLibrary";
  for (auto& class_factory_obj : all_class_factory_objs) {
    if (class_factory_obj->GetRelativeLibraryPath() == library_path) {
      library_class_factory_objs.emplace_back(class_factory_obj);
    }
  }
  return library_class_factory_objs;
}

void DestroyClassFactoryObjectsOfLibrary(
    const std::string& library_path, const ClassLoader* class_loader,
    ClassClassFactoryMap* class_factory_map) {
AINFO<<"(DMCZP) EnteringMethod: DestroyClassFactoryObjectsOfLibrary";
  for (ClassClassFactoryMap::iterator itr = class_factory_map->begin();
       itr != class_factory_map->end();) {
    AbstractClassFactoryBase* class_factory_object = itr->second;
    if (class_factory_object->GetRelativeLibraryPath() == library_path &&
        class_factory_object->IsOwnedBy(class_loader)) {
      class_factory_object->RemoveOwnedClassLoader(class_loader);
      // when no anybody owner,delete && erase
      if (!class_factory_object->IsOwnedByAnybody()) {
        class_factory_map->erase(itr++);
        delete class_factory_object;
      } else {
        ++itr;
      }
    } else {
      ++itr;
    }
  }
}

void DestroyClassFactoryObjectsOfLibrary(const std::string& library_path,
                                         const ClassLoader* loader) {
AINFO<<"(DMCZP) EnteringMethod: DestroyClassFactoryObjectsOfLibrary";
  std::lock_guard<std::recursive_mutex> lck(GetClassFactoryMapMapMutex());

  BaseToClassFactoryMapMap& factory_map_map = GetClassFactoryMapMap();
  for (auto& baseclass_map : factory_map_map) {
    DestroyClassFactoryObjectsOfLibrary(library_path, loader,
                                        &baseclass_map.second);
  }
}

LibpathPocolibVector::iterator FindLoadedLibrary(
    const std::string& library_path) {
  LibpathPocolibVector& opened_libraries = GetLibPathPocoShareLibVector();
  LibpathPocolibVector::iterator itr;
  for (itr = opened_libraries.begin(); itr != opened_libraries.end(); ++itr) {
    if (itr->first == library_path) {
      break;
    }
  }
  return itr;
}

bool IsLibraryLoadedByAnybody(const std::string& library_path) {
AINFO<<"(DMCZP) EnteringMethod: IsLibraryLoadedByAnybody";
  std::lock_guard<std::recursive_mutex> lck(GetLibPathPocoShareLibMutex());

  LibpathPocolibVector& opened_libraries = GetLibPathPocoShareLibVector();
  LibpathPocolibVector::iterator itr = FindLoadedLibrary(library_path);
  return itr != opened_libraries.end();
}

bool IsLibraryLoaded(const std::string& library_path,
                     ClassLoader* class_loader) {
AINFO<<"(DMCZP) EnteringMethod: IsLibraryLoaded";
  bool is_lib_loaded_by_anyone = IsLibraryLoadedByAnybody(library_path);
  ClassFactoryVector lib_class_factory_objs =
      GetAllClassFactoryObjectsOfLibrary(library_path);
  auto num_lib_class_factory_objs = lib_class_factory_objs.size();
  if (is_lib_loaded_by_anyone && num_lib_class_factory_objs == 0) {
    return true;
  }

  ClassFactoryVector lib_loader_class_factory_objs;
  for (auto& class_factory_obj : lib_class_factory_objs) {
    if (class_factory_obj->IsOwnedBy(class_loader)) {
      lib_loader_class_factory_objs.emplace_back(class_factory_obj);
    }
  }

  auto num_lib_loader_class_factory_objs = lib_loader_class_factory_objs.size();
  return (is_lib_loaded_by_anyone &&
          (num_lib_loader_class_factory_objs <= num_lib_class_factory_objs));
}

bool LoadLibrary(const std::string& library_path, ClassLoader* loader) {
AINFO<<"(DMCZP) EnteringMethod: LoadLibrary";
  if (IsLibraryLoadedByAnybody(library_path)) {
    AINFO << "lib has been loaded by others,only attach to class factory obj."
          << library_path;
    ClassFactoryVector lib_class_factory_objs =
        GetAllClassFactoryObjectsOfLibrary(library_path);
    for (auto& class_factory_obj : lib_class_factory_objs) {
      class_factory_obj->AddOwnedClassLoader(loader);
    }
    return true;
  }

  PocoLibraryPtr poco_library = nullptr;
  static std::recursive_mutex loader_mutex;
  {
    std::lock_guard<std::recursive_mutex> lck(loader_mutex);

    try {
      SetCurActiveClassLoader(loader);
      SetCurLoadingLibraryName(library_path);
      poco_library = PocoLibraryPtr(new Poco::SharedLibrary(library_path));
    } catch (const Poco::LibraryLoadException& e) {
      SetCurLoadingLibraryName("");
      SetCurActiveClassLoader(nullptr);
      AERROR << "poco LibraryLoadException: " << e.message();
    } catch (const Poco::LibraryAlreadyLoadedException& e) {
      SetCurLoadingLibraryName("");
      SetCurActiveClassLoader(nullptr);
      AERROR << "poco LibraryAlreadyLoadedException: " << e.message();
    } catch (const Poco::NotFoundException& e) {
      SetCurLoadingLibraryName("");
      SetCurActiveClassLoader(nullptr);
      AERROR << "poco NotFoundException: " << e.message();
    }

    SetCurLoadingLibraryName("");
    SetCurActiveClassLoader(nullptr);
  }

  if (poco_library == nullptr) {
    AERROR << "poco shared library failed: " << library_path;
    return false;
  }

  auto num_lib_objs = GetAllClassFactoryObjectsOfLibrary(library_path).size();
  if (num_lib_objs == 0) {
    AWARN << "Class factory objs counts is 0, maybe registerclass failed.";
  }

  std::lock_guard<std::recursive_mutex> lck(GetLibPathPocoShareLibMutex());
  LibpathPocolibVector& opened_libraries = GetLibPathPocoShareLibVector();
  opened_libraries.emplace_back(
      std::pair<std::string, PocoLibraryPtr>(library_path, poco_library));
  return true;
}

void UnloadLibrary(const std::string& library_path, ClassLoader* loader) {
AINFO<<"(DMCZP) EnteringMethod: UnloadLibrary";
  {
    std::lock_guard<std::recursive_mutex> lck(GetLibPathPocoShareLibMutex());
    LibpathPocolibVector& opened_libraries = GetLibPathPocoShareLibVector();
    LibpathPocolibVector::iterator itr = FindLoadedLibrary(library_path);
    if (itr == opened_libraries.end()) {
      AERROR << "Attempt to UnloadLibrary lib, but can't find lib: "
             << library_path;
      return;
    }

    std::string library_path = itr->first;
    try {
      DestroyClassFactoryObjectsOfLibrary(library_path, loader);

      if (GetAllClassFactoryObjectsOfLibrary(library_path).empty()) {
        itr->second->unload();
        itr = opened_libraries.erase(itr);
      } else {
        AWARN << "ClassFactory Objects still remain in memory,meaning other "
                 "ClassLoaders are still using library:"
              << library_path;
      }
    } catch (const Poco::RuntimeException& e) {
      AERROR << "library unLoad error: Poco::RuntimeException: " << e.message();
    }
  }
}

}  // End namespace utility
}  // End namespace class_loader
}  // namespace cyber
}  // namespace apollo
