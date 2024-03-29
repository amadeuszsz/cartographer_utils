// Copyright 2024 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CARTOGRAPHER_UTILS__VISIBILITY_CONTROL_HPP_
#define CARTOGRAPHER_UTILS__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(CARTOGRAPHER_UTILS_BUILDING_DLL) || defined(CARTOGRAPHER_UTILS_EXPORTS)
    #define CARTOGRAPHER_UTILS_PUBLIC __declspec(dllexport)
    #define CARTOGRAPHER_UTILS_LOCAL
  #else  // defined(CARTOGRAPHER_UTILS_BUILDING_DLL) || defined(CARTOGRAPHER_UTILS_EXPORTS)
    #define CARTOGRAPHER_UTILS_PUBLIC __declspec(dllimport)
    #define CARTOGRAPHER_UTILS_LOCAL
  #endif  // defined(CARTOGRAPHER_UTILS_BUILDING_DLL) || defined(CARTOGRAPHER_UTILS_EXPORTS)
#elif defined(__linux__)
  #define CARTOGRAPHER_UTILS_PUBLIC __attribute__((visibility("default")))
  #define CARTOGRAPHER_UTILS_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define CARTOGRAPHER_UTILS_PUBLIC __attribute__((visibility("default")))
  #define CARTOGRAPHER_UTILS_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // CARTOGRAPHER_UTILS__VISIBILITY_CONTROL_HPP_
