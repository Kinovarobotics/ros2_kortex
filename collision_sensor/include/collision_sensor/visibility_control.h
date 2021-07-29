// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Subhas Das, Denis Stogl
 */

#ifndef COLLISION_SENSOR__VISIBILITY_CONTROL_H_
#define COLLISION_SENSOR__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COLLISION_SENSOR_EXPORT __attribute__((dllexport))
#define COLLISION_SENSOR_IMPORT __attribute__((dllimport))
#else
#define COLLISION_SENSOR_EXPORT __declspec(dllexport)
#define COLLISION_SENSOR_IMPORT __declspec(dllimport)
#endif
#ifdef COLLISION_SENSOR_BUILDING_DLL
#define COLLISION_SENSOR_PUBLIC COLLISION_SENSOR_EXPORT
#else
#define COLLISION_SENSOR_PUBLIC COLLISION_SENSOR_IMPORT
#endif
#define COLLISION_SENSOR_PUBLIC_TYPE COLLISION_SENSOR_PUBLIC
#define COLLISION_SENSOR_LOCAL
#else
#define COLLISION_SENSOR_EXPORT __attribute__((visibility("default")))
#define COLLISION_SENSOR_IMPORT
#if __GNUC__ >= 4
#define COLLISION_SENSOR_PUBLIC __attribute__((visibility("default")))
#define COLLISION_SENSOR_LOCAL __attribute__((visibility("hidden")))
#else
#define COLLISION_SENSOR_PUBLIC
#define COLLISION_SENSOR_LOCAL
#endif
#define COLLISION_SENSOR_PUBLIC_TYPE
#endif

#endif  // COLLISION_SENSOR__VISIBILITY_CONTROL_H_
