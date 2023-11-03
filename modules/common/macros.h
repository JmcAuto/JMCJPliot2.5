/******************************************************************************
 * Copyright 2017 The JmcAuto Authors. All Rights Reserved.
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
 * @file
 */

#ifndef MODULES_COMMON_MACROS_H_
#define MODULES_COMMON_MACROS_H_

#include <iostream>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>
#include <cstdlib>
#include <new>

#define CACHELINE_SIZE 64

#define DEFINE_TYPE_TRAIT(name, func)                     \
  template <typename T>                                   \
  struct name {                                           \
    template <typename Class>                             \
    static constexpr bool Test(decltype(&Class::func)*) { \
      return true;                                        \
    }                                                     \
    template <typename>                                   \
    static constexpr bool Test(...) {                     \
      return false;                                       \
    }                                                     \
                                                          \
    static constexpr bool value = Test<T>(nullptr);       \
  };                                                      \
                                                          \
  template <typename T>                                   \
  constexpr bool name<T>::value;


inline void* CheckedMalloc(size_t size) {
  void* ptr = std::malloc(size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}

inline void* CheckedCalloc(size_t num, size_t size) {
  void* ptr = std::calloc(num, size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}


DEFINE_TYPE_TRAIT(HasShutdown, Shutdown)

template <typename T>
typename std::enable_if<HasShutdown<T>::value>::type CallShutdown(T *instance) {
  instance->Shutdown();
}

template <typename T>
typename std::enable_if<!HasShutdown<T>::value>::type CallShutdown(
    T *instance) {
  (void)instance;
}


#undef UNUSED
#undef DISALLOW_COPY_AND_ASSIGN_PTR

#define UNUSED(param) (void)param

#define DISALLOW_COPY_AND_ASSIGN_PTR(classname) \
 private:                                   \
  classname(const classname &);             \
  classname &operator=(const classname &);

#define DECLARE_SINGLETON_PTR(classname)        \
 public:                                    \
  static classname *Instance(bool create_if_needed = true) {              \
    static classname *instance = nullptr;                                 \
    if (!instance && create_if_needed) {                                  \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                     [&] { instance = new (std::nothrow) classname(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
  static void CleanUp() {                   \
   auto instance = Instance(false);         \
    if (instance != nullptr) {              \
      CallShutdown(instance);               \
    }                                       \
  }                                         \
                                            \
 private:                                   \
  classname();                              \
  DISALLOW_COPY_AND_ASSIGN_PTR(classname)

#endif  // MODULES_COMMON_MACROS_H_
