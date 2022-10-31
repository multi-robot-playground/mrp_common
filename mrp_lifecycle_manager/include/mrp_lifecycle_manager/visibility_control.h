#ifndef MULTI_ROBOT_LIFECYCLE_MANAGER__VISIBILITY_CONTROL_H_
#define MULTI_ROBOT_LIFECYCLE_MANAGER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MULTI_ROBOT_LIFECYCLE_MANAGER_EXPORT __attribute__ ((dllexport))
    #define MULTI_ROBOT_LIFECYCLE_MANAGER_IMPORT __attribute__ ((dllimport))
  #else
    #define MULTI_ROBOT_LIFECYCLE_MANAGER_EXPORT __declspec(dllexport)
    #define MULTI_ROBOT_LIFECYCLE_MANAGER_IMPORT __declspec(dllimport)
  #endif
  #ifdef MULTI_ROBOT_LIFECYCLE_MANAGER_BUILDING_LIBRARY
    #define MULTI_ROBOT_LIFECYCLE_MANAGER_PUBLIC MULTI_ROBOT_LIFECYCLE_MANAGER_EXPORT
  #else
    #define MULTI_ROBOT_LIFECYCLE_MANAGER_PUBLIC MULTI_ROBOT_LIFECYCLE_MANAGER_IMPORT
  #endif
  #define MULTI_ROBOT_LIFECYCLE_MANAGER_PUBLIC_TYPE MULTI_ROBOT_LIFECYCLE_MANAGER_PUBLIC
  #define MULTI_ROBOT_LIFECYCLE_MANAGER_LOCAL
#else
  #define MULTI_ROBOT_LIFECYCLE_MANAGER_EXPORT __attribute__ ((visibility("default")))
  #define MULTI_ROBOT_LIFECYCLE_MANAGER_IMPORT
  #if __GNUC__ >= 4
    #define MULTI_ROBOT_LIFECYCLE_MANAGER_PUBLIC __attribute__ ((visibility("default")))
    #define MULTI_ROBOT_LIFECYCLE_MANAGER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MULTI_ROBOT_LIFECYCLE_MANAGER_PUBLIC
    #define MULTI_ROBOT_LIFECYCLE_MANAGER_LOCAL
  #endif
  #define MULTI_ROBOT_LIFECYCLE_MANAGER_PUBLIC_TYPE
#endif

#endif  // MULTI_ROBOT_LIFECYCLE_MANAGER__VISIBILITY_CONTROL_H_
