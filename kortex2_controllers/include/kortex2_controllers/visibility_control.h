#ifndef KORTEX2_CONTROLLERS__VISIBILITY_CONTROL_H_
#define KORTEX2_CONTROLLERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define KORTEX2_CONTROLLERS_EXPORT __attribute__((dllexport))
#define KORTEX2_CONTROLLERS_IMPORT __attribute__((dllimport))
#else
#define KORTEX2_CONTROLLERS_EXPORT __declspec(dllexport)
#define KORTEX2_CONTROLLERS_IMPORT __declspec(dllimport)
#endif
#ifdef KORTEX2_CONTROLLERS_BUILDING_LIBRARY
#define KORTEX2_CONTROLLERS_PUBLIC KORTEX2_CONTROLLERS_EXPORT
#else
#define KORTEX2_CONTROLLERS_PUBLIC KORTEX2_CONTROLLERS_IMPORT
#endif
#define KORTEX2_CONTROLLERS_PUBLIC_TYPE KORTEX2_CONTROLLERS_PUBLIC
#define KORTEX2_CONTROLLERS_LOCAL
#else
#define KORTEX2_CONTROLLERS_EXPORT __attribute__((visibility("default")))
#define KORTEX2_CONTROLLERS_IMPORT
#if __GNUC__ >= 4
#define KORTEX2_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define KORTEX2_CONTROLLERS_LOCAL __attribute__((visibility("hidden")))
#else
#define KORTEX2_CONTROLLERS_PUBLIC
#define KORTEX2_CONTROLLERS_LOCAL
#endif
#define KORTEX2_CONTROLLERS_PUBLIC_TYPE
#endif

#endif  // KORTEX2_CONTROLLERS__VISIBILITY_CONTROL_H_
