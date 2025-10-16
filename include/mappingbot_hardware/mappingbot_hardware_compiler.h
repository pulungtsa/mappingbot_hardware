#ifndef __MAPPINGBOT_HARDWARE__MAPPINGBOT_HARDWARE_COMPILER_H__
#define __MAPPINGBOT_HARDWARE__MAPPINGBOT_HARDWARE_COMPILER_H__

#if defined _WIN32 || defined __CYGWIN__
#   ifdef __GNUC__
#       define MAPPINGBOT_HARDWARE_EXPORT __attribute__((dllexport))
#       define MAPPINGBOT_HARDWARE_IMPORT __attribute__((dllimport))
#   else
#       define MAPPINGBOT_HARDWARE_EXPORT __declspec(dllexport)
#       define MAPPINGBOT_HARDWARE_IMPORT __declspec(dllimport)
#   endif
#   ifdef MAPPINGBOT_HARDWARE_BUILDING_DLL
#       define MAPPINGBOT_HARDWARE_PUBLIC MAPPINGBOT_HARDWARE_EXPORT
#   else
#       define MAPPINGBOT_HARDWARE_PUBLIC MAPPINGBOT_HARDWARE_IMPORT
#   endif
#   define MAPPINGBOT_HARDWARE_PUBLIC_TYPE MAPPINGBOT_HARDWARE_PUBLIC
#   define MAPPINGBOT_HARDWARE_LOCAL
#else
#   define MAPPINGBOT_HARDWARE_EXPORT __attribute__((visibility("default")))
#   define MAPPINGBOT_HARDWARE_IMPORT
#   if __GNUC__ >= 4
#       define MAPPINGBOT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#       define MAPPINGBOT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#   else
#       define MAPPINGBOT_HARDWARE_PUBLIC
#       define MAPPINGBOT_HARDWARE_LOCAL
#   endif
#   define MAPPINGBOT_HARDWARE_PUBLIC_TYPE
#endif

#endif // __MAPPINGBOT_HARDWARE__MAPPINGBOT_HARDWARE_COMPILER_H__