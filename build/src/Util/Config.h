#ifndef CNOID_CONFIG_H
#define CNOID_CONFIG_H

#define CNOID_MAJOR_VERSION 2
#define CNOID_MINOR_VERSION 0
#define CNOID_PATCH_VERSION 0
#define CNOID_VERSION 0x020000
#define CNOID_INTERNAL_VERSION 2

#define CNOID_VERSION_STRING "2.0"
#define CNOID_FULL_VERSION_STRING "2.0.0"
#define CNOID_VERSION_SUBDIR "choreonoid-2.0"
#define CNOID_PLUGIN_SUBDIR "lib/choreonoid-2.0"
#define CNOID_SHARE_SUBDIR "share"

/* #undef CNOID_USE_PYTHON2 */
#define CNOID_ENABLE_GETTEXT
/* #undef CNOID_ENABLE_FREE_TYPE */
/* #undef CNOID_ENABLE_GPERFTOOLS_PROFILER */

#if defined _WIN32 || defined __CYGWIN__
#define CNOID_GENERAL_EXPORT __declspec(dllexport)
#else
#if __GNUC__ >= 4
#define CNOID_GENERAL_EXPORT __attribute__ ((visibility("default")))
#endif
#endif

namespace cnoid {

CNOID_GENERAL_EXPORT unsigned int getVersion();
CNOID_GENERAL_EXPORT unsigned int getInternalVersion();

}

#endif
