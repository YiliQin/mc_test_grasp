#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define McTestGraspController_DLLIMPORT __declspec(dllimport)
#  define McTestGraspController_DLLEXPORT __declspec(dllexport)
#  define McTestGraspController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define McTestGraspController_DLLIMPORT __attribute__((visibility("default")))
#    define McTestGraspController_DLLEXPORT __attribute__((visibility("default")))
#    define McTestGraspController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define McTestGraspController_DLLIMPORT
#    define McTestGraspController_DLLEXPORT
#    define McTestGraspController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef McTestGraspController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define McTestGraspController_DLLAPI
#  define McTestGraspController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef McTestGraspController_EXPORTS
#    define McTestGraspController_DLLAPI McTestGraspController_DLLEXPORT
#  else
#    define McTestGraspController_DLLAPI McTestGraspController_DLLIMPORT
#  endif // McTestGraspController_EXPORTS
#  define McTestGraspController_LOCAL McTestGraspController_DLLLOCAL
#endif // McTestGraspController_STATIC