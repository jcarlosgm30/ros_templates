// Copyright 2021-2025
//
// @author: Jose Carlos Garcia
// @email: jcarlos3094@gmail.com
//

#ifndef ROS_PKG_TEMPLATE__VISIBILITY_H_
#define ROS_PKG_TEMPLATE__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define ROS_PKG_TEMPLATE_EXPORT __attribute__ ((dllexport))
    #define ROS_PKG_TEMPLATE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROS_PKG_TEMPLATE_EXPORT __declspec(dllexport)
    #define ROS_PKG_TEMPLATE_IMPORT __declspec(dllimport)
  #endif

  #ifdef ROS_PKG_TEMPLATE_DLL
    #define ROS_PKG_TEMPLATE_PUBLIC ROS_PKG_TEMPLATE_EXPORT
  #else
    #define ROS_PKG_TEMPLATE_PUBLIC ROS_PKG_TEMPLATE_IMPORT
  #endif

  #define ROS_PKG_TEMPLATE_PUBLIC_TYPE ROS_PKG_TEMPLATE_PUBLIC

  #define ROS_PKG_TEMPLATE_LOCAL

#else

  #define ROS_PKG_TEMPLATE_EXPORT __attribute__ ((visibility("default")))
  #define ROS_PKG_TEMPLATE_IMPORT

  #if __GNUC__ >= 4
    #define ROS_PKG_TEMPLATE_PUBLIC __attribute__ ((visibility("default")))
    #define ROS_PKG_TEMPLATE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROS_PKG_TEMPLATE_PUBLIC
    #define ROS_PKG_TEMPLATE_LOCAL
  #endif

  #define ROS_PKG_TEMPLATE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROS_PKG_TEMPLATE__VISIBILITY_H_
