#pragma once

#define PARAM_HELPER_STRINGIFY(x) #x

#define PARAM_HELPER_TOSTRING(x) PARAM_HELPER_STRINGIFY(x)

#define READ_PARAM_BEGIN ros::NodeHandle ___nh_param_reader("~");

#define READ_PARAM_END ;

#define READ_STRING_PARAM_WITH_DEFAULT(x, default_value)             \
  std::string x;                                                     \
  ___nh_param_reader.param<std::string>(PARAM_HELPER_TOSTRING(x), x, \
                                        default_value);              \
  ROS_INFO_STREAM(PARAM_HELPER_TOSTRING(x) << ": " << x);

#define READ_INT_PARAM_WITH_DEFAULT(x, default_value)                        \
  int x;                                                                     \
  ___nh_param_reader.param<int>(PARAM_HELPER_TOSTRING(x), x, default_value); \
  ROS_INFO_STREAM(PARAM_HELPER_TOSTRING(x) << ": " << x);

#define READ_DOUBLE_PARAM_WITH_DEFAULT(x, default_value)                        \
  double x;                                                                     \
  ___nh_param_reader.param<double>(PARAM_HELPER_TOSTRING(x), x, default_value); \
  ROS_INFO_STREAM(PARAM_HELPER_TOSTRING(x) << ": " << x);

#define READ_PARAM_WITH_DEFAULT(type_of_x, x, default_value)                       \
  type_of_x x;                                                                     \
  ___nh_param_reader.param<type_of_x>(PARAM_HELPER_TOSTRING(x), x, default_value); \
  ROS_INFO_STREAM(PARAM_HELPER_TOSTRING(x) << ": " << x);
