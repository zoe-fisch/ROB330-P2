// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbot_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbot_interfaces/msg/encoders.h"


#ifndef MBOT_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_
#define MBOT_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/Encoders in the package mbot_interfaces.
typedef struct mbot_interfaces__msg__Encoders
{
  /// ROS time when encoder data was read
  builtin_interfaces__msg__Time stamp;
  /// no units
  int64_t ticks[3];
  /// no units
  int32_t delta_ticks[3];
  int32_t delta_time;
} mbot_interfaces__msg__Encoders;

// Struct for a sequence of mbot_interfaces__msg__Encoders.
typedef struct mbot_interfaces__msg__Encoders__Sequence
{
  mbot_interfaces__msg__Encoders * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbot_interfaces__msg__Encoders__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBOT_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_
