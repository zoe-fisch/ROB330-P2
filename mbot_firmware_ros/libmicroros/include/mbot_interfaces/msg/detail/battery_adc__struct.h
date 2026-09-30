// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbot_interfaces:msg/BatteryADC.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbot_interfaces/msg/battery_adc.h"


#ifndef MBOT_INTERFACES__MSG__DETAIL__BATTERY_ADC__STRUCT_H_
#define MBOT_INTERFACES__MSG__DETAIL__BATTERY_ADC__STRUCT_H_

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

/// Struct defined in msg/BatteryADC in the package mbot_interfaces.
typedef struct mbot_interfaces__msg__BatteryADC
{
  /// ROS time when analog data was read
  builtin_interfaces__msg__Time stamp;
  /// no units
  int16_t raw[4];
  /// volts
  float volts[4];
} mbot_interfaces__msg__BatteryADC;

// Struct for a sequence of mbot_interfaces__msg__BatteryADC.
typedef struct mbot_interfaces__msg__BatteryADC__Sequence
{
  mbot_interfaces__msg__BatteryADC * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbot_interfaces__msg__BatteryADC__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBOT_INTERFACES__MSG__DETAIL__BATTERY_ADC__STRUCT_H_
