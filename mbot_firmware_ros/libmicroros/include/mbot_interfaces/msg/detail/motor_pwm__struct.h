// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbot_interfaces:msg/MotorPWM.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbot_interfaces/msg/motor_pwm.h"


#ifndef MBOT_INTERFACES__MSG__DETAIL__MOTOR_PWM__STRUCT_H_
#define MBOT_INTERFACES__MSG__DETAIL__MOTOR_PWM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MotorPWM in the package mbot_interfaces.
typedef struct mbot_interfaces__msg__MotorPWM
{
  /// [-1.0, 1.0]
  float pwm[3];
} mbot_interfaces__msg__MotorPWM;

// Struct for a sequence of mbot_interfaces__msg__MotorPWM.
typedef struct mbot_interfaces__msg__MotorPWM__Sequence
{
  mbot_interfaces__msg__MotorPWM * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbot_interfaces__msg__MotorPWM__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBOT_INTERFACES__MSG__DETAIL__MOTOR_PWM__STRUCT_H_
