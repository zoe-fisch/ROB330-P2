// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbot_interfaces:msg/MotorVelocity.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbot_interfaces/msg/motor_velocity.h"


#ifndef MBOT_INTERFACES__MSG__DETAIL__MOTOR_VELOCITY__STRUCT_H_
#define MBOT_INTERFACES__MSG__DETAIL__MOTOR_VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MotorVelocity in the package mbot_interfaces.
typedef struct mbot_interfaces__msg__MotorVelocity
{
  float velocity[3];
} mbot_interfaces__msg__MotorVelocity;

// Struct for a sequence of mbot_interfaces__msg__MotorVelocity.
typedef struct mbot_interfaces__msg__MotorVelocity__Sequence
{
  mbot_interfaces__msg__MotorVelocity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbot_interfaces__msg__MotorVelocity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBOT_INTERFACES__MSG__DETAIL__MOTOR_VELOCITY__STRUCT_H_
