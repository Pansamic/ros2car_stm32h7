// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file PidState.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _PidState_H_
#define _PidState_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "std_msgs/msg/Header.h"
#include "builtin_interfaces/msg/Duration.h"

typedef struct control_msgs_msg_PidState
{
    std_msgs_msg_Header header;
    builtin_interfaces_msg_Duration timestep;
    double error;
    double error_dot;
    double p_error;
    double i_error;
    double d_error;
    double p_term;
    double i_term;
    double d_term;
    double i_max;
    double i_min;
    double output;
} control_msgs_msg_PidState;

struct ucdrBuffer;

bool control_msgs_msg_PidState_serialize_topic(struct ucdrBuffer* writer, const control_msgs_msg_PidState* topic);
bool control_msgs_msg_PidState_deserialize_topic(struct ucdrBuffer* reader, control_msgs_msg_PidState* topic);
uint32_t control_msgs_msg_PidState_size_of_topic(const control_msgs_msg_PidState* topic, uint32_t size);


#ifdef __cplusplus
}
#endif

#endif // _PidState_H_