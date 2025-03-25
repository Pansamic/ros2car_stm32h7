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
 * @file TransitionEvent.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _TransitionEvent_H_
#define _TransitionEvent_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "State.h"
#include "Transition.h"

typedef struct lifecycle_msgs_msg_TransitionEvent
{
    uint64_t timestamp;
    lifecycle_msgs_msg_Transition transition;
    lifecycle_msgs_msg_State start_state;
    lifecycle_msgs_msg_State goal_state;
} lifecycle_msgs_msg_TransitionEvent;

struct ucdrBuffer;

bool lifecycle_msgs_msg_TransitionEvent_serialize_topic(struct ucdrBuffer* writer, const lifecycle_msgs_msg_TransitionEvent* topic);
bool lifecycle_msgs_msg_TransitionEvent_deserialize_topic(struct ucdrBuffer* reader, lifecycle_msgs_msg_TransitionEvent* topic);
uint32_t lifecycle_msgs_msg_TransitionEvent_size_of_topic(const lifecycle_msgs_msg_TransitionEvent* topic, uint32_t size);


#ifdef __cplusplus
}
#endif

#endif // _TransitionEvent_H_