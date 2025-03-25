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
 * @file TwistWithCovarianceStamped.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _TwistWithCovarianceStamped_H_
#define _TwistWithCovarianceStamped_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "std_msgs/msg/Header.h"
#include "TwistWithCovariance.h"

typedef struct geometry_msgs_msg_TwistWithCovarianceStamped
{
    std_msgs_msg_Header header;
    geometry_msgs_msg_TwistWithCovariance twist;
} geometry_msgs_msg_TwistWithCovarianceStamped;

struct ucdrBuffer;

bool geometry_msgs_msg_TwistWithCovarianceStamped_serialize_topic(struct ucdrBuffer* writer, const geometry_msgs_msg_TwistWithCovarianceStamped* topic);
bool geometry_msgs_msg_TwistWithCovarianceStamped_deserialize_topic(struct ucdrBuffer* reader, geometry_msgs_msg_TwistWithCovarianceStamped* topic);
uint32_t geometry_msgs_msg_TwistWithCovarianceStamped_size_of_topic(const geometry_msgs_msg_TwistWithCovarianceStamped* topic, uint32_t size);


#ifdef __cplusplus
}
#endif

#endif // _TwistWithCovarianceStamped_H_