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
 * @file Point.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _Point_H_
#define _Point_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct geometry_msgs_msg_Point
{
    double x;
    double y;
    double z;
} geometry_msgs_msg_Point;

struct ucdrBuffer;

bool geometry_msgs_msg_Point_serialize_topic(struct ucdrBuffer* writer, const geometry_msgs_msg_Point* topic);
bool geometry_msgs_msg_Point_deserialize_topic(struct ucdrBuffer* reader, geometry_msgs_msg_Point* topic);
uint32_t geometry_msgs_msg_Point_size_of_topic(const geometry_msgs_msg_Point* topic, uint32_t size);


#ifdef __cplusplus
}
#endif

#endif // _Point_H_