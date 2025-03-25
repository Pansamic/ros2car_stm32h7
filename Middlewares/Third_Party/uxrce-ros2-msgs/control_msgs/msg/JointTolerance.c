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
 * @file JointTolerance.c
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#include "control_msgs/msg/JointTolerance.h"

#include <ucdr/microcdr.h>
#include <string.h>

bool control_msgs_msg_JointTolerance_serialize_topic(ucdrBuffer* writer, const control_msgs_msg_JointTolerance* topic)
{
    bool success = true;

        success &= ucdr_serialize_string(writer, topic->name);

        success &= ucdr_serialize_double(writer, topic->position);

        success &= ucdr_serialize_double(writer, topic->velocity);

        success &= ucdr_serialize_double(writer, topic->acceleration);

    return success && !writer->error;
}

bool control_msgs_msg_JointTolerance_deserialize_topic(ucdrBuffer* reader, control_msgs_msg_JointTolerance* topic)
{
    bool success = true;

        success &= ucdr_deserialize_string(reader, topic->name, 255);

        success &= ucdr_deserialize_double(reader, &topic->position);

        success &= ucdr_deserialize_double(reader, &topic->velocity);

        success &= ucdr_deserialize_double(reader, &topic->acceleration);

    return success && !reader->error;
}

uint32_t control_msgs_msg_JointTolerance_size_of_topic(const control_msgs_msg_JointTolerance* topic, uint32_t size)
{
    uint32_t previousSize = size;
        size += ucdr_alignment(size, 4) + 4 + (uint32_t)strlen(topic->name) + 1;

        size += ucdr_alignment(size, 8) + 8;

        size += ucdr_alignment(size, 8) + 8;

        size += ucdr_alignment(size, 8) + 8;

    return size - previousSize;
}
