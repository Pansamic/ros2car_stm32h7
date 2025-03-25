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
 * @file Quaternion.c
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#include "geometry_msgs/msg/Quaternion.h"

#include <ucdr/microcdr.h>
#include <string.h>

bool geometry_msgs_msg_Quaternion_serialize_topic(ucdrBuffer* writer, const geometry_msgs_msg_Quaternion* topic)
{
    bool success = true;

        success &= ucdr_serialize_double(writer, topic->x);

        success &= ucdr_serialize_double(writer, topic->y);

        success &= ucdr_serialize_double(writer, topic->z);

        success &= ucdr_serialize_double(writer, topic->w);

    return success && !writer->error;
}

bool geometry_msgs_msg_Quaternion_deserialize_topic(ucdrBuffer* reader, geometry_msgs_msg_Quaternion* topic)
{
    bool success = true;

        success &= ucdr_deserialize_double(reader, &topic->x);

        success &= ucdr_deserialize_double(reader, &topic->y);

        success &= ucdr_deserialize_double(reader, &topic->z);

        success &= ucdr_deserialize_double(reader, &topic->w);

    return success && !reader->error;
}

uint32_t geometry_msgs_msg_Quaternion_size_of_topic(const geometry_msgs_msg_Quaternion* topic, uint32_t size)
{
    uint32_t previousSize = size;
        size += ucdr_alignment(size, 8) + 8;

        size += ucdr_alignment(size, 8) + 8;

        size += ucdr_alignment(size, 8) + 8;

        size += ucdr_alignment(size, 8) + 8;

    return size - previousSize;
}
