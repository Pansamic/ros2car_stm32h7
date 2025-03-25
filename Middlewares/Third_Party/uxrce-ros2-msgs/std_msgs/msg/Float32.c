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
 * @file Float32.c
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#include "std_msgs/msg/Float32.h"

#include <ucdr/microcdr.h>
#include <string.h>

bool std_msgs_msg_Float32_serialize_topic(ucdrBuffer* writer, const std_msgs_msg_Float32* topic)
{
    bool success = true;

        success &= ucdr_serialize_float(writer, topic->data);
    return success && !writer->error;
}

bool std_msgs_msg_Float32_deserialize_topic(ucdrBuffer* reader, std_msgs_msg_Float32* topic)
{
    bool success = true;

        success &= ucdr_deserialize_float(reader, &topic->data);
    return success && !reader->error;
}

uint32_t std_msgs_msg_Float32_size_of_topic(const std_msgs_msg_Float32* topic, uint32_t size)
{
    uint32_t previousSize = size;
        size += ucdr_alignment(size, 4) + 4;

    return size - previousSize;
}
