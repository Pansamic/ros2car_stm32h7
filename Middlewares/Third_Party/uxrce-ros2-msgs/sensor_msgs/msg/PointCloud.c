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
 * @file PointCloud.c
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#include "sensor_msgs/msg/PointCloud.h"

#include <ucdr/microcdr.h>
#include <string.h>

bool sensor_msgs_msg_PointCloud_serialize_topic(ucdrBuffer* writer, const sensor_msgs_msg_PointCloud* topic)
{
    bool success = true;

        success &= std_msgs_msg_Header_serialize_topic(writer, &topic->header);
        success &= ucdr_serialize_uint32_t(writer, topic->points_size);
        for(size_t i = 0; i < topic->points_size; ++i)
        {
            success &= geometry_msgs_msg_Point32_serialize_topic(writer, &topic->points[i]);
        }

        success &= ucdr_serialize_uint32_t(writer, topic->channels_size);
        for(size_t i = 0; i < topic->channels_size; ++i)
        {
            success &= sensor_msgs_msg_ChannelFloat32_serialize_topic(writer, &topic->channels[i]);
        }

    return success && !writer->error;
}

bool sensor_msgs_msg_PointCloud_deserialize_topic(ucdrBuffer* reader, sensor_msgs_msg_PointCloud* topic)
{
    bool success = true;

        success &= std_msgs_msg_Header_deserialize_topic(reader, &topic->header);
        success &= ucdr_deserialize_uint32_t(reader, &topic->points_size);
        if(topic->points_size > 100)
        {
            reader->error = true;
        }
        else
        {
            for(size_t i = 0; i < topic->points_size; ++i)
            {
                success &= geometry_msgs_msg_Point32_deserialize_topic(reader, &topic->points[i]);
            }
        }

        success &= ucdr_deserialize_uint32_t(reader, &topic->channels_size);
        if(topic->channels_size > 100)
        {
            reader->error = true;
        }
        else
        {
            for(size_t i = 0; i < topic->channels_size; ++i)
            {
                success &= sensor_msgs_msg_ChannelFloat32_deserialize_topic(reader, &topic->channels[i]);
            }
        }

    return success && !reader->error;
}

uint32_t sensor_msgs_msg_PointCloud_size_of_topic(const sensor_msgs_msg_PointCloud* topic, uint32_t size)
{
    uint32_t previousSize = size;
        size += std_msgs_msg_Header_size_of_topic(&topic->header, size);
        size += ucdr_alignment(size, 4) + 4;
        for(size_t i = 0; i < topic->points_size; ++i)
        {
            size += geometry_msgs_msg_Point32_size_of_topic(&topic->points[i], size);
        }

        size += ucdr_alignment(size, 4) + 4;
        for(size_t i = 0; i < topic->channels_size; ++i)
        {
            size += sensor_msgs_msg_ChannelFloat32_size_of_topic(&topic->channels[i], size);
        }

    return size - previousSize;
}
