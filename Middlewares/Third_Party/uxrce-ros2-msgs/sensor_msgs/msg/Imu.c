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
 * @file Imu.c
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#include "sensor_msgs/msg/Imu.h"

#include <ucdr/microcdr.h>
#include <string.h>


bool sensor_msgs_msg_Imu_serialize_topic(ucdrBuffer* writer, const sensor_msgs_msg_Imu* topic)
{
    bool success = true;

        success &= std_msgs_msg_Header_serialize_topic(writer, &topic->header);
        success &= geometry_msgs_msg_Quaternion_serialize_topic(writer, &topic->orientation);
        success &= ucdr_serialize_array_double(writer, topic->orientation_covariance, sizeof(topic->orientation_covariance) / sizeof(double));

        success &= geometry_msgs_msg_Vector3_serialize_topic(writer, &topic->angular_velocity);
        success &= ucdr_serialize_array_double(writer, topic->angular_velocity_covariance, sizeof(topic->angular_velocity_covariance) / sizeof(double));

        success &= geometry_msgs_msg_Vector3_serialize_topic(writer, &topic->linear_acceleration);
        success &= ucdr_serialize_array_double(writer, topic->linear_acceleration_covariance, sizeof(topic->linear_acceleration_covariance) / sizeof(double));

    return success && !writer->error;
}

bool sensor_msgs_msg_Imu_deserialize_topic(ucdrBuffer* reader, sensor_msgs_msg_Imu* topic)
{
    bool success = true;

        success &= std_msgs_msg_Header_deserialize_topic(reader, &topic->header);
        success &= geometry_msgs_msg_Quaternion_deserialize_topic(reader, &topic->orientation);
        success &= ucdr_deserialize_array_double(reader, topic->orientation_covariance, sizeof(topic->orientation_covariance) / sizeof(double));

        success &= geometry_msgs_msg_Vector3_deserialize_topic(reader, &topic->angular_velocity);
        success &= ucdr_deserialize_array_double(reader, topic->angular_velocity_covariance, sizeof(topic->angular_velocity_covariance) / sizeof(double));

        success &= geometry_msgs_msg_Vector3_deserialize_topic(reader, &topic->linear_acceleration);
        success &= ucdr_deserialize_array_double(reader, topic->linear_acceleration_covariance, sizeof(topic->linear_acceleration_covariance) / sizeof(double));

    return success && !reader->error;
}

uint32_t sensor_msgs_msg_Imu_size_of_topic(const sensor_msgs_msg_Imu* topic, uint32_t size)
{
    uint32_t previousSize = size;
        size += std_msgs_msg_Header_size_of_topic(&topic->header, size);
        size += geometry_msgs_msg_Quaternion_size_of_topic(&topic->orientation, size);
        size += ucdr_alignment(size, 8) + sizeof(topic->orientation_covariance);

        size += geometry_msgs_msg_Vector3_size_of_topic(&topic->angular_velocity, size);
        size += ucdr_alignment(size, 8) + sizeof(topic->angular_velocity_covariance);

        size += geometry_msgs_msg_Vector3_size_of_topic(&topic->linear_acceleration, size);
        size += ucdr_alignment(size, 8) + sizeof(topic->linear_acceleration_covariance);

    return size - previousSize;
}
