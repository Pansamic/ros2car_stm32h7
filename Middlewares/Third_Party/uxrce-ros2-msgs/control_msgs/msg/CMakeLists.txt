
cmake_minimum_required(VERSION 2.8.12)
if (${CMAKE_VERSION} VERSION_GREATER 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(client_examples)

find_package(microcdr REQUIRED)
find_package(microxrcedds_client REQUIRED)

set(SRCS
    DynamicJointState.c
    )

# DynamicJointState Subscriber
add_executable(DynamicJointStateSubscriber DynamicJointStateSubscriber.c ${SRCS})
target_link_libraries(DynamicJointStateSubscriber microxrcedds_client)
set_target_properties(DynamicJointStateSubscriber PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(DynamicJointStateSubscriber
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

# DynamicJointState Publisher
add_executable(DynamicJointStatePublisher DynamicJointStatePublisher.c ${SRCS})
target_link_libraries(DynamicJointStatePublisher microxrcedds_client)
set_target_properties(DynamicJointStatePublisher PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(DynamicJointStatePublisher
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

