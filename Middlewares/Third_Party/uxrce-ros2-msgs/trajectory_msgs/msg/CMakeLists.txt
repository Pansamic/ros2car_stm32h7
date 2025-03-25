
cmake_minimum_required(VERSION 2.8.12)
if (${CMAKE_VERSION} VERSION_GREATER 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(client_examples)

find_package(microcdr REQUIRED)
find_package(microxrcedds_client REQUIRED)

set(SRCS
    JointTrajectory.c
    )

# JointTrajectory Subscriber
add_executable(JointTrajectorySubscriber JointTrajectorySubscriber.c ${SRCS})
target_link_libraries(JointTrajectorySubscriber microxrcedds_client)
set_target_properties(JointTrajectorySubscriber PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(JointTrajectorySubscriber
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

# JointTrajectory Publisher
add_executable(JointTrajectoryPublisher JointTrajectoryPublisher.c ${SRCS})
target_link_libraries(JointTrajectoryPublisher microxrcedds_client)
set_target_properties(JointTrajectoryPublisher PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(JointTrajectoryPublisher
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

