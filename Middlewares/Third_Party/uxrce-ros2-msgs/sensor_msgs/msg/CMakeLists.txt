
cmake_minimum_required(VERSION 2.8.12)
if (${CMAKE_VERSION} VERSION_GREATER 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(client_examples)

find_package(microcdr REQUIRED)
find_package(microxrcedds_client REQUIRED)

set(SRCS
    BatteryState.c
    )

# BatteryState Subscriber
add_executable(BatteryStateSubscriber BatteryStateSubscriber.c ${SRCS})
target_link_libraries(BatteryStateSubscriber microxrcedds_client)
set_target_properties(BatteryStateSubscriber PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(BatteryStateSubscriber
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

# BatteryState Publisher
add_executable(BatteryStatePublisher BatteryStatePublisher.c ${SRCS})
target_link_libraries(BatteryStatePublisher microxrcedds_client)
set_target_properties(BatteryStatePublisher PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(BatteryStatePublisher
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

