
cmake_minimum_required(VERSION 2.8.12)
if (${CMAKE_VERSION} VERSION_GREATER 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(client_examples)

find_package(microcdr REQUIRED)
find_package(microxrcedds_client REQUIRED)

set(SRCS
    OccupancyGridUpdate.c
    )

# OccupancyGridUpdate Subscriber
add_executable(OccupancyGridUpdateSubscriber OccupancyGridUpdateSubscriber.c ${SRCS})
target_link_libraries(OccupancyGridUpdateSubscriber microxrcedds_client)
set_target_properties(OccupancyGridUpdateSubscriber PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(OccupancyGridUpdateSubscriber
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

# OccupancyGridUpdate Publisher
add_executable(OccupancyGridUpdatePublisher OccupancyGridUpdatePublisher.c ${SRCS})
target_link_libraries(OccupancyGridUpdatePublisher microxrcedds_client)
set_target_properties(OccupancyGridUpdatePublisher PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(OccupancyGridUpdatePublisher
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

