
cmake_minimum_required(VERSION 2.8.12)
if (${CMAKE_VERSION} VERSION_GREATER 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(client_examples)

find_package(microcdr REQUIRED)
find_package(microxrcedds_client REQUIRED)

set(SRCS
    ImageMarker.c
    )

# ImageMarker Subscriber
add_executable(ImageMarkerSubscriber ImageMarkerSubscriber.c ${SRCS})
target_link_libraries(ImageMarkerSubscriber microxrcedds_client)
set_target_properties(ImageMarkerSubscriber PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(ImageMarkerSubscriber
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

# ImageMarker Publisher
add_executable(ImageMarkerPublisher ImageMarkerPublisher.c ${SRCS})
target_link_libraries(ImageMarkerPublisher microxrcedds_client)
set_target_properties(ImageMarkerPublisher PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(ImageMarkerPublisher
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

