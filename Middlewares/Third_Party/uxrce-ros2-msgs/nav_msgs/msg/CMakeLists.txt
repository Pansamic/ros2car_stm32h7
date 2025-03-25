
cmake_minimum_required(VERSION 2.8.12)
if (${CMAKE_VERSION} VERSION_GREATER 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(client_examples)

find_package(microcdr REQUIRED)
find_package(microxrcedds_client REQUIRED)

set(SRCS
    GridCells.c
    )

# GridCells Subscriber
add_executable(GridCellsSubscriber GridCellsSubscriber.c ${SRCS})
target_link_libraries(GridCellsSubscriber microxrcedds_client)
set_target_properties(GridCellsSubscriber PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(GridCellsSubscriber
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

# GridCells Publisher
add_executable(GridCellsPublisher GridCellsPublisher.c ${SRCS})
target_link_libraries(GridCellsPublisher microxrcedds_client)
set_target_properties(GridCellsPublisher PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(GridCellsPublisher
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

