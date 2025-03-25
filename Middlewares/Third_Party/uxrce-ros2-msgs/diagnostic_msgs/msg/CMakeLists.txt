
cmake_minimum_required(VERSION 2.8.12)
if (${CMAKE_VERSION} VERSION_GREATER 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(client_examples)

find_package(microcdr REQUIRED)
find_package(microxrcedds_client REQUIRED)

set(SRCS
    DiagnosticArray.c
    )

# DiagnosticArray Subscriber
add_executable(DiagnosticArraySubscriber DiagnosticArraySubscriber.c ${SRCS})
target_link_libraries(DiagnosticArraySubscriber microxrcedds_client)
set_target_properties(DiagnosticArraySubscriber PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(DiagnosticArraySubscriber
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

# DiagnosticArray Publisher
add_executable(DiagnosticArrayPublisher DiagnosticArrayPublisher.c ${SRCS})
target_link_libraries(DiagnosticArrayPublisher microxrcedds_client)
set_target_properties(DiagnosticArrayPublisher PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(DiagnosticArrayPublisher
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

