
cmake_minimum_required(VERSION 2.8.12)
if (${CMAKE_VERSION} VERSION_GREATER 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(client_examples)

find_package(microcdr REQUIRED)
find_package(microxrcedds_client REQUIRED)

set(SRCS
    ModelCoefficients.c
    )

# ModelCoefficients Subscriber
add_executable(ModelCoefficientsSubscriber ModelCoefficientsSubscriber.c ${SRCS})
target_link_libraries(ModelCoefficientsSubscriber microxrcedds_client)
set_target_properties(ModelCoefficientsSubscriber PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(ModelCoefficientsSubscriber
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

# ModelCoefficients Publisher
add_executable(ModelCoefficientsPublisher ModelCoefficientsPublisher.c ${SRCS})
target_link_libraries(ModelCoefficientsPublisher microxrcedds_client)
set_target_properties(ModelCoefficientsPublisher PROPERTIES
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    )
target_compile_options(ModelCoefficientsPublisher
    PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/W4 /wd4700 /wd4996 /wd4820 /wd4255 /wd4668>
        $<$<C_COMPILER_ID:GNU>:-pedantic -fstrict-aliasing -Wall -Wcast-align -Wshadow>
    )

