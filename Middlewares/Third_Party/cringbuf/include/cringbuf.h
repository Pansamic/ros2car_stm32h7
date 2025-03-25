/**
 * @file cringbuf.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief ring buffer for embeded system.
 * @feature multi-thread safe.
 * @feature optimize memeory copy speed.
 * @feature support fill or discard dataframe when buffer is going to be full.
 * @feature optimize for continuous memory block, DMA-friendly.
 * @version 0.2.0
 * @date 2025-03-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __RINGBUF_H__
#define __RINGBUF_H__

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************/
/*                    INCLUDE                        */
/*****************************************************/
#include <stdint.h>
#include <stddef.h>
/*****************************************************/
/*                     MACRO                         */
/*****************************************************/

/*****************************************************/
/*                     DEFINE                        */
/*****************************************************/
#if defined (__x86_64__) || defined (_M_X64)
    #define MAX_BYTE_POWER_OF_TWO (3)
    typedef uint64_t ringbuf_max_size_t;
#elif defined (i386) || defined (__i386__) || defined (__i386) || defined (_M_IX86)
    #define MAX_BYTE_POWER_OF_TWO (2)
    typedef uint32_t ringbuf_max_size_t;
#elif defined (__aarch64__) || defined (_M_ARM64)
    #define MAX_BYTE_POWER_OF_TWO (3)
    typedef uint64_t ringbuf_max_size_t;
#elif defined (__arm__) || defined (_M_ARM)
    #define MAX_BYTE_POWER_OF_TWO (2)
    typedef uint32_t ringbuf_max_size_t;
#else
    #error "Unsupported platform"
#endif
#define PLATFORM_MAX_BYTES (1<<MAX_BYTE_POWER_OF_TWO)



#if !defined(__WINDOWS__) && (defined(WIN32) || defined(WIN64) || defined(_MSC_VER) || defined(_WIN32))
#define __WINDOWS__
#endif

#ifdef __WINDOWS__

/* When compiling for windows, we specify a specific calling convention to avoid issues where we are being called from a project with a different default calling convention.  For windows you have 3 define options:

RINGBUF_HIDE_SYMBOLS - Define this in the case where you don't want to ever dllexport symbols
RINGBUF_EXPORT_SYMBOLS - Define this on library build when you want to dllexport symbols (default)
RINGBUF_IMPORT_SYMBOLS - Define this if you want to dllimport symbol

For *nix builds that support visibility attribute, you can define similar behavior by

setting default visibility to hidden by adding
-fvisibility=hidden (for gcc)
or
-xldscope=hidden (for sun cc)
to CFLAGS

then using the RINGBUF_API_VISIBILITY flag to "export" the same symbols the way RINGBUF_EXPORT_SYMBOLS does

*/

#define RINGBUF_CDECL __cdecl
#define RINGBUF_STDCALL __stdcall

/* export symbols by default, this is necessary for copy pasting the C and header file */
#if !defined(RINGBUF_HIDE_SYMBOLS) && !defined(RINGBUF_IMPORT_SYMBOLS) && !defined(RINGBUF_EXPORT_SYMBOLS)
#define RINGBUF_EXPORT_SYMBOLS
#endif

#if defined(RINGBUF_HIDE_SYMBOLS)
#define RINGBUF_PUBLIC(type)   type RINGBUF_STDCALL
#elif defined(RINGBUF_EXPORT_SYMBOLS)
#define RINGBUF_PUBLIC(type)   __declspec(dllexport) type RINGBUF_STDCALL
#elif defined(RINGBUF_IMPORT_SYMBOLS)
#define RINGBUF_PUBLIC(type)   __declspec(dllimport) type RINGBUF_STDCALL
#endif
#else /* !__WINDOWS__ */
#define RINGBUF_CDECL
#define RINGBUF_STDCALL

#if (defined(__GNUC__) || defined(__SUNPRO_CC) || defined (__SUNPRO_C)) && defined(RINGBUF_API_VISIBILITY)
#if __GNUC__ >= 4
#define RINGBUF_PUBLIC(type)   __attribute__((visibility("default"))) type
#elif 
#define RINGBUF_PUBLIC(type)   type
#endif /* __GNUC__ >= 4 */
#else
#define RINGBUF_PUBLIC(type)   type
#endif
#endif

/* project version */
#define RINGBUF_VERSION_MAJOR 0
#define RINGBUF_VERSION_MINOR 2
#define RINGBUF_VERSION_PATCH 0

/*****************************************************/
/*                   TYPEDEFINE                      */
/*****************************************************/
typedef enum ringbuf_return_type_definition
{
    RINGBUF_OK = 0,
    RINGBUF_ERROR,
    RINGBUF_FULL,
    RINGBUF_EMPTY,
    RINGBUF_LACK_SPACE,
    RINGBUF_LOCKED,
}ringbuf_ret_t;

typedef enum ringbuf_lock_type_definition
{
    RINGBUF_UNLOCK = 0,
    RINGBUF_LOCK,
}ringbuf_lock_t;

typedef enum ringbuf_rule_type_definition
{
    RINGBUF_RULE_FILL = 1,
    RINGBUF_RULE_DISCARD,
    RINGBUF_RULE_OVERWRITE
}ringbuf_rule_t;
typedef struct ringbuf_type_definition
{
    uint8_t *buf;
    volatile size_t capacity;
    volatile size_t size;
    volatile size_t head;
    volatile size_t tail;
    volatile uint8_t is_full;
    volatile uint8_t is_empty;
    volatile ringbuf_lock_t lock;
    ringbuf_rule_t rule;
}ringbuf_t;
/*****************************************************/
/*                    VARIABLE                       */
/*****************************************************/

/*****************************************************/
/*               FUNCTION DECLARATION                */
/*****************************************************/
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_init(ringbuf_t *ringbuf, void *buf, size_t capacity, ringbuf_rule_t rule);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_reset(ringbuf_t *ringbuf);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_align_optimize(ringbuf_t *ringbuf);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_write_byte(ringbuf_t *ringbuf, uint8_t data);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_write_block(ringbuf_t *ringbuf, void *data, size_t length);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_compensate_written(ringbuf_t *ringbuf, size_t length);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_byte(ringbuf_t *ringbuf, uint8_t *data);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_block(ringbuf_t *ringbuf, void *data, size_t length, size_t *read_length);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_peek_byte(ringbuf_t *ringbuf, uint8_t *data);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_peek_block(ringbuf_t *ringbuf, void *data, size_t length, size_t *read_length);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_remove_byte(ringbuf_t *ringbuf);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_remove_block(ringbuf_t *ringbuf, size_t length, size_t *removed_length);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_find_byte(ringbuf_t *ringbuf, uint8_t data, size_t *offset);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_find_block(ringbuf_t *ringbuf, void *data, size_t length, size_t *offset);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_size(ringbuf_t *ringbuf, size_t *size);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_capacity(ringbuf_t *ringbuf, size_t *capacity);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_free_size(ringbuf_t *ringbuf, size_t *free_size);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_free_continuous_block(ringbuf_t *ringbuf, void **data, size_t *length);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_stuffed_continuous_block(ringbuf_t *ringbuf, void **data, size_t *length);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_lock(ringbuf_t *ringbuf);
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_unlock(ringbuf_t *ringbuf);
RINGBUF_PUBLIC(int)           ringbuf_locked(ringbuf_t *ringbuf);




#ifdef __cplusplus
}
#endif

#endif /* __RINGBUF_H__ */
