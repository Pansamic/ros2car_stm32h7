 /**
 * @file cringbuf.c
 * @author pansamic (pansamic@foxmail.com)
 * @brief ring buffer for embeded system.
 * @feature multi-thread safe.
 * @feature optimize memeory copy speed.
 * @feature support fill or discard dataframe when buffer is going to be full.
 * @feature optimize for continuous memory block, DMA-friendly.
 * @version 0.2.0
 * @date 2023-10-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */
/*****************************************************/
/*                    INCLUDE                        */
/*****************************************************/
#include "cringbuf.h"

/*****************************************************/
/*                     MACRO                         */
/*****************************************************/
/* This is a safeguard to prevent copy-pasters from using incompatible C and header files */
#if (RINGBUF_VERSION_MAJOR != 0) || (RINGBUF_VERSION_MINOR != 2) || (RINGBUF_VERSION_PATCH != 0)
    #error cringbuf.h and cringbuf.c have different versions. Make sure that both have the same.
#endif
/**
 * Checks if the buffer_size is a power of two.
 * Due to the design only <tt> RING_BUFFER_SIZE-1 </tt> items
 * can be contained in the buffer.
 * buffer_size must be a power of two.
*/
#define RINGBUF_IS_POWER_OF_TWO(buffer_capacity)\
    (((buffer_capacity) & ((buffer_capacity) - 1)) == 0)

#define RINGBUF_GET_FORWARD_INDEX(ringbuf,forward_num)\
    ((ringbuf->head + (forward_num)) & (ringbuf->capacity - 1))

#define RINGBUF_GET_RETREAT_INDEX(ringbuf,retreat_num)\
    ((ringbuf->tail + (retreat_num)) & (ringbuf->capacity - 1))

#define RINGBUF_ADD_LENGTH(ringbuf,forward_num)\
    ringbuf->head = RINGBUF_GET_FORWARD_INDEX(ringbuf,(forward_num));\
    ringbuf->size += (forward_num)

#define RINGBUF_REMOVE_LENGTH(ringbuf,retreat_num)\
    ringbuf->tail = RINGBUF_GET_RETREAT_INDEX(ringbuf,(retreat_num));\
    ringbuf->size -= (retreat_num)

#define RINGBUF_CHECK_FULL(ringbuf)\
    if(ringbuf->head == ringbuf->tail && ringbuf->size == ringbuf->capacity)\
    {\
        ringbuf->is_full = 1;\
    }

#define RINGBUF_CHECK_EMPTY(ringbuf)\
    if(ringbuf->head == ringbuf->tail)\
    {\
        ringbuf->is_empty = 1;\
    }
/*****************************************************/
/*                     DEFINE                        */
/*****************************************************/

/*****************************************************/
/*                   TYPEDEFINE                      */
/*****************************************************/

/*****************************************************/
/*                    VARIABLE                       */
/*****************************************************/

/*****************************************************/
/*              FUNCTION DECLARATION                 */
/*****************************************************/
inline static size_t ringbuf_memcpy(void *dest, void *src, size_t length);

/*****************************************************/
/*               FUNCTION DEFINITION                 */
/*****************************************************/

RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_init(ringbuf_t *ringbuf, void *buf, size_t capacity, ringbuf_rule_t rule)
{
    if (ringbuf == NULL || buf == NULL || capacity == 0)
    {
        return RINGBUF_ERROR;
    }
    if(!RINGBUF_IS_POWER_OF_TWO(capacity))
    {
        return RINGBUF_ERROR;
    }
    ringbuf->buf = buf;
    ringbuf->capacity = capacity;
    ringbuf->size = 0;
    ringbuf->head = 0;
    ringbuf->tail = 0;
    ringbuf->is_full = 0;
    ringbuf->is_empty = 1;
    ringbuf->lock = 0;
    ringbuf->rule = rule;
    return RINGBUF_OK;
}

RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_reset(ringbuf_t *ringbuf)
{
    if(ringbuf == NULL)
    {
        return RINGBUF_ERROR;
    }
    ringbuf->size = 0;
    ringbuf->head = 0;
    ringbuf->tail = 0;
    ringbuf->is_full = 0;
    ringbuf->is_empty = 1;
    ringbuf->lock = 0;
    return RINGBUF_OK;
}

// TODO
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_align_optimize(ringbuf_t *ringbuf)
{
    (void)ringbuf;
    return RINGBUF_OK;
}

RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_write_byte(ringbuf_t *ringbuf, uint8_t data)
{
    if(ringbuf == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    if(ringbuf->is_full && ringbuf->rule != RINGBUF_RULE_OVERWRITE)
    {
        return RINGBUF_FULL;
    }
    ringbuf->buf[ringbuf->head] = data;
    RINGBUF_ADD_LENGTH(ringbuf,1);
    if(ringbuf->is_full)
    {
        RINGBUF_REMOVE_LENGTH(ringbuf,1);
    }
    RINGBUF_CHECK_FULL(ringbuf);
    ringbuf->is_empty = 0;
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}

RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_write_block(ringbuf_t *ringbuf, void *data, size_t length)
{
    /* latch head and tail to avoid modification
     * by other thread or interrupt. */
    size_t current_head = ringbuf->head;
    size_t current_tail = ringbuf->tail;
    /* block begining from head */
    size_t first_block_length = 0;
    /* block begining from buffer head */
    size_t second_block_length = 0;
    size_t free_size = 0;
    size_t overwrite_size = 0;

    if(length == 0)
    {
        return RINGBUF_OK;
    }
    if(ringbuf == NULL || data == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf->is_full && ringbuf->rule != RINGBUF_RULE_OVERWRITE)
    {
        return RINGBUF_FULL;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }

    free_size = ringbuf->capacity - ringbuf->size;
    overwrite_size = length > free_size ? length - free_size : 0;

    if((free_size < length) && (ringbuf->rule == RINGBUF_RULE_DISCARD))
    {
        return RINGBUF_LACK_SPACE;
    }

    if(current_head > current_tail)
    {
        first_block_length = (current_head + length)>ringbuf->capacity ? ringbuf->capacity - current_head : length;
        second_block_length = (length - first_block_length)>(current_tail)?(current_tail):(length - first_block_length);
    }
    else if(current_head < current_tail)
    {
        second_block_length = 0;
        first_block_length = length > free_size ? free_size : length;
    }
    else
    {
        if(ringbuf->is_empty)
        {
            ringbuf->head = 0;
            ringbuf->tail = 0;
            current_head = ringbuf->head;
            current_tail = ringbuf->tail;
            first_block_length = length>ringbuf->capacity ? ringbuf->capacity : length;
            second_block_length = 0;
        }
    }

    ringbuf_memcpy((uint8_t*)ringbuf->buf + current_head, data, first_block_length);
    ringbuf_memcpy((uint8_t*)ringbuf->buf, (uint8_t*)data + first_block_length, second_block_length);
    RINGBUF_ADD_LENGTH(ringbuf,first_block_length + second_block_length);
    RINGBUF_CHECK_FULL(ringbuf);
    ringbuf->is_empty = 0;

    if(ringbuf->rule == RINGBUF_RULE_OVERWRITE)
    {
        for(size_t i=0 ; i<overwrite_size ; i++)
        {
            ringbuf->buf[ringbuf->head] = *((uint8_t*)data + free_size + i);
            RINGBUF_ADD_LENGTH(ringbuf,1);
            // TODO: examine the effect of commenting this line.
            // RINGBUF_REMOVE_LENGTH(ringbuf,1);
        }
    }
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}

RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_compensate_written(ringbuf_t *ringbuf, size_t length)
{
    if(ringbuf == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    RINGBUF_ADD_LENGTH(ringbuf,length);
    RINGBUF_CHECK_FULL(ringbuf);
    ringbuf->is_empty = 0;
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}

RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_byte(ringbuf_t *ringbuf, uint8_t *data)
{
    if(ringbuf == NULL || data == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf->is_empty)
    {
        return RINGBUF_EMPTY;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    *data = ringbuf->buf[ringbuf->tail];
    RINGBUF_REMOVE_LENGTH(ringbuf,1);
    RINGBUF_CHECK_EMPTY(ringbuf);
    ringbuf->is_full = 0;
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}

RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_block(ringbuf_t *ringbuf, void *data, size_t length, size_t *read_length)
{
    ringbuf_ret_t ret = RINGBUF_OK;
    size_t actual_read_length = 0;
    ret = ringbuf_peek_block(ringbuf, data, length, &actual_read_length);
    *read_length = 0;
    if(ret != RINGBUF_OK)
    {
        return ret;
    }
    ret = ringbuf_remove_block(ringbuf, actual_read_length, read_length);
    if(ret != RINGBUF_OK)
    {
        return ret;
    }
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_peek_byte(ringbuf_t *ringbuf, uint8_t *data)
{
    if(ringbuf == NULL || data == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf->is_empty)
    {
        return RINGBUF_EMPTY;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    *data = ringbuf->buf[ringbuf->tail];
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_peek_block(ringbuf_t *ringbuf, void *data, size_t length, size_t *read_length)
{
    /* latch head and tail to avoid modification
     * by other thread or interrupt. */
    size_t current_head = ringbuf->head;
    size_t current_tail = ringbuf->tail;
    /* block begining from tail */
    size_t first_block_length = 0;
    /* block begining from buffer head */
    size_t second_block_length = 0;
    size_t actual_read_length = 0;

    if(ringbuf == NULL || data == NULL || length == 0)
    {
        *read_length = 0;
        return RINGBUF_ERROR;
    }
    if(ringbuf->is_empty)
    {
        *read_length = 0;
        return RINGBUF_EMPTY;
    }
    if(ringbuf_lock(ringbuf))
    {
        *read_length = 0;
        return RINGBUF_LOCKED;
    }
    if(current_head > current_tail)
    {
        first_block_length = length > ringbuf->size ? ringbuf->size : length;
        second_block_length = 0;
        actual_read_length = first_block_length;
    }
    else if(current_head < current_tail)
    {
        first_block_length = length > (ringbuf->capacity - current_tail) ? (ringbuf->capacity - current_tail) : length;
        second_block_length = (length - first_block_length)>(current_head)?(current_head):(length - first_block_length);
        actual_read_length = first_block_length + second_block_length;
    }
    else
    {
        if(ringbuf->is_full)
        {
            first_block_length = (length > ringbuf->capacity-current_tail) ? ringbuf->capacity-current_tail : length;
            second_block_length = (length > first_block_length) ? ((length > ringbuf->capacity) ? current_head : length - first_block_length) : 0;
            actual_read_length = first_block_length + second_block_length;
        }
        else if(ringbuf->is_empty)
        {
            return RINGBUF_EMPTY;
        }
    }
    ringbuf_memcpy(data, (uint8_t*)ringbuf->buf + current_tail, first_block_length);
    ringbuf_memcpy((uint8_t*)data + first_block_length, ringbuf->buf, second_block_length);
    *read_length = actual_read_length;
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_remove_byte(ringbuf_t *ringbuf)
{
    if(ringbuf == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf->is_empty)
    {
        return RINGBUF_EMPTY;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    RINGBUF_REMOVE_LENGTH(ringbuf,1);
    RINGBUF_CHECK_EMPTY(ringbuf);
    ringbuf->is_full = 0;
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_remove_block(ringbuf_t *ringbuf, size_t length, size_t *removed_length)
{
    size_t actual_removed_length = 0;

    if(ringbuf == NULL || removed_length == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf->is_empty)
    {
        return RINGBUF_EMPTY;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }

    actual_removed_length = length > ringbuf->size ? ringbuf->size : length;
    RINGBUF_REMOVE_LENGTH(ringbuf,actual_removed_length);
    RINGBUF_CHECK_EMPTY(ringbuf);
    if(actual_removed_length > 0)
    {
        ringbuf->is_full = 0;
    }
    *removed_length = actual_removed_length;
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_find_byte(ringbuf_t *ringbuf, uint8_t data, size_t *offset)
{
    size_t current_tail = 0;
    size_t current_offset = 0;
    if(ringbuf == NULL || offset == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf->is_empty)
    {
        return RINGBUF_EMPTY;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    current_tail = ringbuf->tail;
    for(size_t i=0 ; i<ringbuf->size ; i++)
    {
        current_offset = (current_tail + i) & (ringbuf->capacity - 1);
        if(ringbuf->buf[current_offset] == data)
        {
            *offset = i;
            ringbuf_unlock(ringbuf);
            return RINGBUF_OK;
        }
    }
    ringbuf_unlock(ringbuf);
    return RINGBUF_ERROR;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_find_block(ringbuf_t *ringbuf, void *data, size_t length, size_t *offset)
{
    size_t current_tail = 0;
    size_t current_offset = 0;
    size_t actual_offset = 0;
    size_t actual_length = 0;
    size_t i = 0;
    if(ringbuf == NULL || data == NULL || offset == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf->is_empty)
    {
        return RINGBUF_EMPTY;
    }
    if(ringbuf->size < length)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    current_tail = ringbuf->tail;
    for(i=0 ; i<ringbuf->size ; i++)
    {
        current_offset = (current_tail + i) & (ringbuf->capacity - 1);
        if(ringbuf->buf[current_offset] == *((uint8_t*)data))
        {
            actual_offset = i;
            actual_length = 1;
            while(actual_length < length)
            {
                if(ringbuf->buf[(current_offset + actual_length) & (ringbuf->capacity - 1)] != *((uint8_t*)data + actual_length))
                {
                    break;
                }
                actual_length++;
            }
            if(actual_length == length)
            {
                *offset = actual_offset;
                ringbuf_unlock(ringbuf);
                return RINGBUF_OK;
            }
        }
    }
    ringbuf_unlock(ringbuf);
    return RINGBUF_ERROR;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_size(ringbuf_t *ringbuf, size_t *size)
{
    if(ringbuf == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    *size = ringbuf->size;
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_capacity(ringbuf_t *ringbuf, size_t *capacity)
{
    if(ringbuf == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    *capacity = ringbuf->capacity;
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_free_size(ringbuf_t *ringbuf, size_t *free_size)
{
    if(ringbuf == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    *free_size = ringbuf->capacity - ringbuf->size;
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_free_continuous_block(ringbuf_t *ringbuf, void **data, size_t *length)
{
    if(ringbuf == NULL || data == NULL || length == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    if(ringbuf->head > ringbuf->tail)
    {
        *data = ringbuf->buf + ringbuf->head;
        *length = ringbuf->capacity - ringbuf->head;
    }
    else if(ringbuf->head < ringbuf->tail)
    {
        *data = ringbuf->buf + ringbuf->head;
        *length = ringbuf->tail - ringbuf->head;
    }
    else
    {
        if(ringbuf->is_full)
        {
            *data = NULL;
            *length = 0;
            return RINGBUF_FULL;
        }
        else if(ringbuf->is_empty)
        {
            ringbuf->head = 0;
            ringbuf->tail = 0;
            *data = ringbuf->buf;
            *length = ringbuf->capacity;
        }
    }
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_get_stuffed_continuous_block(ringbuf_t *ringbuf, void **data, size_t *length)
{
    if(ringbuf == NULL || data == NULL || length == NULL)
    {
        return RINGBUF_ERROR;
    }
    if(ringbuf->is_empty)
    {
        return RINGBUF_EMPTY;
    }
    if(ringbuf_lock(ringbuf))
    {
        return RINGBUF_LOCKED;
    }
    if(ringbuf->head > ringbuf->tail)
    {
        *data = ringbuf->buf + ringbuf->tail;
        *length = ringbuf->head - ringbuf->tail;
    }
    else
    {
        *data = ringbuf->buf + ringbuf->tail;
        *length = ringbuf->capacity - ringbuf->tail;
    }
    ringbuf_unlock(ringbuf);
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_lock(ringbuf_t *ringbuf)
{
    if(ringbuf->lock == RINGBUF_LOCK)
    {
        return RINGBUF_LOCKED;
    }
    ringbuf->lock = RINGBUF_LOCK;
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(ringbuf_ret_t) ringbuf_unlock(ringbuf_t *ringbuf)
{
    ringbuf->lock = RINGBUF_UNLOCK;
    return RINGBUF_OK;
}
RINGBUF_PUBLIC(int) ringbuf_locked(ringbuf_t *ringbuf)
{
    if(ringbuf->lock == RINGBUF_LOCK)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
inline static size_t ringbuf_memcpy(void *dest, void *src, size_t length)
{
    if(length == 0)
    {
        return 0;
    }
    size_t block_size = length >> MAX_BYTE_POWER_OF_TWO;
    size_t byte_size = length & (PLATFORM_MAX_BYTES-1);
    ringbuf_max_size_t *dest_block_start_ptr = (ringbuf_max_size_t*)((uint8_t*)dest + byte_size);
    ringbuf_max_size_t *src_block_start_ptr = (ringbuf_max_size_t*)((uint8_t*)src + byte_size);
    uint8_t *dest_byte_start_ptr = (uint8_t *)dest;
    uint8_t *src_byte_start_ptr = (uint8_t *)src;
    while(byte_size--)
    {
        *dest_byte_start_ptr++ = *src_byte_start_ptr++;
    }
    while(block_size--)
    {
        *(dest_block_start_ptr) = *(src_block_start_ptr);
        dest_block_start_ptr++;
        src_block_start_ptr++;
    }
    return length;
}