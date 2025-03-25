# cringbuf

High performance ring buffer for generic platform, optimized for embeded system.

1. multi-thread safe.
2. optimize memeory copy speed.
3. support fill or discard dataframe when buffer is going to be full.
4. optimize for continuous memory block, DMA-friendly.

---

## Build Test

### test on Linux

```bash
git clone https://github.com/Pansamic/cringbuf.git
cd cringbuf/test/Generic
mkdir build && cd build
cmake ..
make
./cringbuf_test 1024 10
```

### test on windows

```bash
git clone https://github.com/Pansamic/cringbuf.git
cd cringbuf/test/Generic
mkdir build && cd build
cmake -G "MinGW Makefiles" ..
mingw32-make
cringbuf_test 1024 10
```


### test on STM32 MCU

TODO. Files under test/STM32 are not ready to be tested. 

## Example

[On Linux PC](https://github.com/Pansamic/cringbuf/tree/master/test/linux_PC)

[On STM32 MCU](https://github.com/Pansamic/cringbuf/tree/master/test/STM32)

## API

```c
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
```

## TODO

1. add comment.