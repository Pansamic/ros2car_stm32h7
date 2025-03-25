#include <stdarg.h>
#include "main.h"
#include "syslog.h"
#include "lwprintf.h"

// one more byte for string end check.
char log_fmt_buffer1[LOG_TEMP_BUFFER_SIZE+1];
char log_fmt_buffer2[2*LOG_TEMP_BUFFER_SIZE+1];

void log_critical(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_CRITICAL
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    lwvsnprintf(log_fmt_buffer1, LOG_TEMP_BUFFER_SIZE, fmt, args);
    va_end(args);
    uint64_t nsec = (uint32_t)((uint64_t)ETH->MACSTNR*999999999ULL/0x7FFFFFFFULL);
    lwsnprintf(log_fmt_buffer2, 2*LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[31mcritical\033[0m]%s", ETH->MACSTSR, (uint32_t)nsec/1000, log_fmt_buffer1);
    len = strlen(log_fmt_buffer2);
    uart_send(&uart1_cb, log_fmt_buffer2, len);
#else
    (void)fmt;
#endif
}
void log_error(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_ERROR
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    lwvsnprintf(log_fmt_buffer1, LOG_TEMP_BUFFER_SIZE, fmt, args);
    va_end(args);
    uint64_t nsec = (uint32_t)((uint64_t)ETH->MACSTNR*999999999ULL/0x7FFFFFFFULL);
    lwsnprintf(log_fmt_buffer2, 2*LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[31m  error \033[0m]%s", ETH->MACSTSR, (uint32_t)nsec/1000, log_fmt_buffer1);
    len = strlen(log_fmt_buffer2);
    uart_send(&uart1_cb, log_fmt_buffer2, len);
#else
    (void)fmt;
#endif
}
void log_warn(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_WARN
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    lwvsnprintf(log_fmt_buffer1, LOG_TEMP_BUFFER_SIZE, fmt, args);
    va_end(args);
    uint64_t nsec = (uint32_t)((uint64_t)ETH->MACSTNR*999999999ULL/0x7FFFFFFFULL);
    lwsnprintf(log_fmt_buffer2, 2*LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[33m  warn  \033[0m]%s", ETH->MACSTSR, (uint32_t)nsec/1000, log_fmt_buffer1);
    len = strlen(log_fmt_buffer2);
    uart_send(&uart1_cb, log_fmt_buffer2, len);
#else
    (void)fmt;
#endif
}
void log_info(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_INFO
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    lwvsnprintf(log_fmt_buffer1, LOG_TEMP_BUFFER_SIZE, fmt, args);
    va_end(args);
    uint64_t nsec = (uint32_t)((uint64_t)ETH->MACSTNR*999999999ULL/0x7FFFFFFFULL);
    lwsnprintf(log_fmt_buffer2, 2*LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[34m  info  \033[0m]%s", ETH->MACSTSR, (uint32_t)nsec/1000, log_fmt_buffer1);
    len = strlen(log_fmt_buffer2);
    uart_send(&uart1_cb, log_fmt_buffer2, len);
#else
    (void)fmt;
#endif
}
void log_debug(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_DEBUG
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    lwvsnprintf(log_fmt_buffer1, LOG_TEMP_BUFFER_SIZE, fmt, args);
    va_end(args);
    uint64_t nsec = (uint32_t)((uint64_t)ETH->MACSTNR*999999999ULL/0x7FFFFFFFULL);
    lwsnprintf(log_fmt_buffer2, 2*LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[36m  debug \033[0m]%s", ETH->MACSTSR, (uint32_t)nsec/1000, log_fmt_buffer1);
    len = strlen(log_fmt_buffer2);
    uart_send(&uart1_cb, log_fmt_buffer2, len);
#else
    (void)fmt;
#endif
}
void log_trace(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_TRACE
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    lwvsnprintf(log_fmt_buffer1, LOG_TEMP_BUFFER_SIZE, fmt, args);
    va_end(args);
    uint64_t nsec = (uint32_t)((uint64_t)ETH->MACSTNR*999999999ULL/0x7FFFFFFFULL);
    lwsnprintf(log_fmt_buffer2, 2*LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[36m  trace \033[0m]%s", ETH->MACSTSR, (uint32_t)nsec/1000, log_fmt_buffer1);
    len = strlen(log_fmt_buffer2);
    uart_send(&uart1_cb, log_fmt_buffer2, len);
#else
    (void)fmt;
#endif
}