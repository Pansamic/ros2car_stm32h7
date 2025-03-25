#include <stdarg.h>
#include "main.h"
#include "syslog.h"
#include "lwprintf.h"

// one more byte for string end check.
char log_fmt_buffer[LOG_TEMP_BUFFER_SIZE];

void log_critical(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_CRITICAL
    const char* p = log_fmt_buffer;
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    uint64_t nsec = (uint64_t)ETH->MACSTNR;
    nsec *= 999999999ULL;
    nsec /= 0x7FFFFFFFULL;
    p += lwsnprintf(p, LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[31mcritical\033[0m]", ETH->MACSTSR, (uint32_t)nsec/1000);
    va_end(args);
    len = p-log_fmt_buffer;
    p += lwvsnprintf(p, LOG_TEMP_BUFFER_SIZE-len, fmt, args);
    len = p-log_fmt_buffer;
    uart_send(&uart1_cb, log_fmt_buffer, len);
#else
    (void)fmt;
#endif
}

void log_error(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_ERROR
    const char* p = log_fmt_buffer;
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    uint64_t nsec = (uint64_t)ETH->MACSTNR;
    nsec *= 999999999ULL;
    nsec /= 0x7FFFFFFFULL;
    p += lwsnprintf(p, LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[31m  error \033[0m]", ETH->MACSTSR, (uint32_t)nsec/1000);
    va_end(args);
    len = p-log_fmt_buffer;
    p += lwvsnprintf(p, LOG_TEMP_BUFFER_SIZE-len, fmt, args);
    len = p-log_fmt_buffer;
    uart_send(&uart1_cb, log_fmt_buffer, len);
#else
    (void)fmt;
#endif
}

void log_warn(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_WARN
    const char* p = log_fmt_buffer;
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    uint64_t nsec = (uint64_t)ETH->MACSTNR;
    nsec *= 999999999ULL;
    nsec /= 0x7FFFFFFFULL;
    p += lwsnprintf(p, LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[33m  warn  \033[0m]", ETH->MACSTSR, (uint32_t)nsec/1000);
    va_end(args);
    len = p-log_fmt_buffer;
    p += lwvsnprintf(p, LOG_TEMP_BUFFER_SIZE-len, fmt, args);
    len = p-log_fmt_buffer;
    uart_send(&uart1_cb, log_fmt_buffer, len);
#else
    (void)fmt;
#endif
}

void log_info(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_INFO
    const char* p = log_fmt_buffer;
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    uint64_t nsec = (uint64_t)ETH->MACSTNR;
    nsec *= 999999999ULL;
    nsec /= 0x7FFFFFFFULL;
    p += lwsnprintf(p, LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[34m  info  \033[0m]", ETH->MACSTSR, (uint32_t)nsec/1000);
    va_end(args);
    len = p-log_fmt_buffer;
    p += lwvsnprintf(p, LOG_TEMP_BUFFER_SIZE-len, fmt, args);
    len = p-log_fmt_buffer;
    uart_send(&uart1_cb, log_fmt_buffer, len);
#else
    (void)fmt;
#endif
}

void log_debug(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_DEBUG
    const char* p = log_fmt_buffer;
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    uint64_t nsec = (uint64_t)ETH->MACSTNR;
    nsec *= 999999999ULL;
    nsec /= 0x7FFFFFFFULL;
    p += lwsnprintf(p, LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[36m  debug \033[0m]", ETH->MACSTSR, (uint32_t)nsec/1000);
    va_end(args);
    len = p-log_fmt_buffer;
    p += lwvsnprintf(p, LOG_TEMP_BUFFER_SIZE-len, fmt, args);
    len = p-log_fmt_buffer;
    uart_send(&uart1_cb, log_fmt_buffer, len);
#else
    (void)fmt;
#endif
}

void log_trace(const char* fmt, ...)
{
#if LOG_LEVEL <= LOG_LEVEL_TRACE
    const char* p = log_fmt_buffer;
    va_list args;
    va_start(args, fmt);
    size_t len = 0;
    uint64_t nsec = (uint64_t)ETH->MACSTNR;
    nsec *= 999999999ULL;
    nsec /= 0x7FFFFFFFULL;
    p += lwsnprintf(p, LOG_TEMP_BUFFER_SIZE, "[%10lu.%06lu][\033[36m  trace \033[0m]", ETH->MACSTSR, (uint32_t)nsec/1000);
    va_end(args);
    len = p-log_fmt_buffer;
    p += lwvsnprintf(p, LOG_TEMP_BUFFER_SIZE-len, fmt, args);
    len = p-log_fmt_buffer;
    uart_send(&uart1_cb, log_fmt_buffer, len);
#else
    (void)fmt;
#endif
}