#ifndef __LOG_H__
#define __LOG_H__

#define LOG_TEMP_BUFFER_SIZE 256

#define LOG_LEVEL_CRITICAL 6
#define LOG_LEVEL_ERROR    5
#define LOG_LEVEL_WARN     4
#define LOG_LEVEL_INFO     3
#define LOG_LEVEL_DEBUG    2
#define LOG_LEVEL_TRACE    1

#define LOG_LEVEL LOG_LEVEL_INFO

#define LOG_CRITICAL(...) log_critical(__VA_ARGS__)
#define LOG_ERROR(...) log_error(__VA_ARGS__)
#define LOG_WARN(...) log_warn(__VA_ARGS__)
#define LOG_INFO(...) log_info(__VA_ARGS__)
#define LOG_DEBUG(...) log_debug(__VA_ARGS__)
#define LOG_TRACE(...) log_trace(__VA_ARGS__)

void log_critical(const char* fmt, ...);
void log_error(const char* fmt, ...);
void log_warn(const char* fmt, ...);
void log_info(const char* fmt, ...);
void log_debug(const char* fmt, ...);
void log_trace(const char* fmt, ...);
#endif // __LOG_H__