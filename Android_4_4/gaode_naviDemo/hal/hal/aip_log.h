#ifndef __AIP_LOG_H__
#define __AIP_LOG_H__

#ifndef __func__
# if __STDC_VERSION__ < 199901L // No C99 compiler
#  if __GNUC__ >= 2 // GCC supports __FUNCTION__ instead
#   define __func__ __FUNCTION__
#  else // GCC does not even support __FUNCTION__
#   define __func__ ""
#  endif
# endif
#endif

#ifdef __GNUG__
# define __CPP_FUNC_SIGNATURE__ __PRETTY_FUNCTION__
#else
# define __CPP_FUNC_SIGNATURE__ ""
#endif

typedef enum {
    AIP_LOG_VERBOSE,
    AIP_LOG_DEBUG,
    AIP_LOG_INFO,
    AIP_LOG_WARN,
    AIP_LOG_ERROR,
    AIP_LOG_FATAL
} AIP_LOGPRIORITY;

#define AIP_LOG(pri, fmt, ...) _aip_log(pri, __FILE__, (long int)__LINE__, __func__, __CPP_FUNC_SIGNATURE__, fmt, ## __VA_ARGS__)
#define AIP_LOGV(fmt, ...) AIP_LOG(AIP_LOG_VERBOSE, fmt, ## __VA_ARGS__)
#define AIP_LOGD(fmt, ...) AIP_LOG(AIP_LOG_DEBUG, fmt, ## __VA_ARGS__)
#define AIP_LOGI(fmt, ...) AIP_LOG(AIP_LOG_INFO, fmt, ## __VA_ARGS__)
#define AIP_LOGW(fmt, ...) AIP_LOG(AIP_LOG_WARN, fmt, ## __VA_ARGS__)
#define AIP_LOGE(fmt, ...) AIP_LOG(AIP_LOG_ERROR, fmt, ## __VA_ARGS__)
#define AIP_LOGF(fmt, ...) AIP_LOG(AIP_LOG_FATAL, fmt, ## __VA_ARGS__)

int _aip_log(AIP_LOGPRIORITY pri,
                char const * const file, long int const line,
                char const * const c_func, char const * const cpp_func,
                char const * const fmt, ...);

#endif /* __AIP_LOG_H__ */
