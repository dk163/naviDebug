#include <log/log.h>

#include "aip_log.h"

//#define PRINT_FUNC_NAME

static const char TAG[]= "amap_aip";

static int _get_func_name_first_index(char const * const func_name) {
    if(func_name && *func_name) {
        int index = 0;
        int func_len = (int) strlen(func_name);
    
        int i = 0;
        for(; i < func_len && func_name[i]!='('; ++i) {
            if (func_name[i] == ' ') {
                index = i;
            }
        }
        
        if (index + 1 < func_len && func_name[index] == ' ') {
            ++index;
        } else {
            index = 0;
        }
        return index;
    } else {
        return 0;
    }
}

int _aip_log(AIP_LOGPRIORITY pri,
                char const * const file, long int const line,
                char const * const c_func, char const * const cpp_func,
                char const * const fmt, ...) {
    char buf[256]={0};
    
    va_list args;
    va_start(args,fmt);
    int num_w = vsnprintf(buf, sizeof(buf), fmt, args);
    if (num_w < 0) {
        ALOG(LOG_ERROR, TAG, "failed to log: %s", buf);
    }
    va_end(args);

#ifdef PRINT_FUNC_NAME
    int first_index = _get_func_name_first_index(c_func);
    int func_len = (int) strlen(c_func);
    char *func_name = new char[func_len - first_index + 1]();
    strncpy(func_name, c_func + first_index, func_len - first_index);
    func_name[func_len - first_index] = '\0';
#else
    char *func_name = new char[2]();
#endif
    
    int res;
    switch(pri)
    {
        case AIP_LOG_VERBOSE:
             res = ALOG(LOG_VERBOSE, TAG, "%s:%li %s: %s", file, line, func_name, buf);
             break;
        case AIP_LOG_DEBUG:
             res = ALOG(LOG_DEBUG, TAG, "%s:%li %s: %s", file, line, func_name, buf);
             break;
        case AIP_LOG_INFO:
             res = ALOG(LOG_INFO, TAG, "%s:%li %s: %s", file, line, func_name, buf);
             break;
        case AIP_LOG_WARN:
             res = ALOG(LOG_WARN, TAG, "%s:%li %s: %s", file, line, func_name, buf);
             break;
        case AIP_LOG_ERROR:
             res = ALOG(LOG_ERROR, TAG, "%s:%li %s: %s", file, line, func_name, buf);
             break;
        case AIP_LOG_FATAL:
             res = ALOG(LOG_FATAL, TAG, "%s:%li %s: %s", file, line, func_name, buf);
             break;
        default:
             res = ALOG(LOG_ERROR, TAG, "INVALID LOG LEVEL!");
             break;
    }

    delete [] func_name;
    return res;
}
