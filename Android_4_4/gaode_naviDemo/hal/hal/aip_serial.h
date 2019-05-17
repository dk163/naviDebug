#ifndef __AIP_SERIAL_H__
#define __AIP_SERIAL_H__

#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <sys/file.h>
#include <errno.h>

namespace AIP {
    struct std_baudrate {
        int index;
        speed_t value;
    };

    struct PortConfig {
        int baudrate;
        char parity;
        int bits;
        int stop_bits;
    };
    
    class SerialPort {
    public:
        SerialPort();
        SerialPort(bool need_lock);
        ~SerialPort();
        
        bool Open(const char *path, struct PortConfig &config);
        int Read(char *buffer, int size); // milliseconds
        int Write(char *buffer, int size);
        void Close();
    private:
        bool change_status(int action, int mask);
        
    private:
        SerialPort(const SerialPort &rhs);
        SerialPort& operator=(const SerialPort &rhs);

    public:
        int fd;
        
    private:
        bool is_opened;
        bool need_lock;
        bool is_locked;
        bool is_setting_changed;
        struct termios setting_origin;
    };
}

#endif