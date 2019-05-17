#include "aip_serial.h"
#include "aip_log.h"
#include "common.h"

using namespace AIP;

#define BAUD(n) {n, B##n}

static const struct std_baudrate std_baudrates[] = {
    BAUD(50), BAUD(75), BAUD(110), BAUD(134), BAUD(150), BAUD(200),
    BAUD(300), BAUD(600),BAUD(1200),BAUD(1800), BAUD(2400), BAUD(4800),
    BAUD(9600), BAUD(19200), BAUD(38400), BAUD(57600), BAUD(115200),BAUD(230400),
    BAUD(460800)};

SerialPort::SerialPort() : fd(-1), is_opened(false), need_lock(false), 
    is_locked(false), is_setting_changed(false){
    memset(&this->setting_origin, 0, sizeof(this->setting_origin));
}

SerialPort::SerialPort(bool need_lock) : fd(-1), is_opened(false), need_lock(need_lock), 
    is_locked(false), is_setting_changed(false){
    memset(&this->setting_origin, 0, sizeof(this->setting_origin));
}

bool SerialPort::Open(const char *path, struct PortConfig &config) {
    if (NULL == path) {
        return false;
    }

    AIP_LOGI("path=%s,baudrate=%d,parity=%c,bits=%d,stop_bits=%d", 
        path, config.baudrate, config.parity, config.bits, config.stop_bits);

    if (this->is_opened) {
        // warning
    }

    int flags = O_RDWR | O_NOCTTY | O_NONBLOCK;
    int fd = open(path, flags);
    if (-1 == fd) {
        AIP_LOGE("open failed!");
        return false;
    }
    this->fd = fd;
    this->is_opened = true;
    AIP_LOGI("this->fd=%d", this->fd);
    
    if (need_lock) {
        if(flock(fd, LOCK_EX | LOCK_NB) != 0) {
            AIP_LOGE("flock failed!");
            Close();
            return false;
        }
        this->is_locked = true;
    }

    int ret = -1;
    ret = tcgetattr(fd, &(this->setting_origin));
    if (-1 == ret) {
        AIP_LOGE("tcgetattr failed!");
        Close();
        return false;
    }

    if (config.bits < 0) {
        AIP_LOGE("config.bits=%d invalid!", config.bits);
        Close();
        return false;
    }

    if (config.stop_bits < 0) {
        AIP_LOGE("config.stop_bits=%d invalid!", config.stop_bits);
        Close();
        return false;
    }

    struct termios setting_new;
    memset(&setting_new, 0, sizeof(setting_new));
    
    int baudrate = 0;
    int cbits = CS8;
    int cpar = 0;
    int ipar = IGNPAR;
    int cstop = 0;

    int i;
    for (i=0; i<(int)ARRAY_SIZE(std_baudrates); i++) {
        if (config.baudrate == std_baudrates[i].index) {
            baudrate = std_baudrates[i].value;
            break;
        }
    }
    
    if (i >= (int)ARRAY_SIZE(std_baudrates)) {
        AIP_LOGE("config.baudrate=%d invalid!", config.baudrate);
        Close();
        return false;
    }
    
    switch (config.bits) {
    case 8:
        cbits = CS8;
        break;
    case 7:
        cbits = CS7;
        break;
    case 6:
        cbits = CS6;
        break;
    case 5:
        cbits = CS5;
        break;
    default:
        AIP_LOGE("unexpected bits!");
        Close();
        return false;
    }
    
    switch (config.parity) {
    case 'N':
    case 'n':
        cpar = 0;
        ipar = IGNPAR;
        break;
    case 'E':
    case 'e':
        cpar = PARENB;
        ipar = INPCK;
        break;
    case 'O':
    case 'o':
        cpar = (PARENB | PARODD);
        ipar = INPCK;
        break;
    default:
        AIP_LOGE("unexpected parity!");
        Close();
        return false;
    }
    
    switch (config.stop_bits) {
    case 1:
        cstop = 0;
        break;
    case 2:
        cstop = CSTOPB;
        break;
    default:
        AIP_LOGE("unexpected stop bits!");
        Close();
        return false;
    }
    
    setting_new.c_cflag = cbits | cpar | cstop | CLOCAL | CREAD;
    setting_new.c_iflag = ipar;
    setting_new.c_oflag = 0;
    setting_new.c_lflag = 0;
    setting_new.c_cc[VMIN] = 0;
    setting_new.c_cc[VTIME] = 0;

    AIP_LOGI("c_cflag=%d, c_iflag=%d", setting_new.c_cflag, setting_new.c_iflag);

    int ispeed = cfsetispeed(&setting_new, baudrate);
    int ospeed = cfsetospeed(&setting_new, baudrate);
    
    if (tcsetattr(this->fd, TCSANOW, &setting_new) == -1) {
        AIP_LOGE("tcsetattr failed!");
        Close();
        return false;
    }
    this->is_setting_changed = true;

    // turn on DTR and DTS
    if (!change_status(0, TIOCM_DTR | TIOCM_RTS)) {
        AIP_LOGE("change_status failed!");
        Close();
        return false;
    }

    return true;
}

int SerialPort::Read(char *buffer, int size) { 
    int n = read(this->fd, buffer, size);
    
    if(n < 0) {
        if(errno == EAGAIN) {
            return 0;
        }
    }

    return n;
}

int SerialPort::Write(char *buffer, int size) {
    int n = write(this->fd, buffer, size);
    if(n < 0) {
        if(errno == EAGAIN) {
          return 0;
        }
    }

    return n;
}

bool SerialPort::change_status(int action, int mask) {
    int status;
    if(ioctl(this->fd, TIOCMGET, &status) == -1) {
        AIP_LOGE("get status failed!");
        return false;
    }
    
    if (0 == action) {
        status |= mask;
    } else {
        status &= ~mask;
    }

    // AIP_LOGI("status=%d", status);
    
    if (ioctl(this->fd, TIOCMSET, &status) == -1) {
        AIP_LOGE("set status failed!");
        return false;
    }

    return true;
}

void SerialPort::Close() {
    if (this->is_opened) {
        // turn off DTR and DTS
        change_status(1, TIOCM_DTR | TIOCM_RTS);

        // restore setting
        if (this->is_setting_changed) {
            tcsetattr(this->fd, TCSANOW, &this->setting_origin);
            this->is_setting_changed = false;
        }

        // close port
        if (close(this->fd) != -1) {
            this->is_opened = false;
        }

        // unlock port
        if (this->need_lock && this->is_locked) {
            flock(this->fd, LOCK_UN);
            this->is_locked = false;
        }
    }
}

SerialPort::~SerialPort() {
    Close();
}
