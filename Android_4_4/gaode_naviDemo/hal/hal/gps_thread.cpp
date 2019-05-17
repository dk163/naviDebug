#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <assert.h>
#include <sys/epoll.h>
#include <cutils/sockets.h>

#include "gps_thread.h"

#include "aip_log.h"
#include "aip_parser.h"
#include "aip_serial.h"
#include "aip_hw_if.h"


// configuration of gps port 
static char aip_device_path[] = "/dev/ttymxc0";
static AIP::PortConfig config = {115200, 'N', 8, 1};
static AIP::SerialPort port;

/* commands sent to the gps thread */
typedef enum {
    CMD_UNKNOWN = 0,
    CMD_QUIT  = 1,
    CMD_START = 2,
    CMD_STOP  = 3
}GPS_THREAD_CMDS;

static int epoll_register( int  epoll_fd, int  fd )
{
    struct epoll_event  ev;
    int                 ret, flags;

    /* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

static int epoll_deregister( int  epoll_fd, int  fd )
{
    int  ret;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

static void update_gps_status(GpsStatusValue val)
{
    GpsState &state = AipHwIf::getInstance()->m_gpsState;
    GpsCallbacks &callbacks = AipHwIf::getInstance()->m_callbacks;
    
    state.status.status = val;
    
    if (callbacks.status_cb != NULL) {
        callbacks.status_cb(&state.status);
    }
}

/* dev control */
static void gps_dev_power(int state)
{
    return;
}

static void gps_dev_init(int fd){
    gps_dev_power(1);
    return;
}

static void gps_dev_deinit(int fd){
    gps_dev_power(0);
    return;
}

static void gps_dev_start(int fd){
    update_gps_status(GPS_STATUS_SESSION_BEGIN);
    return;
}

static void gps_dev_stop(int fd){
    update_gps_status(GPS_STATUS_SESSION_END);
    return;
}

/* gps state control */
int gps_state_init(){
    GpsState &state = AipHwIf::getInstance()->m_gpsState;
    GpsCallbacks &callbacks = AipHwIf::getInstance()->m_callbacks;

    state.init = true;
    
    if(!port.Open(aip_device_path, config)) {
        AIP_LOGE("open port failed!");
        return -1;
    }
    
    if (socketpair(AF_LOCAL, SOCK_STREAM, 0, state.ctl_fd) < 0) {
        AIP_LOGE("could not create thread control socket pair: %s", strerror(errno));
        return -1;
    }

    state.thread = callbacks.create_thread_cb("gps thread", aip_thread, &state);
    if (!state.thread) {
        AIP_LOGE("could not create GPS thread: %s", strerror(errno));
        return -1;
    }
    
    return 0;
}

int gps_state_start() {
    GpsState &state = AipHwIf::getInstance()->m_gpsState;

    char cmd = CMD_START;
    int ret = -1;
    
    do {
        ret = write(state.ctl_fd[0], &cmd, 1);
    } while (ret < 0 && errno == EINTR);

    if (ret != 1) {
        AIP_LOGE("gps_state_start() could not send CMD_START command: ret=%d: %s", ret, strerror(errno));
        return -1;
    }
    
    return 0;
}

int gps_state_stop() {
    GpsState &state = AipHwIf::getInstance()->m_gpsState;

    char cmd = CMD_STOP;
    int ret = -1;
    
    do {
        ret = write(state.ctl_fd[0], &cmd, 1);
    } while (ret < 0 && errno == EINTR);

    if (ret != 1) {
        AIP_LOGE("gps_state_stop() could not send CMD_STOP command: ret=%d: %s", ret, strerror(errno));
        return -1;
    }
    
    return 0;
}

void gps_state_done(){
    GpsState &state = AipHwIf::getInstance()->m_gpsState;
    
    // tell the thread to quit, and wait for it
    char   cmd = CMD_QUIT;
    write(state.ctl_fd[0], &cmd, 1);

    void*  dummy;
    pthread_join(state.thread, &dummy);

    // close the control socket pair
    close(state.ctl_fd[0] );
    state.ctl_fd[0] = -1;
    close(state.ctl_fd[1] );
    state.ctl_fd[1] = -1;

    // close connection to the QEMU GPS daemon
    port.Close();
    state.init = 0;
}


///////////////////////////////////////////////////////////////////////////////
// 主线程
void aip_thread(void *arg)
{
    AIP_LOGI("aip_thread begin");

    GpsState *ctl = (GpsState *) arg;
    int epoll_fd = epoll_create(2);
    int started = 0;
    int gps_fd = port.fd;
    int control_fd = ctl->ctl_fd[1];

    // register control file descriptors for polling
    epoll_register(epoll_fd, control_fd);
    epoll_register(epoll_fd, gps_fd);

    char buffer[RBUFSIZE+4];
    const char *remain = 0;
    size_t remainSize = 0;
    
    memset(buffer, '\0', sizeof(buffer));
    
    while (true) {
        struct epoll_event events[2];
        int nevents = epoll_wait(epoll_fd, events, 2, -1);
        if (nevents < 0) {
            if (errno != EINTR) {
                AIP_LOGE("epoll_wait() unexpected error: %s", strerror(errno));
            }
            continue;
        }
        
        for (int ne = 0; ne < nevents; ne++) {
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                AIP_LOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                gps_dev_deinit(gps_fd);
                return;
            }
            
            if ((events[ne].events & EPOLLIN) != 0) {
                int fd = events[ne].data.fd;
                if (fd == control_fd) {
                    char cmd = CMD_UNKNOWN;
                    int ret = -1;
                    do {
                        ret = read( fd, &cmd, 1);
                    } while (ret < 0 && errno == EINTR);

                    if (cmd == CMD_QUIT) {
                        AIP_LOGI("GPS thread quitting on demand");
                        gps_dev_deinit(gps_fd);
                        return;
                    } else if (cmd == CMD_START) {
                        if (!started) {
                            AIP_LOGI("GPS thread starting");
                            started = 1;
                            gps_dev_start(gps_fd);
                        }
                    } else if (cmd == CMD_STOP) {
                        if (started) {
                            AIP_LOGI("GPS thread stopping");
                            started = 0;
                            gps_dev_stop(gps_fd);
                        }
                    }
                } else if (fd == gps_fd) {
                    /* read port, parse and callback */
                    int avalibleSize = RBUFSIZE - remainSize;
                    // UBX_LOG(LCAT_DEBUG, "avalibleSize=%d", avalibleSize);
                    int readSize = port.Read(buffer + remainSize, avalibleSize);
                    // UBX_LOG(LCAT_DEBUG, "readSize=%d\n", readSize);
                    if (readSize <= 0) {
                        AIP_LOGE("port.Read() readSize<0");
                        break;
                    }

                    int realSize = remainSize + readSize;
                    buffer[realSize] = '\0';
                
                    // 解析缓冲区，并通过回调向外传递结果
                    remainSize = parseBuffer(realSize, buffer, callback, &remain);
                    
                    // copy遗留非完整行 将其copy到buffer的头部
                    if (remainSize > 0) {
                        memcpy(buffer, remain, remainSize);
                    }
                } else {
                    AIP_LOGE("epoll_wait() returned unkown fd %d ?", fd);
                }
            }
        }
    }
    return;
}


int gps_inject_time(GpsUtcTime timeUtcGps, int64_t timeReference, int uncertainty)
{
    return 0;
}

int gps_inject_location(double latitude, double longitude, float accuracy)
{
    return 0;
}

void gps_delete_aiding_data(GpsAidingData flags)
{
    return;
}

int gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence, 
        uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
    return 0;
}

