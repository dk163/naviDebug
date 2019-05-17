/******************************************************************************
 *
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice is
 * included in all copies of any software which is or includes a copy or
 * modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF
 * THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: Android GNSS Driver
 *
 ******************************************************************************
 * $Id: ComThread.cpp 108350 2015-12-16 10:47:43Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/com/ComThread.cpp $
 *****************************************************************************/
 
/*! \file
    \ref CComThread implementation.

    \brief
    The \ref CComThread class is implemented in this file.
    Please refer to \ref ComThread.h for further information. 
*/

#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <limits.h>
#include <poll.h>
#include <stdlib.h>
#include <signal.h>
#include <assert.h>
#include "ComThread.h"
#include "../helper/helperFunctions.h"

/*! The constructor creates an object of this class and will initialise it to
    be operational. The Thread will however only be started when \ref start is
    called successfully.
*/    
CComThread::CComThread()
    : _config(NULL)
    , _ref_comThread(0)
    , _isStarted(false)
    , _runFlag(false)
    , _fds()
    , _numFds(0)
{
    pthread_mutex_init(&_isStartedMutex, NULL);
    pthread_mutex_init(&_fdsMutex, NULL);
    pthread_mutex_init(&_runFlagMutex, NULL);
    pthread_mutex_init(&_dMutex, NULL);
    pthread_mutex_init(&_numFdsMutex, NULL);
}

/*! The destructor will teardown the object and free the ressources
    where required.
*/
CComThread::~CComThread()
{
    stop_no_check();
    pthread_mutex_destroy(&_isStartedMutex);
    pthread_mutex_destroy(&_fdsMutex);
    pthread_mutex_destroy(&_runFlagMutex);
    pthread_mutex_destroy(&_dMutex);
    pthread_mutex_destroy(&_fdsMutex);
}

/*! Add the file descriptor to the handling list of the thread
    and get its location for interaction later on.

    \param fd    : file descriptor to be added
    \return      : On success the position of the file descriptor
                   in the list. Otherwise a negative number
*/
ssize_t CComThread::addSync(int fd)
{
    if(fd < 0 || _numFds > SSIZE_MAX)
        return -1;

    ssize_t result=-1;
    FD_POS_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.fd=fd;
    tmp.pos=acquireUniqNum();
    pthread_mutex_lock(&_fdsMutex);
    if(_fds.push(tmp))
    {
        result=tmp.pos;
    }
    pthread_mutex_unlock(&_fdsMutex);
    return result;
}

/*! Write to any of the file descriptors handled by the corresponding
    \ref CComThread object.

    \param dest  : Specifies the index of the file descriptor to which the
                   the data should be written. The index is defined by
                   the order in which the file descripter were added
                   to the \ref add call. Specifying a dest greater
                   than SSIZE_MAX will result in all descriptors at
                   positions which are greater or equal dest-SSIZE_MAX-1
                   being written to.
    \param buf   : Points to a buffer from which the data should be taken
                   that is written to the file descriptor specified by dest
    \param size  : Specifies the number of elements which should be read
                   read from buf for writing.
    \return        On success the minimum number of bytes written to the file
                   descriptors are returned. On failure -1 will be returned.
                   A failure will only occur if all write attempts fail
*/
ssize_t CComThread::writeToSync(size_t dest, unsigned char const * buf, size_t size)
{
    if( !isStarted() || !buf || !size || (dest >= _numFds && dest <= SSIZE_MAX) )
        return -1;

    assert(_config);

    // Find the file descriptor if it exists
    ssize_t result=-1;
    pthread_mutex_lock(&_fdsMutex);
    for(CList<FD_POS_t>::CIter i(_fds); i.isValid(); ++i)
    {
        FD_POS_t tmp;
        bool retVal=i.current(&tmp);
        // retVal must always be true, otherwise something
        // is off with the iterator / list (paralell use?)
        // and a segmentation fault is risked
        assert(retVal); ((void)(retVal));

        // Write to the file descriptor if it matches the
        // requested destination or if a destination bigger
        // than SSIZE_MAX was defined in which case a write
        // will take place if the position+SSIZE_MAX+1 is eqaul
        // or greater than the requested destination 
        if( tmp.pos==dest
         || ( dest > SSIZE_MAX && tmp.pos + SSIZE_MAX + 1 >= dest ) )
        {
            ssize_t writtenSize=write(tmp.fd, buf, size);
            if(result==-1 || (writtenSize >= 0 && writtenSize < result))
                result=writtenSize;
        }
    }
    pthread_mutex_unlock(&_fdsMutex);

    return result;
}

/*! Remove the file descriptor from the handling list of the thread

    \param pos   : file descriptor at this position shall be removed
    \return      : the file descriptor on success, a negative number otherwise
*/
int CComThread::remove(size_t pos)
{
    if( pos > _numFds || pos  > SSIZE_MAX )
        return -1;

    int result=-1;
    pthread_mutex_lock(&_fdsMutex);
    for(CList<FD_POS_t>::CIter i(_fds); i.isValid() && result==-1; ++i)
    {
        FD_POS_t tmp;
        bool retVal=i.current(&tmp);
        assert(retVal); ((void)(retVal));
        if(tmp.pos==pos)
        {
            free(tmp.buf);
            bool erased=i.erase();
            assert(erased); ((void)(erased));
            result=tmp.fd;
        }
    }
    pthread_mutex_unlock(&_fdsMutex);
    return result;
}

/*! Reads from all file descriptors ready and process the received
    data on success.

    \return       true if it could be determined which file descriptors are
                  ready. Otherwise, and in case no file descriptors are under
                  control by this object, false will be returned. 
*/
bool CComThread::readAndProcess()
{
    ARRAY_FDS_t arrFd;
    if(!getPollFds(&arrFd))
        return false;


    bool result=true;
    int ret = poll(arrFd.pollfds, arrFd.num, _config->timeout_ms);

    // Why did poll return?
    if( ret == -1 ) // An error occured
    {
        error(COMTHREAD_ERR_POLL, -1, errno);
        result=false;
    }
    else if(!ret) // timeout
    {
        timeout();
    }
    else // There is data input
    {
        // Is there data at the usr interface?
        ssize_t bytes_read = 0;

        // iterate through all interfaces and pass 
        // the read data to process(). If the
        // number of fds with data available has been
        // read (return value of poll) stop iterating 
        for(size_t i=0; i < arrFd.num; ++i)
        {
            size_t pos=arrFd.pos[i];
            int fd=arrFd.pollfds[i].fd;
            short revents=arrFd.pollfds[i].revents;
            // Check if there is an event from this file
            // descriptor. If the event is a DATA_AVAILABLE event
            // check if this event is supposed
            // to be handled with the process function
            if( (revents & POLLHUP)
             || (revents & POLLRDHUP) )
            {
                if(removeAndClose(pos))
                {
                    notify(pos, SOURCE_CLOSED);
                }
            }
            else if( (revents & POLLIN) 
                  && notify(pos, DATA_AVAILABLE) )
            {
                // Maximal size of a UBX message: 64 KiB
                // Do nothing if there is nothing to read
                unsigned char *pTmp = NULL;
                bytes_read=availRead(fd, &pTmp, MAX_BUFFER_SIZE);
                if(bytes_read > 0)
                {
                    // Store the data and let it be processed by the callback
                    process(pos, pTmp, bytes_read);
                    free(pTmp);
                }
                else if(bytes_read!=0)
                {
                    error(COMTHREAD_ERR_READ, arrFd.pos[i], errno);
                }
            }
        }
    }
    free(arrFd.pollfds);
    free(arrFd.pos);
    return result;
}


/*! Handle the delayed adds from \ref add
 
    \return                    : true if something was done, false otherwise
*/
bool CComThread::handleDelayedAdd()
{
    assert(isStarted());
    bool result=false;

    FD_POS_t tmpFd;
    while(_dAddFds.pop(&tmpFd, &_dMutex))
    {
        _fds.push(tmpFd, &_fdsMutex);
        result=true;
    }
    return result;
}

/*! Handle the delayed closes from \ref removeAndClose

    \return                    : true if something was done, false otherwise
*/
bool CComThread::handleDelayedClose()
{
    assert(isStarted());
    bool result=false;

    size_t pos;
    while(_dCloseFds.pop(&pos, &_dMutex))
    {
        result=true;
        int fd = remove(pos);
        assert(fd>0);
        if(fd > 0)
        {
            close(fd);
        }
    }
    return result;
}

/*! Handle the delayed writes from \ref writeTo

    \return                    : true if something was done, false otherwise
*/
bool CComThread::handleDelayedWrite()
{
    assert(isStarted());
    bool result=false;
    W_DELAYED_t tmp;
    while(_dWriteBufs.pop(&tmp, &_dMutex))
    {
        result=true;
        writeToSync(tmp.dest, tmp.buf, tmp.size);
        free(tmp.buf);
    }
    return result;
}

/*! This function will retrieve the pollfds structure out of the
    list of handled fds of the object. This function is NOT
    thread safe and must be protected by the same lock protecting
    the list when writing.
    \param arrFds     : Pointer to a struct to be filled by the function
    \return             true on success and false otherwise
*/
bool CComThread::getPollFds(ARRAY_FDS_t *arrFd)
{
    if(!arrFd)
        return false;

    bool result=false;
    size_t size=_fds.getSize();
    if(size > 0)
    {
        struct pollfd * pollFdTmp=(struct pollfd*) malloc(sizeof(*pollFdTmp)*size);
        size_t * posTmp=(size_t*) malloc(sizeof(*posTmp)*size);
        if(pollFdTmp && posTmp)
        {
            FD_POS_t cur;
            size_t counter=0;
            for(CList<FD_POS_t>::CIter i(_fds); i.isValid(); ++i, ++counter)
            {
                bool ret=i.current(&cur);
                assert(ret); ((void)(ret));
                pollFdTmp[counter].events = POLLIN;
                pollFdTmp[counter].fd     = cur.fd;
                posTmp[counter]           = cur.pos;
            }
            assert(counter==size);
            // Copy to the final destination
            arrFd->pollfds=pollFdTmp;
            arrFd->pos=posTmp;
            arrFd->num=size;
            result=true;
        }
        else
        {
            free(pollFdTmp);
            free(posTmp);
        }
    }
    return result;
}

/*! This function has to be called to setup the communication thread interface.
    The file descriptors passed to this function will be listened to by the 
    thread which will be started. The configuration struct passed will be
    copied by this function and needs only to exist during the call to this
    function. The same is true for the pointer to the array of file descriptors
    in it. The context pointer will however only be copied. The memory location
    it points to has to exist until \ref stop is called.

    \param config     : Contains callbacks and the file descriptors used to 
                        communicate with the interfaces
    \return             On success 0 is returned. On failure a negative number
                        is returned.
*/
bool CComThread::start(COM_CONFIG_t *config)

{
    if(isStarted() || !config || !config->func_process) // Can't open as resource is already is opened
        return false;

    COM_CONFIG_t *tmpconf=(COM_CONFIG_t *) malloc(sizeof(COM_CONFIG_t));
    if(!tmpconf)
        return false;

    memcpy(tmpconf, config, sizeof(COM_CONFIG_t));
    _config=tmpconf;

    // Create the thread responsible for packeting valid messages from the stream
    setRunFlag(true);

    if( 0 != pthread_create(&_ref_comThread
                            , NULL
                            , CComThread::bootstrapThread
                            , this) )
    {
        // something went wrong during thread creation - clean up
        int errno_bak = errno;

        // CLose all interfaces
        stop();
        errno = errno_bak;
    }
    else //everything is fine
    {
        // This is also the 'go' signal for the thread
        setStarted(true);
    }

    return isStarted();
}

/*! Add the file descriptor to the handling list of the thread
    and get its location for interaction later on.

    \param fd    : file descriptor to be added
    \return      : On successful scheduling the position of the file
                   descriptor in the list. Otherwise a negative number
*/
ssize_t CComThread::add(int fd)
{
    if(!isStarted() || fd < 0 || _numFds > SSIZE_MAX)
        return -1;

    ssize_t result=-1;
    FD_POS_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.fd=fd;
    tmp.pos=acquireUniqNum();
    if(_dAddFds.push(tmp, &_dMutex))
    {
        result=tmp.pos;
    }
    return result;
}

/*! Remove the file descriptor to the handling list of thread
    and close it.

    \param pos   : position of the file descriptor to be closed
    \return      : true on successfully scheduling a file descriptor for
                   removal. false otherwise
*/
bool CComThread::removeAndClose(size_t pos)
{
    if(!isStarted() || pos > _numFds || pos > SSIZE_MAX )
        return false;

    bool result=_dCloseFds.push(pos, &_dMutex);
    return result;
}

/*! This function tries to close the interface to the COM server and will return zero
    on success and a negative value on failure.

    \return A zero on success and a negative value on failure
*/
bool CComThread::stop()
{
    if(!isStarted()) // Can't close as resource is not yet open
        return false;

    stop_no_check();
    return true;
}

/*! By calling this function the calling application will be able to 
    to write to any of the file descriptors handled by the corresponding
    \ref CComThread object without having to actually call a write call
    itself (it is heavily encouraged to use this function). 

    \param dest  : Specifies the index of the file descriptor to which the
                   the data should be written. The index is defined by
                   the order in which the file descripter were added
                   to the \ref add call. Specifying a dest greater
                   than SSIZE_MAX will result in all descriptors at
                   positions which are greater or equal dest-SSIZE_MAX-1
                   being written to.
    \param buf   : Points to a buffer from which the data should be taken
                   that is written to the file descriptor specified by dest
    \param size  : Specifies the number of elements which should be read
                   read from buf for writing.
    \return        On successful scheduling true, otherwise false
*/
bool CComThread::writeTo(size_t dest, unsigned char const * buf, size_t size)
{
    if( !isStarted()
     || !buf
     || !size
     || (dest >= _numFds && dest <= SSIZE_MAX) )
        return false;

    assert(_config);

    // Find the file descriptor if it exists
    ssize_t result=-1;
    W_DELAYED_t data;
    data.dest=dest;
    data.buf=(unsigned char *) malloc(size);
    data.size=size;
    if(data.buf)
    {
        memcpy(data.buf, buf, size);
        _dWriteBufs.push(data, &_dMutex);
        result=true;
    }

    return result;
}

/*! Write to any of the file descriptors handled by the corresponding
    \ref CComThread object without having to actually call a write call
    itself. This version of the function enables the caller to write to
    several file descriptors at once.

    \param dest  : Specifies the indices of the file descriptors to which the
                   the data should be written.
    \param buf   : Points to a buffer from which the data should be taken
                   that is written to the file descriptor specified by dest
    \param size  : Specifies the number of elements which should be read
                   read from buf for writing.
    \return        On success the number of file descriptors to which the
                   data was successfully and completely written is returned.
                   Otherwise -1 is returned.
*/
ssize_t CComThread::writeTo(size_t *dest, size_t count, unsigned char const * buf, size_t size)
{
    if( !isStarted() || !buf || !size || !dest)
        return -1;

    assert(_config);
    size_t numFdsWritten=0;
    // Write to each listed destination as long as
    // it is within the valid destination range
    for(size_t i=0; i < count; ++i)
    {
        if( dest[i] <= (size_t)SSIZE_MAX
         && (size_t) writeTo(dest[i], buf, size) == size)
            ++numFdsWritten;
    }

    return numFdsWritten;
}

/*! As some of the internal functionality needs to
    clean the remaining ressources even if the
    interface has already been closed, this function
    is implemented without the check \ref stop()
    performs regarding the status of the object.
*/
void CComThread::stop_no_check()
{
    if(getRunFlag())
    {
        // First the thread has to be terminated
        setRunFlag(false); // terminates thread
        pthread_join(_ref_comThread, NULL);
    }
    // And then the values can be set back to their defaults
    setStarted(false);

    if(_config)
    {
        free(_config);
        _config=NULL;
    }


    // Clean all the file descriptors existing
    FD_POS_t tmpFd;
    while(_fds.pop(&tmpFd, &_fdsMutex))
    {
        if(tmpFd.fd > 0)
        {
            close(tmpFd.fd);
        }
        if(tmpFd.buf)
        {
            free(tmpFd.buf);
        }
    }

    // Clean all the queued file descriptors
    while(_dAddFds.pop(&tmpFd, &_dMutex))
    {
        if(tmpFd.fd > 0)
        {
            close(tmpFd.fd);
            assert(!tmpFd.buf);
        }
    }
    // Clean all the write buffers queued
    W_DELAYED_t tmpBuf;
    while(_dWriteBufs.pop(&tmpBuf, &_dMutex))
    {
        free(tmpBuf.buf);
    }
    // Clear all the close rquests
    _dCloseFds.clear(&_dMutex);

    pthread_mutex_lock(&_numFdsMutex);
    _numFds=0;
    pthread_mutex_unlock(&_numFdsMutex);
}


/*! Non-static class member functions can not be used to pass to pthread_create
    in order to run them as a separate thread because they posess a hidden
    argument which resembles a pointer to the actual object (this-pointer).
    Instead this static bootstrap function is called, which will not put such
    a hidden 'this' argument on the stack. Instead the one argument allowed by
    pthread_create is used to pass the corresponding pointer, which will then be
    used to call \ref CComThread on the correct object.

    \param arg : The pointer to the object on which the function must be called
    \return      NULL
*/
void *CComThread::bootstrapThread(void *arg)
{
    assert(arg);
    ((CComThread *) arg)->comThread();

    return NULL;
}

/*! The communication thread will check if new
    data arrives from the file descriptors handed
    to \ref start, will read it and pass it on to
    the configured function for processing. In case
    a timeout occurs during which no data arrives
    at any of the interfaces the configured timeout
    function will be called
 */
void CComThread::comThread()
{
    assert(_config);
    // Ignore all signals in this thread
    // (Otherwise it will interfere with poll)
    sigset_t set;
    sigfillset(&set);
    pthread_sigmask(SIG_SETMASK, &set, NULL);

    // Wait for the start() function to declare the
    // Thread started
    while(!isStarted())
        usleep(100000);

    while(getRunFlag())
    {
        bool sleepReq=false;

        // Get and process all data
        pthread_mutex_lock(&_fdsMutex);
        sleepReq=~readAndProcess();
        pthread_mutex_unlock(&_fdsMutex);

        // Handle all delayed actions
        handleDelayedAdd();           // Add the new file descriptors to the system
        handleDelayedWrite();         // Write buffers
        handleDelayedClose();         // Remove unnecessary file descriptors

        if(sleepReq)
            usleep(_config->timeout_ms);
    }
}

/*! This helper function will be called from the thread and is responsible for
    forwarding the received values to the callback passed during setup of the
    object through \ref start after storing the data received in the
       corresponding receive buffer. This function will only be called
    on a successful read.

    \param source              : The interface where the data passed was read
                                 from indicated as index in the filedescriptor
                                 array passed during \ref start 
    \param buf                 : The data that was read. Will not be NULL
    \param size                : The number of bytes passed. Will not be 0
    \return                      true on success, false otherwise
*/
bool CComThread::process(size_t source, unsigned char * buf, size_t size)
{
    assert(buf);
    assert(size);
    assert(_config);
    assert(_config->func_process);
    bool success=false;

    CList<FD_POS_t>::CIter iter(_fds);
    // Find the right position in the list
    while(iter.isValid() && !success)
    {
        if(iter->pos == source)
            success=true;
        else
            ++iter;
    }

    assert(iter->size >= iter->count);
    if(success)
    {
        success=false;
        assert((iter->size && iter->buf) || (!iter->size && !iter->buf));
        size_t freeMem = iter->size - iter->count;
        // Does the new data fit in the existing buffer? If not resize it
        if( freeMem >= size)
        {
            success=true;
        }
        else
        {
            // Has too much old data been accumulated in the buffer?
            // If yes, discard it.
            if( size + iter->size > MAX_BUFFER_SIZE )
            {
                free(iter->buf);
                iter->buf = NULL;
                iter->count = 0;
                iter->size = 0;
            }

            unsigned char * tmp = (unsigned char *) realloc(iter->buf, iter->size + size - freeMem);
            if(tmp)
            {
                iter->buf = tmp;
                iter->size += size - freeMem;
                success=true;
            }
        }

        // If there was either free space from the beginning
        // Or it has now been created, so fill in the data
        if(success)
        {
            memcpy(&iter->buf[iter->count], buf, size);
            iter->count += size;
        }
        else
        {
            free(iter->buf);
            iter->buf=NULL;
            iter->count = 0;
            iter->size = 0;
            success=false;
        }
    }

    // On failure, retry later on, otherwise process now
    if(success)
    {
        size_t processed = _config->func_process(_config->context, source, iter->buf, iter->count);
        if(processed == iter->count)
        {
            free(iter->buf);
            iter->buf=NULL;
            iter->count=0;
            iter->size=0;
        }
        else if(processed < iter->count)
        {
            iter->count -= processed;
            memmove(iter->buf, &iter->buf[processed], iter->count);
        }
        else // More has been processed than made available
        {
            error(COMTHREAD_ERR_PROC, source, errno);
            success = false;
            assert(0);
        }
    }
    return success;
}

/*! If no data arrives from any interface within the specified timeout 
    this function will be called which will call the calllback passed
    to the object through \ref start
*/
void CComThread::timeout() const
{
    assert(_config);
    if(_config->func_timeout)
    {
        _config->func_timeout(_config->context);
    }
}

/*! This function will be called from the thread if an error
    occurs while reading / writing or waiting on any of the
    interfaces.

    \param operation :  Will indicate which operation led to
                        the error. Valid values are
                        COMTHREAD_ERR_POLL
                        COMTHREAD_ERR_READ
                        COMTHREAD_ERR_PROC
    \param fdindex   :  Indicates which of the file descriptors
                        was involved during the error. If an
                        operation failed which does not relate
                        to a specific file descriptor, -1 will
                        be passed.
    \param correrrno :  The errno value that was set during the
                        operation that caused the error
*/
void CComThread::error(int operation, ssize_t fdindex, int correrrno) const
{
    assert(_config);
    if(_config->func_error)
    {
        _config->func_error(_config->context, operation, fdindex, correrrno);
    }
}

/*! This function will be called from the thread if data is available
    from a source. If func_nttify has not been defined in \ref _config
    true will be returned.

    \param source              : Number of the file descriptor from which
                                 data is available
    \param type                : Type of event that occurred
    \return                      true if the data should be read and passed
                                 to the process function. false if the
                                 data will be handled otherwise.
*/
bool CComThread::notify(size_t source, NOTIF_t type) const
{
    assert(_config);
    bool result=true; // In doubt, read the buffer!
    if(_config->func_notify)
    {
        result=_config->func_notify(_config->context, source, type);
    }
    return result;
}

/*! Set the run flag to the value provided

    \param newValue            : The future value of _runFlag 
*/
void CComThread::setRunFlag(bool newValue)
{
    pthread_mutex_lock(&_runFlagMutex);
    _runFlag=newValue;
    pthread_mutex_unlock(&_runFlagMutex);
}

/*! Get the run flag to the value provided

    \return                      The current value of _runFlag 
*/
bool CComThread::getRunFlag()
{
    pthread_mutex_lock(&_runFlagMutex);
    bool result=_runFlag;
    pthread_mutex_unlock(&_runFlagMutex);
    return result;
}

/*! Return a uniq value

    \return                      A uniq value
*/
size_t CComThread::acquireUniqNum()
{
    pthread_mutex_lock(&_numFdsMutex);
    size_t result=_numFds++;
    pthread_mutex_unlock(&_numFdsMutex);
    return result;
}

/*! Set the object to be started or stopped

    \param newValue            : The new status of the object
*/
void CComThread::setStarted(bool newValue)
{
    pthread_mutex_lock(&_isStartedMutex);
    _isStarted=newValue;
    pthread_mutex_unlock(&_isStartedMutex);
}

/*! Get the status of the object

    \return                    : true if the thread is started
                                 and false otherwise
*/
bool CComThread::isStarted()
{
    pthread_mutex_lock(&_isStartedMutex);
    bool result=_isStarted;
    pthread_mutex_unlock(&_isStartedMutex);
    return result;
}

