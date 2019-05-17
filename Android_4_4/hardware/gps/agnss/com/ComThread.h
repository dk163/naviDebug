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
 * $Id: ComThread.h 112238 2016-03-09 10:36:38Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/com/ComThread.h $
 *****************************************************************************/
 
/*! \file
    \ref CComThread implementation.

    \brief
    Contains the \ref CComThread declarations
    The \ref CComThread class is declared in this file.
    Please refer to \ref ComThread.cpp for further information. 
*/

#ifndef __UBX_COMTHREAD_H__
#define __UBX_COMTHREAD_H__
#include <unistd.h>
#include <pthread.h>
#include <stdarg.h>
#include "../helper/helperTypes.h"
#include "../list/List.h"

#define COMTHREAD_ERR_POLL 0   //!< Indicates an error occured while polling
#define COMTHREAD_ERR_READ 1   //!< Indicates an error occured while reading
#define COMTHREAD_ERR_PROC 2   //!< Indicates an error while allocating memory
#define MAX_BUFFER_SIZE 65536  //!< Maximum size of the buffers in use

/*! \class CComThread
    \brief Provides a communication thread interface

    This class implements file descriptor handler in a separate thread. The
    object will act as a wrapper to the thread and provide data read from
    the file descriptors to the functions assigned to the function pointers
    in \ref COM_CONFIG_t. It is possible to 
*/
class CComThread
{
private: // Definitions
    //! Structure with which the \ref _fds list will be filled
    typedef struct
    {
        int fd;                    //!< File descriptor to be stored
        size_t pos;                //!< Position within the list
        unsigned char *buf;        //!< Data ready for processing
        size_t count;              //!< Data in buf
        size_t size;               //!< Size of the buffer
    } FD_POS_t;

    //! Delayed write requests are stored in this structure
    typedef struct
    {
        size_t dest;               //!< The destination of the write operation
        unsigned char *buf;        //!< The data that must be written
        size_t size;               //!< The size of the buffer to be written
    } W_DELAYED_t;

    //! Positions as dynamically allocated array of size_t and pollfd structures
    typedef struct
    {
        struct pollfd * pollfds;   //!< The pollfds entry for each pos
        size_t * pos;              //!< The fd positions for each entry in pollfds
        size_t num;                //!< The number of elements in each of the two 
                                   //!< other arrays
    } ARRAY_FDS_t;

public: // Definitions
    //! Type of notifications sent, if an event occurs within the class
    typedef enum
    {
        DATA_AVAILABLE,            //!< Data is ready to be read from this file source
        SOURCE_CLOSED              //!< This source had to be closed
    } NOTIF_t;

    //! Pointer to a processing function.
    /*! Pointer to a function that processes data read by \ref CComThread
        \param context             : User defined context pointer. May be NULL
        \param source              : Number of the file descriptor the data comes
                                     from
        \param buf                 : The data that has been read by the thread
        \param size                : Number of bytes in buf
        \return                    : The number of bytes processed. Must be
                                     smaller than size. If less bytes are
                                     processed than passed to the function,
                                     the remaining bytes will be passed again
                                     during the next call to the function together
                                     with additional data if available
    */
    typedef size_t (*COM_PROCESS_p) (void const * context, size_t source, unsigned char * buf, size_t size);

    //! Pointer to a timeout function
    /*! Pointer to a function that has to be notified of a \ref CComThread timeout
        \param context             : User defined context pointer. May be NULL
    */
    typedef void (*COM_TIMEOUT_p) (void const * context);

    //! Pointer to an error handling function
    /*! Pointer to a function that has to be notified of a \ref CComThread error
        \param context             : User defined context pointer. May be NULL
        \param operation           : Operation that failed
        \param fdindex             : Number of the file description on
                                     which the operation failed. (-1 if not
                                     linked to a certain source)
        \param correrrno           : errno value linked to the issue
    */
    typedef void (*COM_ERROR_p)   (void const * context, int operation, ssize_t fdindex, int correrrno);

    //! Pointer to a notifying function
    /*! Pointer to a function that notifies a callback that an event has occurred
        on the indicated source.
        \param context             : User defined context pointer. May be NULL
        \param source              : Number of the file descriptor on which the
                                     the event occurred
        \param type                : Type of event that occurred
        \return                      true if the data should be read and passed
                                     to the process function. false if the
                                     data will be handled otherwise.
    */
    typedef bool (*COM_NOTIFY_p)  (void const * context, size_t source, NOTIF_t type);

    //! Structure which configures the \ref CComThread object on \ref start
    /*! Structure which condfigures the \ref CComThread object on \ref start.
        Note that only the functions ending on _delayed are allowed to be
        called from within the callbacks of the same object. Otherwise
        a lockup will occurr
    */
    typedef struct
    {
        COM_PROCESS_p func_process; //!< required: Defines the process function
                                    //!< to which read data must be passed.
                                    //!< Please refer to \ref process for more
                                    //!< information
        COM_TIMEOUT_p func_timeout; //!< optional: Defines the timeout function
                                    //!< that will be called if no data arrives
                                    //!< from any of the defined interfaces
                                    //!< Please refer to \ref timeout for more
                                    //!< information
        COM_ERROR_p func_error;     //!< optional: Defines the error function
                                    //!< which will be called if an error occures
                                    //!< Please refer to \ref error for more
                                    //!< information
        COM_NOTIFY_p func_notify;   //!< optional: Defines the notifying function
                                    //!< which will be called if data is available
                                    //!< at an interface. If not specified, all
                                    //!< all data will be processed.
        unsigned int timeout_ms;    //!< optional: Defines the timeout after
                                    //!< which the timeout function should be
                                    //!< called. A negative number will make
                                    //!< the thread wait indefinitely for data,
                                    //!< while a value of zero will make the
                                    //!< the thread busy poll the passed
                                    //!< interfaces. It is highly recommended
                                    //!< to use a positive number.
        void * context;             //!< optional: This pointer will be passed
                                    //!< as first argument to any of the
                                    //!< callbacks defined above
    } COM_CONFIG_t;

public: // Functions
    //! Default Constructor
    CComThread();
    
    //! Destructor
    //lint -sem(CComThread::stop_no_check,cleanup)
    virtual ~CComThread();

    //! Open the communication interface and start the thread
    bool start(COM_CONFIG_t *config);

    //! Make the thread take care of a new fd provided, when ready
    ssize_t add(int fd);

    //! Remove and close the specified positions file descriptor, when ready
    bool removeAndClose(size_t pos);

    //! Close the communication interface and stop the thread
    bool stop();

    //! Writes data to the defined interface, when ready
    bool writeTo(size_t dest, unsigned char const * buf, size_t size);

    //! Writes data to the defined interfaces
    ssize_t writeTo(size_t *dest, size_t count, unsigned char const * buf, size_t size);

private: // Functions
    //! Copy Cosntructor (Not implemented)
    CComThread(const CComThread &o);
    
    //! Assignment operator (Not implemented)
    const CComThread & operator=(const CComThread &o);

    //! Make the thread take care of a new fd provided
    ssize_t addSync(int fd);

    //! Writes data to the defined interface
    ssize_t writeToSync(size_t dest, unsigned char const * buf, size_t size);

    //! Remove the specified position
    int remove(size_t pos);

    //! Read and process all data of file descriptors under control
    bool readAndProcess();

    //! Handle the delayed adds from \ref add
    bool handleDelayedAdd();

    //! Handle the delayed closes from \ref removeAndClose
    bool handleDelayedClose();

    //! Handle the delayed writes from \ref writeTo
    bool handleDelayedWrite();

    //! Get the pollfd structure. This function is NOT thread safe.
    bool getPollFds(ARRAY_FDS_t *arrFd);

    //! Close without checking for an open interface (used by close and destr.)
    void stop_no_check();

    //! A helper function to create the thread \ref comThread
    static void *bootstrapThread(void *arg);

    //! This function will be executed as the communication thread
    void comThread();

    //! Will call the callback passed during setup, if existing, from the thread
    bool process(size_t source, unsigned char * buf, size_t size);

    //! Will be called from the thread on timeout
    void timeout() const;

    //! Will be called from the thread on error
    void error(int operation, ssize_t fdindex, int correrrno) const;

    //! Notify the creator of this object about available data
    bool notify(size_t source, NOTIF_t type) const;

    //! Set the run flag to the new value
    void setRunFlag(bool newValue);

    //! Get the current run flag value
    bool getRunFlag();

    //! Return a uniq number
    size_t acquireUniqNum();

    //! Set the object to be started or stopped
    void setStarted(bool newValue);

    //! Is the thread up and running?
    bool isStarted();

private: // Variables
    COM_CONFIG_t * _config;          //!< Configuration of this object 

    pthread_t _ref_comThread;        //!< Reference to the object internal thread

    bool _isStarted;                 //!< Indicates if \ref start was succesful
    pthread_mutex_t _isStartedMutex; //!< Avoid _isStarted race conditions

    bool _runFlag;                   //!< Indicates if the thread is running
    pthread_mutex_t _runFlagMutex;   //!< Avoid _fds and _numFds race conditions

    CList<FD_POS_t> _fds;            //!< File descriptor list
    pthread_mutex_t _fdsMutex;       //!< Avoid _fds race conditions

    CList<FD_POS_t> _dAddFds;        //!< File descriptors to be added to the list
    CList<size_t> _dCloseFds;        //!< File descriptors to be removed from the list
    CList<W_DELAYED_t> _dWriteBufs;  //!< Data to write to the position delayed
    pthread_mutex_t _dMutex;         //!< Avoid race conditions for delayed (_d*) operations

    size_t _numFds;                  //!< Number of file descriptors since creation
    pthread_mutex_t _numFdsMutex;    //!< Avoid _fds race conditions
};
#endif //ifndef __UBX_COMTHREAD_H__

