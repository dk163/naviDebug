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
 * $Id: ThreadedPacketIf.h 104538 2015-10-20 14:48:22Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/com/ThreadedPacketIf.h $
 *****************************************************************************/
 
/*! \file
    \ref CThreadedPacketIf declaration.

    \brief
    Contains the \ref CThreadedPacketIf declarations
    The \ref CThreadedPacketIf class is declared in this file
    Please refer to \ref ThreadedPacketIf.h for further information 
*/
#ifndef __UBX_IO_IF_H__
#define __UBX_IO_IF_H__
#include "ComThread.h"
#include "../helper/helperTypes.h"

/*! \class CThreadedPacketIf
    \brief Abstract communication class with packet datagram interface

    An abstract class that provides means for a derived class to
    communicate between a main application and several other data sources
    specified by file descriptors in a separate thread without having
    to care about setting everything up properly. This class makes
    use of the class \ref CComThread for handling the file descriptors
    in a separate class and is thread safe. The derived class has
    not to take care about any thread safety issues, as long as it
    does not create any threads on its own or communicates
    with the creator of the object.
*/
class CThreadedPacketIf
{
public: // Functions
    //! Constructor which must be called by derived classes
    CThreadedPacketIf();

    //! Destructor. Will teardown most of the object
    virtual ~CThreadedPacketIf();

    //! Opens the packet interface and starts the communication thread
    int open();

    //! Close the interface
    int close();

    //! Has open been called successfully?
    bool isOpen();
protected: // Functions

    //! Make the thread take care of a new fd provided
    ssize_t add(int fd);

    //! Remove the specified position and close the corresponding file descriptor
    bool removeAndClose(size_t pos);

    //! Wrapper for \ref CComThread::writeTo
    ssize_t writeTo(size_t dest, unsigned char const * buf, size_t size);

    ///////////////////////////////////////////////////////////
    // Start Declaration of functions accessed by CComThread //
    ///////////////////////////////////////////////////////////

    //! \ref CComThread passes data to be processed to this function
    static size_t process( void const * context
                         , size_t source
                         , unsigned char * buf
                         , size_t size );

    //! \ref CComThread passes data to be processed to this function
    size_t process(size_t source, unsigned char * buf, size_t size);

    //! Indicate that a timeout has occurred in the \ref CComThread
    static void timeout(void const * context);

    //! Indicate that a timeout has occurred in the \ref CComThread
    void timeout();

    //! Will report an error to the derived class that occured in \ref CComThread
    static void errorOccurred( void const * context
                             , int operation
                             , ssize_t fdindex
                             , int correrrno );

    //! Will report an error to the derived class that occured in \ref CComThread
    void errorOccurred(int operation, ssize_t fdindex, int correrrno);

    //! Notify the creator of this object about available data
    static bool notify(void const * context, size_t source, CComThread::NOTIF_t type);

    //! Notify the creator of this object about available data
    bool notify(size_t source, CComThread::NOTIF_t type);

    //////////////////////////////////////////////////////////
    // Stop Declaration of functions accessed by CComThread //
    //////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////
    // Start Definition of functions implemented by derived classes //
    //////////////////////////////////////////////////////////////////

    //! Called on \ref open and provides the additional interfaces 
    /*! This function will be called in the process of opening the
        the packet interface of this thread. All the file descriptors
        provided by this function will be handled internally of this
        object.

        \param fds        : All the interfaces the object created from
                            the derived class would like to retrieve and
                            process data from must be stored in a newly
                            allocated array of file descriptors. This
                            argument to this function will be an address
                            to the pointer which must point to the array
                            of file descriptors after a successful call.
                            The calling function will free the ressource
                            on success when the array is not longer
                            required. If the call was not successful.
                            Can be set to NULL, if no interface should
                            be handled other than the packet interface.
        \param timeout_ms : The value of the variable of which the address
                            is passed this function can be set by the 
                            called function to the value in milliseconds
                            after which the \ref impl_timeout function
                            should be called if no data arrives from
                            any interface.
        \return             On success a value >=0, indicating the
                            number of elements in fds. On error, return
                            a negative number.
    */
    virtual int impl_open(int **fds, int *timeout_ms) = 0;

    //! Called on \ref close
    /* This function will allow the part of the object that is defined
       in the derived class to teardown ressources used at that level.
       Particularly closing the file descriptors provided during the
       call to \ref impl_open (if any).

       \reurn             : Returns 0 on success and -1 otherwise.
    */
    virtual int impl_close() = 0;

    //! Called on \ref process for processing of data retrieved
    /*! This function will be called if any data could be retrieved
        from a specified file descriptor. 
        \param source              : Specifies which file descriptor the data has
                                     been retrieved. The argument itself is NOT
                                     the actual file descriptor, but the index+1
                                     the file descriptor retrieved during the
                                     call to \ref impl_open. The first index
                                     (index 0) refers to the packet
                                     interface always created by \ref open.
        \param buf                 : The data that must be processed. Will not be
                                     NULL
        \param size                : Size of buf in bytes. Will not be 0
        \return                    : The number of bytes processed. Must be
                                     smaller than size. If less bytes are
                                     processed than passed to the function,
                                     the remaining bytes will be passed again
                                     during the next call to the function
                                     together with additional data if
                                     available
    */
    virtual size_t impl_process(size_t source, unsigned char * buf, size_t size)=0;

    //! Called on \ref timeout 
    /*! Will be called if a timeout has occured waiting for data from the 
        file descriptors defined during specified interfaces. 
    */
    virtual void impl_timeout()=0;

    /*! Will be called by \ref CComThread from the thread if an error has occured.
        It will, on the other hand, call \ref impl_errorOccurred, implemented by
        a derived class, to pass this information on to the relevant part of object.

        \param operation :  Will indicate which operation led to
                            the error. Valid values are
                            COMTHREAD_ERR_POLL
                            COMTHREAD_ERR_READ
                            COMTHREAD_ERR_MEMORY
        \param fdindex   :  Indicates which of the file descriptors
                            was involved during the error. If an
                            operation failed which does not relate
                            to a specific file descriptor, -1 will
                            be passed.
        \param correrrno :  The errno value that was set during the
                            operation that caused the error
    */
    virtual void impl_errorOccurred(int operation, ssize_t fdindex, int correrrno)=0;


    /*! This function will be called from the thread if data is available
        from a source.

        \param source              : Number of the file descriptor from which
                                     data is available
        \param type                : Type of event that occurred
        \return                      true if the data should be read and passed
                                     to the \ref impl_process function. false if
                                     the data will be handled otherwise.
    */
    virtual bool impl_notify(size_t source, CComThread::NOTIF_t type)=0;
    /////////////////////////////////////////////////////////////////
    // Stop Definition of functions implemented by derived classes //
    /////////////////////////////////////////////////////////////////
private: // Functions
    //! Close the interface wihthout checking if it is open
    int close_no_check();

private: // Variables
    CComThread * _com;       //!< Reference to the threading thread
    bool _isOpen;            //!< Indicate if the packet interface is open
    int _fd_packet_usr;      //!< File descriptor on the object creator side
                             //!< used to pass data to the actual application
                             //!< for reading / writing data from / to the
                             //!< \ref CComThread side
    int _fd_packet_thread;   //!< Used on \ref CComThread side to write and read
                             //!< data from the user side of the packet interface
};
#endif //ifndef __UBX_IO_IF_H__
