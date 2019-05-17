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
 * $Id: Agnss.cpp 105975 2015-11-13 09:31:03Z marcus.picasso $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/Agnss.cpp $
 *****************************************************************************/
 
/*! \file
    AGNSS interface implementation.

    \brief

    Selected functions of the abstract receiver interface class CAgnss
    are implemented here.
*/

#include <errno.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "Agnss.h"
#include "storage/LockedStorage.h"
#include "helper/helperFunctions.h"

const size_t CAgnss::DEFAULT_TIME_ACCURACY_S=20;

/*! Creates a object of the CAgnss class type and initialises
    the ressources required during \ref setup

    \param conf        : A pointer to a configuration structure
                         of type \ref CONF_t. The content
                         this parameter will be copied. The struct
                         has only to exist during the call to this
                         function. The pointers contained in it,
                         will however have to be accessible during
                         during the whole lifetime of the object.
                         Must not be NULL
   \param print_prefix : A zero-terinated prefix that will be used
                         during logging. Again, the parameter will
                         be copied and the passed string does only
                         need to exist during the call to this
                         function. May be NULL.
*/
CAgnss::CAgnss(CONF_t *conf, char const * print_prefix)
    : CThreadedPacketIf()
    , _time(_data[TIME])
    , _pos(_data[POSITION], _time)
    , _conf(NULL)
    , _fd(-1)
    , _print_prefix(NULL)
{
    // If these memory allocations go wrong or invalid arguments
    // were passed setup() is going to complain about that.
    if(conf)
    {
        _conf=(CONF_t *) malloc(sizeof(*conf));
        if(_conf)
        {
            memcpy(_conf, conf, sizeof(*_conf));
            // If the systme time is trustworhty
            // something has to be stored in the
            // data container never the less,
            // to make sure hasData() does not
            // complain about that for time
            if(_conf->time_source==TIME_SYSTEM)
            {
                ACCTIME_t tmpAccNow;
                memset(&tmpAccNow, 0, sizeof(tmpAccNow));
                tmpAccNow.valid=true;
                tmpAccNow.time.tv_sec=getEarliestSaneTime();
                tmpAccNow.acc.tv_sec=DEFAULT_TIME_ACCURACY_S; // For completeness
                _time.injectTime(&tmpAccNow);
                assert(hasData(TIME));
            }
        }
    }
    _print_prefix = print_prefix ?
                      strdup(print_prefix)
                      : strdup("");
    pthread_mutex_init(&_req_mutex, NULL);
}

/*! Destructor which will teardown the part of the object
    that exists over the lifetime of the object and can be
    destroyed like this. It is crucial that any derived
    class calls \ref teardown in its destructor!
*/
CAgnss::~CAgnss()
{
    free(_conf);
    free(_print_prefix);
    pthread_mutex_destroy(&_req_mutex);
}

/*! This function will start the thread for processing the
    data coming from the receiver if the data that was passed
    to the of this class is valid and it is not already running.

    \return             'true' on success. 'false' otherwise
*/
bool CAgnss::setup()
{
    bool result=false;
    if(!isOpen() && _conf && _print_prefix)
    {
        _fd=this->open();
        result=(_fd>=0);
    }
    return result;
}

/*! This function will start stop the thread responsible
    for processing the data coming from the receiver. It
    is the inverse operation to \ref setup.

    \return             'true' on success. 'false' otherwise
*/
bool CAgnss::teardown()
{
    bool result=true;
    _fd=-1;
    if(isOpen())
    {
        result=(this->close()==0);
    }
    return result;
}

/*! This function will pass the data passed to the thread for
    processing. It is required to successfully call \ref setup
    before being able to call this function.
    
    \param buf        : A pointer to the data that should be
                        forwarded to thread for processing.
                        Must not be NULL.
    \param size       : The size of the array buf points to in
                        bytes. Must not be 0.
    \return             The number of bytes written on success
                        and -1 on failure.
*/ ssize_t CAgnss::processMsg(unsigned char const * buf, size_t size)
{
    ssize_t result=-1;
    if(isOpen() && buf && size)
    {
        result=write(_fd, buf, size);
    }
    return result;
}

/*! This function will initiate the specified action for
    the specified service in a separate thread - if possible.
    It is possible to schedule 10 other actions while one
    is already executed.
    
    \param service    : The service for which the action should be executed
    \param action     : The action which should be scheduled
    \return             'true' if the action could be schedules.
                        'false' otherwise
*/
bool CAgnss::scheduleAction(SERVICE_t service, ACTION_t action)
{
    bool result=false;
    if(isOpen() && isValidServiceAction(service, action))
    {
        result=stateRequest(service, action);
    }
    if(result)
    {
        print_std("Successfully scheduled %s of %s data", agnssActionToString(action)
                                                        , agnssServiceTypeToString(service));
    }
    else
    {
        print_err("Could not schedule %s of %s data", agnssActionToString(action)
                                                    , agnssServiceTypeToString(service));
    }
    return result;
}

/*! This function will clear the list of scheduled actions
    and stop the current one. No calls to the finished-action
    callback will be executed.
*/
void CAgnss::clearActions()
{
    if(isOpen())
    {
        resetState();
    }
}

/*! This function will check if the passed service-action combination is valid

    \param service    : The service for which it is checked if the 
                        action is valid.
    \param action     : The action for which it is checked if the service
                        is valid.
    \return             true if valid, false otherwise
*/
bool CAgnss::isValidServiceAction(SERVICE_t service, ACTION_t action)
{
    return impl_isValidServiceAction(service, action);
}

/*! This function will save the provided data in the object
    after verifying its validity for the specified service.
    This call will overwrite any data already saved or downloaded
    for the specified service, if successful. This function can
    be called before \ref setup has been executed.

    \param service    : The service for which the data should be saved
    \param data       : A pointer to the data which should be saved.
                        Must not be NULL
    \param size       : Size of the data passed for saving in bytes.
                        Must not be 0
    \param data_id    : Pointer to the variable that will be set to
                        the checksum of the data saved (only on success).
                        This can be used to identify the data
    \return             'true' if the data could be saved.
                        'false' otherwise
*/
bool CAgnss::saveToDb( SERVICE_t service
                     , unsigned char const *data
                     , size_t size
                     , uint16_t * const data_id/*=NULL*/)
{
    bool result=false;
    if(impl_isValidData(service, data, size))
    {
        result=_data[service].set(data, size, data_id);
    }
    return result;
}

/*! This function will load the data for the specified service currently
    stored in the object. This function can be called before \ref setup
    has been executed.

    \param service    : The service from which the data should be loaded
    \param data       : The pointer which should be set to point to 
                        a copy of the data for the specified service
                        by the function on success. Must not be NULL
    \param data_id    : Pointer to the variable that will be set to
                        the checksum of the data loaded (only on success).
                        This can be used to identify the data
    \return             The number of bytes stored in data on success
                        and a negative error code on failure
*/
ssize_t CAgnss::loadFromDb( SERVICE_t service
                          , unsigned char **data
                          , uint16_t * const data_id)
{
    return _data[service].getCopy(data, data_id);
}

bool CAgnss::hasData(SERVICE_t service)
{
    return _data[service].hasData();
}

/*! Set the current time for aiding in the time handler
    \param accNow     : The current time. Must not be NULL and the valid
                        flag must be set. If no accuracy is defined, a
                        default accuracy of \ref DEFAULT_TIME_ACCURACY_S
                        seconds will be assumed.
    \return             true on success and false otherwise
*/
bool CAgnss::setCurrentTime(ACCTIME_t const * accNow)
{
    bool result=false;
    if( _conf
     && _conf->time_source == TIME_INTERNAL
     && accNow
     && accNow->valid)
    {
        ACCTIME_t tmpAccNow;
        memcpy(&tmpAccNow, accNow, sizeof(tmpAccNow));
        // Adjust leapseconds if not done
        deductLeapSec(&tmpAccNow);

        // If no accuracy is defined, define a default one
        if(!tmpAccNow.acc.tv_sec && !tmpAccNow.acc.tv_nsec)
            tmpAccNow.acc.tv_sec=DEFAULT_TIME_ACCURACY_S;

        // Normalize times
        normalizeTime(&tmpAccNow.time);
        normalizeTime(&tmpAccNow.acc);
        char timeS[20];
        char accS[20];
        struct tm t;
        strftime(timeS, 20, "%Y.%m.%d %H:%M:%S", gmtime_r(&tmpAccNow.time.tv_sec, &t));
        strftime(accS, 20, "%H:%M:%S", gmtime_r(&tmpAccNow.acc.tv_sec, &t));
        GPS_UBX_AID_INI_U5__t Payload;

        if(createUbxAidIni(&Payload, &tmpAccNow))
        {
            print_std("The internal time has been set to:"
                      " %s.%.9d (GPS: %uwn:%09lums:%09lins)"
                      " Accuracy: %s.%.9d"
                      , timeS
                      , tmpAccNow.time.tv_nsec
                      , Payload.wn
                      , Payload.tow
                      , Payload.towNs
                      , accS
                      , tmpAccNow.acc.tv_nsec);
            result=_time.injectTime(&tmpAccNow);
        }
    }

    return result;
}

/*! Get the current (trusted) time reference. If no trustworthy time
    is available false will be returned.
    \param now        : On success it will be set to the current
                        time.
    \return             true if successful. false otherwise
*/
bool CAgnss::getCurrentTime(ACCTIME_t * now)
{
    bool result=false;
    if(_conf)
    {
        if(_conf->time_source==TIME_SYSTEM)
        {
            ACCTIME_t tmpAccTime;
            memset(&tmpAccTime, 0,  sizeof(tmpAccTime));
            clock_gettime(CLOCK_REALTIME, &tmpAccTime.time);
            tmpAccTime.acc.tv_sec=DEFAULT_TIME_ACCURACY_S;
            tmpAccTime.valid=true;
            // Assume that at least some of the leapseconds are
            // already part of the system time
            tmpAccTime.leapSeconds=true;
            if(isValidAccTime(&tmpAccTime))
            {
                // Is actually a copy of the time requested?
                // If yes copy.
                if(now)
                {
                    memcpy(now, &tmpAccTime, sizeof(*now));
                }
                result=true;
            }
        }
        else
        {
            //Only request the time if it is actually required
            if(now)
            {
                result=_time.getCurrentTime(now);
            }
            else
            {
                result=_time.hasValidTime();
            }
        }
    }
    return result;
}

/*! Does the object currently have access to a valid time source? The return
    value of this parameter depends on if the configuration was set to use
    the system time for aiding (which is assumed to be always availble) or if
    the object keeps track of the time itself, which requires that the
    \ref setCurrentTime function has been called prior to the call to
    this function either from within the object or from somewhere else.

    \return           : true if a time source is available. Otherwise false    
*/
bool CAgnss::hasValidTime()
{
    return getCurrentTime(NULL);
}

/*! Does the object currently have valid position information?
    The object keeps track of the position accuracy and validity itself,
    which requires besides position information, valid time as well.
    The \ref setCurrentPosition function has been called prior to the call
    to this function either from within the object or from somewhere else.

    \return           : true if a time source is available. Otherwise false    
*/
bool CAgnss::hasValidPosition()
{
    bool result=false;
    if( _conf
     && hasValidTime() )
        result=_pos.hasValidPosition();

    return result;
}

/*! Set the current position

    \param here       : The current position
    \return             true on success and false otherwise
*/
bool CAgnss::setCurrentPosition(POS_t const * here)
{
    return _pos.injectPosition(here);
}

/*! Get the current position

    \param here       : Where the current position will be stored on success
    \return             true on success and false otherwise
*/
bool CAgnss::getCurrentPosition(POS_t * here)
{
    return _pos.getCurrentPosition(here);
}

/*! This function will print messages to the default output
    as defined by the callback functions passed to the
    constructor and including the specified prefix.
    \param format     : Format of the output identical
                        to printf and other functions defined
                        by stdio.h    
    \param ...        : The previous argument describes the
                        format of the output. This variable
                        number of additional arguments are
                        converted for output according to
                        that rules.
    \return             Returns the value that is returned by
                        the callback function.
*/
int CAgnss::print_std(const char *format, ...)
{
    int result = -1;

    if(_conf)
    {
        va_list args;
        va_start(args, format);
        result = printWithFunc(_conf->func_std_print, format, args);
        va_end(args);
    }

    return result;
}

/*! This function will print messages to the error output
    as defined by the callback functions passed to the
    constructor and including the specified prefix.
    \param format     : Format of the output identical
                        to printf and other functions defined
                        by stdio.h    
    \param ...        : The previous argument describes the
                        format of the output. This variable
                        number of additional arguments are
                        converted for output according to
                        that rules.
    \return             Returns the value that is returned by
                        the callback function.
*/
int CAgnss::print_err(const char *format, ...)
{
    int result = -1;

    if(_conf)
    {
        va_list args;
        va_start(args, format);
        result = printWithFunc(_conf->func_err_print, format, args);
        va_end(args);
    }

    return result;
}

/*! This static function will be called by used libaries to write data to
    the receiver. This function will then actually call the function
    belonging to the object to handle this.

    \param context    : Must contain the 'this' pointer to the actual
                        actual object on which the function of the same name
                        without this parameter will be executed.
    \param buf        : Will be passed to the actual function of the object
    \param size       : Will be passed to the actual function of the object
    \return             
*/
ssize_t CAgnss::writeToRcv( const void *context
                          , unsigned char const *buf
                          , size_t size )
{
    ssize_t result=-1;
    if(context)
    {
        result= ((CAgnss *) const_cast<void *>(context))->writeToRcv(buf, size);
    }
    return result;
}

/*! This function must be used by derived classes to pass data
    to the receiver.

    \param buf        : A pointer to the data that should be sent to
                        to the receiver. Must not be NULL
    \param size       : The size of the data pointed at by buf in bytes.
                        Must not be NULL
    \return             The number of bytes written to the receiver on success
                        and a negative error code on failure
*/
ssize_t CAgnss::writeToRcv(unsigned char const *buf, size_t size)
{
    ssize_t result=-1;
    if(isOpen() && _conf && _conf->func_write_to_rcv)
        result=_conf->func_write_to_rcv((void const *) _conf->context, buf, size);

    return result;
}

/*! Implements the purely virtual function from the base
    and indicates to the base that no further interfaces
    are required. If the parmeters are valid (non-NULL)
    the \ref impl_init function will be called and
    *fds and *timeout_ms will be assigned with the
    correct values.

    \param fds        : Is not allowed to be NULL and the
                        pointer to which the address is
                        pointing to will be assigned NULL.
    \param timeout_ms : Is not allowed to be NULL and
                        variable to which the address is
                        pointing to will be assigned 1000.
    \return             On success 0 will be returned,
                        otherwise -1
*/
int CAgnss::impl_open(int **fds, int *timeout_ms)
{
    if(!fds || !timeout_ms)
        return -1;

    int result=-1;
    *fds=NULL;

     // One second timeout ought 
     // to be enough for everyone
    *timeout_ms=1000;
    if(impl_init())
        result=0;

    return result;
}

/*! Implements the purely virtual function from the base 
    and will call \ref impl_deinit

    \return            Will return 0 on success and -1 otherwise 
*/
int CAgnss::impl_close()
{
    return impl_deinit()?0:-1;
}

/*! Implements the purely virtual function from the base and
    processes within the thread setup by the base 
    (only) data coming from the non-threaded user side of the
       interface implemented as \ref processMsg call. This
    function will also use \ref stateCheck to manipulate and
    test the internal states and pass the information on
    to the functions of the derived class

    \param source   : Specifies which file descriptor the data has been
                      retrieved. Only a value of 0 will lead to any
                      action.
    \param buf      : The data that must be processed. May be NULL
    \param size     : Size of buf in bytes. May be 0 if buf is NULL
    \return           The number of bytes processed. This will always
                      be exactly size for this implementation.
*/
size_t CAgnss::impl_process(size_t source, unsigned char * buf, size_t size)
{
    assert(source==0); ((void)(source));
    // Check if any state changes / actions
    // are required and pass the data to
    // the functions defined by the
    // derived class in case CAgnss is still
    // in a mode that requires the 
    // derived class to process messages
    stateCheck(buf, size);
    // Never pass
    return size;
}

/*! Will be called if a timeout has occured waiting for data from the 
    file descriptors defined during specified interfaces and will
    call \ref impl_process with all 0 or NULL parameters.
*/
void CAgnss::impl_timeout()
{
    impl_process(0, NULL, 0);
}

/*! Will implement the error handling as defined by the base
    and print out corresponding error messages
*/
void CAgnss::impl_errorOccurred(int operation, ssize_t fdindex, int correrrno)
{
    switch(operation)
    {
        case COMTHREAD_ERR_READ:
        {
            print_err("An error occured while trying to read from file"
                      " descriptor index %i:"
                      " '%s'", fdindex, strerror(correrrno));
            break;
        }
        case COMTHREAD_ERR_PROC:
        {
            print_err("Processing data received from"
                      " file descriptor index %i failed:"
                      " %s", fdindex, strerror(correrrno));
            break;
        }
        case COMTHREAD_ERR_POLL:
        {
            print_err("An error occured"
                      " while waiting for data input from"
                      " the managed interfaces. Trying again."
                      " Error description: %s"
                      , strerror(errno));
            break;
        }
        default:
        {
            print_err("An unknown error occured in the"
                      " processing thread!");
        }
    }
}

/*! Will implement the notification handling as defined by the base.
    This implementation will require all messages to be processed
*/
bool CAgnss::impl_notify(size_t source, CComThread::NOTIF_t type)
{
    ((void) source);
    ((void) type);
    return true;
}

/*! This function will pass the message passed including the
    specified prefix on to the passed function pointer
    \param func       : Function to which the constructed message string
                        will be passed
    \param format     : Format of the output identical
                        to printf and other functions defined
                        by stdio.h    
    \param va         : List of arguments described by the
                        format of the output. This variable
                        number of additional arguments are
                        converted for output according to
                        that rules.
    \return             Returns the value that is returned by
                        the callback function.
*/
int CAgnss::printWithFunc(PRINT_FUNC_p func, const char *format, va_list va)
{
    int result = -1;

    if(func && _conf)
    {
        char *buf;
        char *new_format=NULL;
        int res=0;
        if(_print_prefix)
        {
            res = asprintf(&new_format, "%s%s", _print_prefix, format);
        }
        if(res>=0)
        {
            res = vasprintf(&buf, new_format?new_format:format, va);

            if(res >= 0)
            {
                result = func((void const *) _conf->context, buf);
                free(buf);
            }
            free(new_format);
        }
    }
    return result;
}

/*! This function will modify the status regarding current action
    and service as well as the scheduled actions if required based
    on the status of the current action. This function is immediately
       called to pass new messages or timeouts (in which case the parameters
    will be NULL, respectively 0) on to functions
    selected by the derived classes. To get these functions
    \ref impl_initAction will be called.
    This function should carry out the steps necessary to initialise
    action by calling \ref impl_initAction. If the current action
    finishes, this will be handled as well.

    \param buf      : The data that must be processed. May be NULL
    \param size     : Size of buf in bytes. May be 0 if buf is NULL
*/
void CAgnss::stateCheck(unsigned char * buf, size_t size)
{

    CFuncMngr func;
    bool action_finished=false;
    bool success=false;

    SERVICE_ACTION_PAIR_t curr;
    memset(&curr, 0, sizeof(curr));
    ////////////////////
    // START LOCKED PART
    ////////////////////
    pthread_mutex_lock(&_req_mutex); // LOCK
    // Is there still something to do for a
    // current job?
    if(!_exec_list.front(&func))
    {
        // No. Get a new one then if one is available
        if(_req_action.front(&curr))
        {
            // Get new commands
            print_std("Starting %s %s!"
                  , agnssServiceTypeToString(curr.service)
                  , agnssActionToString(curr.action));
            _exec_list=impl_initAction(curr.service, curr.action);
            // Get current command. If this fails, report error
            if(!_exec_list.front(&func))
            {
                print_err("Starting %s %s failed!"
                      , agnssServiceTypeToString(curr.service)
                      , agnssActionToString(curr.action));
                // Remove failed task
                _exec_list.clear();
                func.clear();
                // Get all the required status information
                _req_action.pop(&curr);
                action_finished=true;
                success=false;
            }
        }
    }

    // execute command
    if(!func.isEmpty())
    {
        TRI_STATE_FUNC_RESULT_t funcRes=func.passData(buf, size);
        // Evaluate result
        if(funcRes!=CALL_AGAIN)
        {
            // If the action failed, we will finish
            // here in any case.
            if(funcRes==FAIL)
            {
                _exec_list.clear();
            }
            else // funcRes==SUCCESS
            {
                // Remove the current function only
                _exec_list.pop();
                success=true;
            }

            // Did this finish the whole set of commands
            // and therefore the complete action?
            if(!_exec_list.getSize())
            {
                // Yes. Get the required status information
                _req_action.pop(&curr);
                action_finished=true;
            }
        }
    }
    pthread_mutex_unlock(&_req_mutex); // UNLOCK
    ////////////////////
    // END LOCKED PART
    ////////////////////

    // Message the end of the action if reqired
    if(action_finished)
    {
        if(success)
        {
            print_std("%s %s finished successfully!" , agnssServiceTypeToString(curr.service)
                                                     , agnssActionToString(curr.action));
        }
        else
        {
            print_err("%s %s finished unsuccessfully! Action failed!" , agnssServiceTypeToString(curr.service)
                                                                      , agnssActionToString(curr.action));
        }
        _conf->func_transfer_finished( (void const *) _conf->context
                                     , curr.service
                                     , curr.action
                                     , success
                                     , impl_getDataId());
    }
}

/*! This function will list a new request an action
    for a specific service to the system as
    long as there are less than 10 requests already pending
    \ref stateCheck will then modify the actual status.

    \param service       : The service for which an action is
                           requested
    \param action        : The action that should be listd
    \return               'true' if the request could be placed.
                          'false' if the list is full
*/
bool CAgnss::stateRequest(SERVICE_t service, ACTION_t action)
{
    bool result=false;
    pthread_mutex_lock(&_req_mutex);
    if(_req_action.getSize() <= 10)
    {
        SERVICE_ACTION_PAIR_t tmpReq;
        tmpReq.service=service;
        tmpReq.action=action;
        _req_action.push(tmpReq);
        result=true;
    }
    pthread_mutex_unlock(&_req_mutex);
    return result;
}

/*! This function will indicate to the state handler
    that all current and scheduled actions should
    be aborted.
*/
void CAgnss::resetState()
{
    pthread_mutex_lock(&_req_mutex);
    _exec_list.clear();
    _req_action.clear();
    pthread_mutex_unlock(&_req_mutex);
}

