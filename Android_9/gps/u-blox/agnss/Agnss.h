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
 * $Id$
 * $HeadURL$
 *****************************************************************************/

/*! \file
    \ref CAgnss definition

    \brief
    Definition of \ref CAgnss, an abstract class derived from \ref
   CThreadedPacketIf
*/

#pragma once

#include "com/ThreadedPacketIf.h"
#include "func/FuncMngr.h"
#include "helper/UbxAidIni.h"
#include "storage/LockedStorage.h"
#include "storage/PositionHandler.h"
#include "storage/TimeHandler.h"
#include <list>
#include <mutex>
#include <stdarg.h>
#include <unistd.h>

/*! \class CAgnss

    \brief
    Abstract clas implementing \ref CThreadedPacketIf and provides an interface
    for u-blox assistance service implementations

    This class implements the interface described by \ref CThreadedPacketIf.
      This abstract class will handle downloaded data for Online
    and Offline services and will communicate with the receiver
    on one side and the servers on the other (implemented by
    derived classes) in a separate thread.
    Classes derived from this one do not have to be thread safe,
    but are not allowed to run any additional threads and call
    functions implemented by this class functions from there
*/
class CAgnss : private CThreadedPacketIf
{
public: // Definitions of structures, values and types
  //! Default accuracy in seconds of a \ref ACCTIME_t type
  static const size_t DEFAULT_TIME_ACCURACY_S;

  //! Service types provided by \ref CAgnss
  typedef enum {
    TIME = 0,           //!< Time information
    POSITION = 1,       //!< Location information
    RECV_AID_STATE = 2, //!< Aiding state of the receiver
    OFFLINE = 3,        //!< AssistNow Offline data
    ONLINE = 4,         //!< AssistNow Online data
    _NUM_SERVICES_ = 5  //!< Marks the end of valid options
  } SERVICE_t;

  //! Action types provided by \ref CAgnss
  typedef enum {
    TRANSFER = 0,     //!< Transfer data to the receiver
    DOWNLOAD = 1,     //!< Download the information from a server
    POLL_RECV = 2,    //!< Poll the information from the receiver
    _NUM_ACTIONS_ = 3 //!< Marks the end of valid options
  } ACTION_t;

  //! Time sources available to \ref CAgnss
  typedef enum {
    TIME_SYSTEM,  //!< Operating system time
    TIME_INTERNAL //!< Internal time keeping after \ref setCurrentTime
  } TIME_SOURCE_t;

  //! A \ref SERVICE_t / \ref ACTION_t pair
  typedef struct
  {
    SERVICE_t service; //!< Service type
    ACTION_t action;   //!< Action type
  } SERVICE_ACTION_PAIR_t;

  //! A function pointer to a print function
  /*! A printing function pointer used to report status information
      \param context            : User defined variable passed to the function
      \param str                : Information to be printed
      \return                     The number of printed characters on success
                                  and a negative number on failure
  */
  typedef int (*PRINT_FUNC_p)(void const *context, const char *str);

  //! A pointer to a function that has to be signaled of the end of an operation
  /*! A pointer to a function thas has to be notified if a successfully started
      function has finished successfully or unsuccessfully.
      \param context            : User defined variable passed to the function
      \param service            : Which service the action concerned
      \param action             : Which action finished
      \param success            : true if the action finished successfully,
                                  false otherwise
      \param dataId             : A checksum identifying the data used for the
                                  the action
  */
  typedef void (*FINISHED_FUNC_p)(
    void const *context, SERVICE_t service, ACTION_t action, bool success, uint16_t dataId);

  //! Configures \ref CAgnss objects
  typedef struct
  {
    PRINT_FUNC_p func_std_print;            //!< Print default messages using this function
                                            //!< This callback must be thread-safe
                                            //!< and must not block indefinitely
    PRINT_FUNC_p func_err_print;            //!< Print errror messages using this function
                                            //!< This callback must be thread-safe
                                            //!< and must not block indefinitely
    FINISHED_FUNC_p func_transfer_finished; //!< Call this function if a successfully
                                            //!< started action finishes
    WRITE_FUNC_p func_write_to_rcv;         //!< Use this function to communicate with
                                            //!< with the receiver
    TIME_SOURCE_t time_source;              //!< Use this time source for time keeping
    void *context;                          //!< Pass this context on to the functions
    //!< specified by the pointers in this struct
  } CONF_t;

public: // Definitions of functions
  //! Construcor that will be called on creation of the singleton
  CAgnss(CONF_t *conf, char const *com_print_prefix = "");

  //! Default Destructor. Will NOT be able to teardown all of the class!
  virtual ~CAgnss();

  //! Sets up the object for operation
  bool setup();

  //! Stops the thread and deinitalises the object
  bool teardown();

  //! Passes data from the receiver to the thread for processing
  ssize_t processMsg(unsigned char const *buf, size_t size);

  //! Schedules an action for the specified service if possible
  bool scheduleAction(SERVICE_t service, ACTION_t action);

  //! Cleares all the actions and stops the current one
  void clearActions();

  //! Verify the service-action combination is a valid one
  bool isValidServiceAction(SERVICE_t service, ACTION_t action);

  //! Save data for the specified service in the object
  bool saveToDb(SERVICE_t service,
                unsigned char const *data,
                size_t size,
                uint16_t *const data_id = NULL);

  //! Load data for the specified service from the object
  ssize_t loadFromDb(SERVICE_t service, unsigned char **data, uint16_t *const data_id = NULL);

  //! Check if there is data stored in the object for this service
  bool hasData(SERVICE_t service);

  //! Set the current time for aiding
  bool setCurrentTime(ACCTIME_t const *now);

  //! Get the current time for aiding
  bool getCurrentTime(ACCTIME_t *now);

  //! Does the object currently have access to a valid time source?
  bool hasValidTime();

  //! Set the current position for aiding
  bool setCurrentPosition(POS_t const *here);

  //! Get the current position for aiding
  bool getCurrentPosition(POS_t *here);

  //! Does the object currently have access to a valid position?
  bool hasValidPosition();

protected:
  //! A function pointer to processing function
  /*! A printing function pointer used to report status information
      \param data               : The data to be processed
      \param size               : The number of elements in data
      \return                     SUCCESS if the function was successful
                                  FAIL if the function failed
                                  CALL_AGAIN if the function has to be
                                   called at least one other time to
                                   complete
  */
  typedef TRI_STATE_FUNC_RESULT_t (CAgnss::*DATA_FUNC_p)(unsigned char const *data, size_t size);

  //! Print to the standard output
  int print_std(const char *format, ...);

  //! Print to the error output
  int print_err(const char *format, ...);

  //! The function used by libMGA to write to the receiver
  static ssize_t writeToRcv(const void *pContext, unsigned char const *buf, size_t size);
  //! Will write the provided data to the receiver
  ssize_t writeToRcv(unsigned char const *buf, size_t size);

  /////////////////////////////////////////////////////////////////
  // Start function declarations as required to implemet by base //
  /////////////////////////////////////////////////////////////////
  virtual int impl_open(int **fds, int *timeout_ms);
  virtual int impl_close();
  virtual size_t impl_process(size_t source, unsigned char *buf, size_t size);
  virtual void impl_timeout();
  virtual void impl_errorOccurred(int operation, ssize_t fdindex, int correrrno);
  virtual bool impl_notify(size_t source, CComThread::NOTIF_t type);
  ////////////////////////////////////////////////////////////////
  // Stop function declarations as required to implemet by base //
  ////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////
  // Start function declarations to be implemented by the derived class //
  ////////////////////////////////////////////////////////////////////////
  //! Initialises objects of classes dervied from this class
  /*! This pure virtual function must be implemented by derived classes and
      will be called before any other of the functions (except the contructor)
      of the object of the derived class will be called from within the thread
      to the application which has created the object that an action
      has finished.

      \return             'true' if the initialisation was successful.
                          Otherwise 'false'. In the later case the
                          initialisation of the of CAgnss will abort.
  */
  virtual bool impl_init() = 0;

  //! Deinitialises objects of classes dervied from this class
  /*! This pure virtual function must be implemented by derived classes and
      will be called while the interface will be stopped.

      \return             'true' if the deinitialisation was successful.
                          Otherwise 'false'. In the later case the
                          deinitialisation of the of CAgnss will be incomplete
  */
  virtual bool impl_deinit() = 0;

  //! Will be called if the specified action should be started
  /*! This pure virtual function must be implemented by derived classes and
      will be called if the action specified should be initialised for the
      specified service.

      \param service    : The service for which action should be initialised
      \param action     : The action which should be initialised
      \return             Will return a \ref List of \ref CFuncMngr function
                          pointers to be executed until the last of them returns
                          SUCCESS or any previous returns FAIL. On CALL_AGAIN
     the
                          same function will be called again the next time.
  */
  virtual std::list<CFuncMngr> impl_initAction(SERVICE_t service, ACTION_t action) = 0;

  //! Will be called to verify the specified action is a valid one
  /*! This pure virtual function must be implemented by derived classes and
      will be called if the action and service specified should be verified
      to be a valid pair.

      \param service    : The service for which an action should be checked
      \param action     : The action which should be checked
      \return             Will return    true if this is valid action/service
     pair
                          and otherwise false
  */
  virtual bool impl_isValidServiceAction(SERVICE_t service, ACTION_t action) = 0;

  //! Will be called if it has to be verified the passed data is valid
  /*! This pure virtual function will be called to verify the data
      passed is valid for the specified service.

      \param service    : The service for which the data should be valid
      \param buf        : A pointer to the data that should be checked.
                          Must not be NULL
      \param size       : The number of bytes pointed to by buf.
                          Must not be 0
      \return             'true' if the data is valid. 'false' otherwise
  */
  virtual bool impl_isValidData(SERVICE_t service, unsigned char const *buf, size_t size) = 0;
  //! Will be called if the data id of the last operation is required
  /*! This pure virtual function will be called to get the data id of the
      last finished operation has to be known for returning it to the
      creator of this object

      \return             Return the data id of the last transfer finished
  */
  virtual uint16_t impl_getDataId() = 0;
  ///////////////////////////////////////////////////////////////////////
  // Stop function declarations to be implemented by the derived class //
  ///////////////////////////////////////////////////////////////////////
private:
  //! Helper function for \ref print_std/\ref print_err that does the actual job
  int printWithFunc(PRINT_FUNC_p func, const char *format, va_list va);

  ////////////////////////////////////////////////////////////////////////////
  // Start declaration of functions modifying the current state or requests //
  ////////////////////////////////////////////////////////////////////////////
  //! Will check and modify the action status of the object if required
  void stateCheck(unsigned char *buf, size_t size);

  //! Will request an action status check
  bool stateRequest(SERVICE_t service, ACTION_t action);

  //! Will clear the current requests
  void resetState();

  bool havePrintPrefix() const;
  ///////////////////////////////////////////////////////////////////////////
  // Stop declaration of functions modifying the current state or requests //
  ///////////////////////////////////////////////////////////////////////////

  CLockedStorage _data[_NUM_SERVICES_]; //!< The stored online data
  CTimeHandler _time;                   //!< Keeps track of the time
  CPositionHandler _pos;                //!< Keeps track of the position
  std::unique_ptr<CONF_t> _conf;
  int _fd{-1};                                  //!< File descriptor used to
                                                //!< write data to the thread
  std::list<SERVICE_ACTION_PAIR_t> _req_action; //!< The currently requested
                                                //!< actions
  std::list<CFuncMngr> _exec_list{};            //!< commands to be executed
  std::mutex _req_mutex{};                      //!< Protects the _req_*
                                                //!< variables
  std::vector<char> _print_prefix{};            //!< Prefix attached to the
                                                //!< messages being printed
};
