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
    \ref CUbxMsg definition

    \brief
    Definition of \ref CUbxMsg, an abstract class that is used to communicate
    with a receiver to send or receive UBX messages defined by the derived
    classes.
*/
#ifndef __UBXMSG__
#define __UBXMSG__
#include "helperFunctions.h"

/*! \class CUbxMsg

    \brief
    Definition of \ref CUbxMsg, an abstract class that is used to communicate
    with a receiver to send or receive UBX messages defined by the derived
    classes.
*/
class CUbxMsg
{
public: // Functions
  //! Constructor
  CUbxMsg(void *context,
          WRITE_FUNC_p pWrite,
          size_t maxMsg,
          bool autoClear = true,
          bool readyToSendDef = false);

  //! Destructor
  virtual ~CUbxMsg();

  //! Does the object have everything required to send the stored messages
  bool isReadyToSend();

  //! Has the request/command been sent?
  bool hasSentData();

  //! Send the request/command to the receiver
  bool sendData();

  //! Clear the received answer / stored command
  void clearData();

  //! Call on a new message, that has been received by the receiver
  TRI_STATE_FUNC_RESULT_t onNewMsg(unsigned char const *buf, size_t size);

protected: // Functions
  //! Set the object to be ready to send the request / answer
  void setReadyToSend(bool readyToSend);

  //! Helper function to write to the receiver
  ssize_t writeData(unsigned char const *buf, size_t size);

  //! Set the \ref _countMsg counter to 0 again to make \ref onNewMsg work
  virtual void resetCounter();

private: // Functions
  ////////////////////////////////////////////////////////////////////////
  // Start function declarations to be implemented by the derived class //
  ////////////////////////////////////////////////////////////////////////
  //! Will be called by the base to make the object send stored to the receiver
  /*! Will be called by \ref sendData if (\ref isReadyToSend) is true.
            At the end of the implementation of this function \ref writeData
            should be called by the derived class on success.

            \result                  : true on success and false otherwise
        */
  virtual bool impl_writeUbxMsg() = 0;

  //! Will verify if a message received is the required answer
  /*! Messages received from the receiver via \ref onNewMsg will be
            handed to this function for the derived class to decide if
            this is the message from the receiver has provided everything
            required for the operation to succeed / fail.

            \param buf               : A pointer to the data that should
                                       be checked to be a message that finalises
                                       the current operation. May be NULL, in
                                       which case a timeout has occurred
            \param size              : Number of elements in buf. May be 0,
                                       if buf is NULL
            \return                    SUCCESS if the passed message finalises
                                       the current operation successfully,
                                       FAIL if the passed message makes the
                                       function fail and CALL_AGAIN if at least
                                       one other message should be passed to the
                                       function to check if the operation has
                                       finished
        */
  virtual TRI_STATE_FUNC_RESULT_t impl_isExpectedAnswer(unsigned char const *buf, size_t size) = 0;

  //! Will reset the state of the object to the state at creation time
  /*! This function will be called by the base on a \ref clearData call
            and should make object of the derived class restore the state
            of the object at creation time.
        */
  virtual void impl_clearData() = 0;
  ///////////////////////////////////////////////////////////////////////
  // Stop function declarations to be implemented by the derived class //
  ///////////////////////////////////////////////////////////////////////

private:                      // Variables
  void *_context;             //!< The context passed to the constructor
  WRITE_FUNC_p _write;        //!< Pointer to the write function provided
  bool _dataSent;             //!< Defines if data has been sent already
  bool _readyToSend;          //!< Defines if the object is ready to sent
  size_t _countMsg;           //!< Messages parsed in on \ref onNewMsg
  const bool _autoClear;      //!< Defines if \ref clearData should be
                              //!< called by the object automatically
                              //!< if an operation has finished
  const bool _readyToSendDef; //!< Default value for \ref _readyToSend
  const size_t _maxMsg;       //!< Maximum number of messages parsed
                              //!< before returning an error
};
#endif //__UBXMSG__
