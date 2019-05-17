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
    \ref CUbxCfgNavX5 definition

    \brief
    Definition of \ref CUbxCfgNavX5, a class to send a UBX-CFG-NAVX5 message
    to the receiver.
*/
#ifndef __UBXNAVX5__
#define __UBXNAVX5__
#include "UbxMsg.h"
#include "helperFunctions.h"

/*! \class CUbxCfgNavX5

    \brief
    Definition of \ref CUbxCfgNavX5, a class derived from \ref CUbxMsg,
    which is used to configure the navigation engine expert settings.This
    version does only allow to enable acknowledgment of aiding messages
    in the receiver.
*/
class CUbxCfgNavX5 : public CUbxMsg
{
public: // Functions
  //! Constructor
  CUbxCfgNavX5(void *context, WRITE_FUNC_p pWrite, size_t maxMsg, bool autoClear = true);

  //! Destructor
  virtual ~CUbxCfgNavX5();

  //! Configure the object to enable or disable ACKs/NAKs for aiding messages
  bool enableAidingAck(bool enable);

private: // Functions
  /////////////////////////////////////////////////////////////////
  // Start function declarations as required to implemet by base //
  /////////////////////////////////////////////////////////////////
  bool impl_writeUbxMsg();
  TRI_STATE_FUNC_RESULT_t impl_isExpectedAnswer(unsigned char const *buf, size_t size);
  virtual void impl_clearData();
  ////////////////////////////////////////////////////////////////
  // Stop function declarations as required to implemet by base //
  ////////////////////////////////////////////////////////////////

  //! Send UBX-CFG-NAVX5 message
  bool writeUbxCfgNavX5(bool enable);

private:              // Variables
  bool _enableAidAck; //!< If aiding of ACKs/NAKs be enabled or disabled
};
#endif //__UBXNAVX5__
