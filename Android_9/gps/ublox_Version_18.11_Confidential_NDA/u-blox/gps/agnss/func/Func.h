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
    \ref CFunc definition

    \brief
    Definition of \ref CFunc, an abstract class which is used mostly as a
    polymorphic pointer for derived classes which might use templates and
    contain function pointers.
*/

#ifndef UBX_FUNC
#define UBX_FUNC
#include "../helper/helperTypes.h"

/*! \class CFunc
    \brief Definition of \ref CFunc, an abstract class which is used mostly
    as a polymorphic pointer for derived classes which might use templates and
    contain function pointers.
*/
class CFunc
{
public:
  //! Constructor
  CFunc(){};

  //! Destructor
  virtual ~CFunc(){};

  //! Signature of stored function pointers in derived classes
  virtual TRI_STATE_FUNC_RESULT_t passData(unsigned char const *data, size_t size) = 0;

  //! Clone the function pointer object
  virtual CFunc *clone() const = 0;
};

#endif //UBX_FUNC
