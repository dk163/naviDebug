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
    \ref CMemberFunc definition and implementation

    \brief
    Definition of \ref CMemberFunc, a class which implements the
    \ref CFunc interface and can store a function pointer.
*/

#ifndef UBX_MEMBERFUNC
#define UBX_MEMBERFUNC
#include "Func.h"

#define FREF(a, b, c)                                                                              \
  (CMemberFunc<a>(b, &a::c)) //!< This acts as a wrapper for the constructor of \ref CMemberFunc

/*! \class CMemberFunc
    \brief Definition of \ref CMemberFunc, a class which implements the
    \ref CFunc interface and can store a function pointer to a member function
    of a specific object.
*/
template <class T> class CMemberFunc : public CFunc
{
public: // Definitions
  //! Function pointer type
  typedef TRI_STATE_FUNC_RESULT_t (T::*DATA_FUNC_p)(unsigned char const *data, size_t size);

public: // Functions
  //! Constructor
  CMemberFunc(T *obj, DATA_FUNC_p func);

  //! Destructor
  virtual ~CMemberFunc();

  //! Interface to access the actual function pointer
  virtual TRI_STATE_FUNC_RESULT_t passData(unsigned char const *data, size_t size);

  //! Clone the object
  virtual CFunc *clone() const;

private:             // Variables
  T *_obj;           //!< Object to which the functions beloncs
  DATA_FUNC_p _func; //!< Function pointer
};

/*! Constructor
    \param obj             : Object to which the function pointer belongs
    \param func            : Function pointer
*/
template <class T> CMemberFunc<T>::CMemberFunc(T *obj, DATA_FUNC_p func) : _obj(obj), _func(func) {}

/*! Destructor
*/
template <class T> CMemberFunc<T>::~CMemberFunc()
{
  _obj = NULL;
  _func = NULL;
}

/*! Pass data on to the function pointer
    \param data            : The data that should be processed by the function
    \param size            : The number of elements in data
    \return                  The return value returned by the function pointer.
                             Will be 'FAIL' if this object is not initialized
                             properly
*/
template <class T>
TRI_STATE_FUNC_RESULT_t CMemberFunc<T>::passData(unsigned char const *data, size_t size)
{
  if (!_obj || !_func)
    return FAIL;

  return (_obj->*_func)(data, size);
}

/* Create a copy of the this object
   \return                   A pointer of a new object on success
                             and NULL otherwise
*/
template <class T> CFunc *CMemberFunc<T>::clone() const
{
  return new (std::nothrow) CMemberFunc<T>(_obj, _func);
}

#endif //UBX_MEMBERFUNC
