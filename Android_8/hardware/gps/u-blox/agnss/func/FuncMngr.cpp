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
 * $Id: FuncMngr.cpp 103715 2015-10-06 11:18:18Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/func/FuncMngr.cpp $
 *****************************************************************************/
 
/*! \file
    \ref CFuncMngr implementation

    \brief
    Implementation of \ref CFuncMngr, a wrapper class for pointers to
    classes derived from the abstract \ref CFunc class. Objects
    of this type will remove the requirement to manually handle
    (create / delete) objects of \ref CFunc type.
*/

#include "FuncMngr.h"

/*! Default constructor
*/
CFuncMngr::CFuncMngr()
    : _func(NULL)
{
}

/*! Constructor taking control over the argument
    \param other           : Object which will be handled by this object
                             and which will be destroyed if required
*/
CFuncMngr::CFuncMngr(CFunc *other)
    : _func(other)
{
}

/*! Copy constructor taking control over a copy of the argument
    \param other           : Object which will be copied. This copy
                             will then be handled by this object
                             and will be destroyed if required
*/
CFuncMngr::CFuncMngr(const CFunc &other)
    : _func(NULL)
{
    *this=other;
}

/*! Copy constructor taking control over a copy of the object managed by
    the argument
    \param other           : The object under control of the argument
                             will be copied. This copy will then be
                             handled by this object and will be destroyed
                             if required
*/
CFuncMngr::CFuncMngr(const CFuncMngr &other)
    : _func(NULL)
{
    *this=other;
}

/* Destructor
*/
CFuncMngr::~CFuncMngr()
{
    //lint -sem(CFuncMngr::clear,cleanup)
    clear();
}

/*! Assignment operator taking control over the object assigned to this object.
    \param other           : Object that will be under control of this object
    \return                  A reference to this object
*/
CFuncMngr & CFuncMngr::operator= (CFunc *other)
{
       // protect against self-assignment
    if (_func != other )
    {
        // Clear the old data
        clear();
        _func=other;
    }
    return *this;
}

/*! Assignment operator. This object will take control over a copy of the
    argument.
    \param other           : Object that will be copied. This copy will then
                             be under control of this object
    \return                  A reference to this object
*/
CFuncMngr & CFuncMngr::operator= (const CFunc &other)
{
       // protect against self-assignment
    if (_func != &other)    
    {
        // Clear the old data
        clear();
        _func=other.clone();
    }
    return *this;
}

/*! Assignment operator. This object will take control over a copy of object
    controlled by the argument.
    \param other           : The object controlled by this argument will be
                             copied. This copy will then be under control of
                             this object.
    \return                  A reference to this object
*/
CFuncMngr & CFuncMngr::operator= (const CFuncMngr & other)
{
       // protect against self-assignment
    if (this != &other)    
    {
        // Clear the old data
        clear();
        if( other._func )
        {
            _func=other._func->clone();
        }
    }
    return *this;
}

/*! Does this object have a function-pointer-object under control?
    \return                  true if there is such an object,
                             false otherwise
*/
bool CFuncMngr::isEmpty()
{
    return (_func==NULL);
}

/*! The arguments passed to this function will be handed on
    to the function pointer object under control by this object
    (if existing).
    \param data            : Data that has to be handled by the
                             function pointer object.
    \param size            : Number of elements in data
    \return                  The return status of the function
                             pointer object. Will be FAIL
                             if no function pointer object is
                             under control of this object.
*/
TRI_STATE_FUNC_RESULT_t CFuncMngr::passData( unsigned char const * data
                                           , size_t size )
{
    if(!_func)
        return FAIL;

    return _func->passData(data, size);
}

/*! Destroy the object under control by this object
*/
void CFuncMngr::clear()
{
    delete _func;
    _func=NULL;
}

