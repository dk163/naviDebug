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
 * $Id: FuncMngr.h 103715 2015-10-06 11:18:18Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/func/FuncMngr.h $
 *****************************************************************************/
 
/*! \file
    \ref CFuncMngr definition 

    \brief
    Definition of \ref CFuncMngr, a wrapper class for pointers to
    classes derived from the abstract \ref CFunc class. Objects
    of this type will remove the requirement to manually handle
    (create / delete) objects of \ref CFunc type.
*/

#ifndef UBX_FUNCMNGR
#define UBX_FUNCMNGR
#include "Func.h"

/*! \class CFuncMngr
    \brief A wrapper class for pointers to
    classes derived from the abstract \ref CFunc class. Objects
    of this type will remove the requirement to manually handle
    (create / delete) objects of \ref CFunc type.
*/
class CFuncMngr
{
    public: // Functions
        //! Default constructor
        CFuncMngr();

        //! Constructor taking control over the argument
        CFuncMngr(CFunc *other);

        //! Copy constructor
        CFuncMngr(const CFunc &other);

        //! Copy constructor
        CFuncMngr(const CFuncMngr &other);

        //! Destructor
        //lint -sem(CFuncMngr::clear,cleanup)
        virtual ~CFuncMngr();

        //! Assignment operator, taking control over the argument
        CFuncMngr & operator= (CFunc *other);

        //! Assignment operator, copying the argument
        CFuncMngr & operator= (const CFunc &other);

        //! Assignment operator, copying the argument
        CFuncMngr & operator= (const CFuncMngr &other);

        //! Delete the object under control
        void clear();

        //! Does this object control a \ref CFunc object?
        bool isEmpty();

        //! Pass data on to the stored function pointer object
        virtual TRI_STATE_FUNC_RESULT_t passData(unsigned char const * data, size_t size);

    private:
        CFunc *_func;            //!< The function pointer object under control
};

#endif //UBX_FUNCMNGR

