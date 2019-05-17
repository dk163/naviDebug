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
 * $Id: List.h 103715 2015-10-06 11:18:18Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/list/List.h $
 *****************************************************************************/
 
 /*! \file
    \ref CList definition and implementation

    \brief
    Definition and implementation of \ref CList, a class that stores entries
    of a specified type as a doubly-linked list.
*/

#ifndef UBX_LIST
#define UBX_LIST
#include <stdlib.h>
#include <new>
#include <assert.h>
#include <pthread.h>

/*! \class CList
    \brief Stores data of a type defined at object creation time in a
    list. Data can be appended to the end with \ref push and retrieved
    from the fron with \ref pop
*/
template <class T> class CList
{
    private: // Definitions
        //! Helper structure which makes up the linked list elements
        struct LIST_ELEMENT
        {
            T value;                  //!< The value of the current element
            LIST_ELEMENT * previous; //!< The next element (NULL if it does not exist)
            LIST_ELEMENT * next;     //!< The next element (NULL if it does not exist)
        };

    public: // Definitions
        /*! \class CIter
            \brief Iterate over \ref CList elements and make modifications to
            to he list through the iterator. During the lifetime of an iterator
            the underlying \ref CList object must not be modified directly
            or through another iterator. Otherwise all iterators through which
            the modification was not done will be invalidated and their further
            use will lead to undefined behaviour. The only exemption from this is,
            if the iterator is pointing to the NULL element, in which a modification
            of the list in any way will not have an influence on its operation
        */
        class CIter
        {
            public:
                //! Constructor for CIter
                CIter(CList &l);

                //! Get the current element the iterator is pointing to
                bool current(T* list);

                //! Is the iterator pointing to the first element?
                bool isBegin();

                //! Is the iterator pointing to the last element?
                bool isEnd();

                //! Is the iterator pointing to a valid element?
                bool isValid();

                //! Erase the current element in the list
                bool erase();

                //! Pointer to the current element
                T* operator->();

                //! Point to the next element in the list
                CList<T>::CIter& operator++();

                //! Point to the next element in the list
                bool next(T* list);

                //! Point to the previous element in the list
                CList<T>::CIter& operator--();

                //! Point to the previous element in the list
                bool previous(T* list);

                //! Make the iterator point to the first element
                void reset();

            private:
                CList &_l;                //!< The list the iterator is operating on
                LIST_ELEMENT *_current;   //!< Current position of the linked list

        };

    public: // Functions
        //! Constructor
        CList();

        //! Copy constructor
        CList(const CList<T> &other);

        //! Destructor
        virtual ~CList();

        //! Assignment operator
        CList<T> & operator= (const CList<T> & other);

        //! Retrieve the number of elements in the list
        size_t getSize();

        //! Retrieve the element at the front
        bool front(T* list, pthread_mutex_t * m = NULL) const;

        //! Retrieve the element at the back
        bool back(T* list, pthread_mutex_t * m = NULL) const;

        //! Retrieve the element at the front and remove it
        bool pop(T* list = NULL, pthread_mutex_t * m = NULL);

        //! Add an element to the back of the list
        bool push(T entry, pthread_mutex_t * m = NULL);

        //! Clear all elements in the list
        void clear(pthread_mutex_t * m = NULL);
    private: // Functions
        //! Retrieve the element at the front without lock
        bool front_no_lock(T* list) const;

        //! Retrieve the element at the back without lock
        bool back_no_lock(T* list) const;

        //! Retrieve the element at the front and remove it without lock
        bool pop_no_lock(T* list = NULL);

        //! Add an element to the back of the list  without lock
        bool push_no_lock(T entry);

        //! Clear all elements in the list  without lock
        void clear_no_lock();

    private: // Variables
        LIST_ELEMENT *_begin;   //!< Start of the linked list
        LIST_ELEMENT *_end;     //!< End of the linked list
        size_t _size;            //!< Number of linked list elements
};

/*! Constructor

    \param l                 : Set the list which the iterator should
                               work upon
*/
template <class T> CList<T>::CIter::CIter(CList &l)
: _l(l)
, _current(l._begin)
{
}

/*! Access the current element

    \param list              : Will be set to the value of current.
                               Must not be NULL
    \return                    true if list could be set, false otherwise
*/
template<class T> bool CList<T>::CIter::current(T* list)
{
    if(!_current || !list)
        return false;

    *list=_current->value;
    return true;
}

/*! Determines if the current element is the beginning of the list

    \return                    true if beginning, false otherwise
*/
template<class T> bool CList<T>::CIter::isBegin()
{
    bool result=false;
    if(_current==_l._begin)
        result=true;    

    return result;
}

/*! Determines if the current element is the end of the list

    \return                    true if end, false otherwise
*/
template<class T> bool CList<T>::CIter::isEnd()
{
    bool result=false;
    if(_current==_l._end)
        result=true;    

    return result;
}

/*! Determines if the current element the iterator is pointing on
    is valid.

    \return                    true if valid, false otherwise
*/
template<class T> bool CList<T>::CIter::isValid()
{
    return _current!=NULL;
}

/*! Erase the current element the iterator is pointing to

    \return                    true on success, false otherwise
*/
template<class T> bool CList<T>::CIter::erase()
{
    bool result=false;
    if(_current)
    {
        assert(_l._size);
        LIST_ELEMENT *p, *o;
        p=_current->previous;
        o=_current;

        // Set the next element to be 
        // the current
        _current=_current->next;

        // Change previous' next pointer
        // to the new _current. If no previous exists,
        // _begin has to be adjusted
        if(p)
        {
            p->next=_current;
        }
        else
        {
            _l._begin=_current;
        }

        // Change nexts previous pointer
        // to the previous. If no current
        // exits, the previous is the new end
        if(_current)
        {
            _current->previous=p;
        }
        else
        {
            assert(_l._end==o);
            _l._end=p;
        }
        --_l._size;
        delete o;
        result=true;
    }
    return result;
}

/*! Access the current element directly. Result is unexpected if the current
    element is not valid.
*/
template<class T> T* CList<T>::CIter::operator->() //prefix
{
    return &_current->value;
}

/*! Make the iterator point to the next item. If the current item is the last
    item the iterator will be set to point to the NULL element. If the current
    element is the NULL element, the iterator will be set to the beginning of
    the list.
*/
template<class T> typename CList<T>::CIter& CList<T>::CIter::operator++() //prefix
{
    if(_current)
    {
        _current=_current->next;
    }
    else
    {
        _current=_l._begin;
    }
    return *this;
}

/*! Make the iterator point to the next item. If the current item is the last
    item the iterator will be set to point to the NULL element. If the current
    element is the NULL element, the iterator will be set to the beginning of
    the list.

    \param list              : Will be set to the value of the next element 
    \return                    true if the new element is valid, false
                               otherwise. If false list will not be set
*/
template<class T> bool CList<T>::CIter::next(T* list)
{
    // Switch to the next element
    ++(*this);

    // If the new (next item)
    // has not reached the end
    // of the list assign the value
    // if requested and make sure
    // a true is returned. Otherwise
    // return false
    bool result=false;
    if(_current)
    {
        if(list)
        {
            *list=_current->value;
        }
        result=true;
    }

    return result;
}

/*! Make the iterator point to the previous item. If the current item is the
    first item the iterator will be set to point to the NULL element. If the
    current element is the NULL element, the iterator will be set to the end
    of the list.
*/
template<class T> typename CList<T>::CIter& CList<T>::CIter::operator--()
{
    // Switch to the next element
    if(_current)
    {
        _current=_current->previous;
    }
    else
    {
        _current=_l._end;
    }
    return *this;
}

/*! Make the iterator point to the previous item. If the current item is the
    first item the iterator will be set to point to the NULL element. If theu
    current element is the NULL element, the iterator will be set to the end of
    the list.

    \param list              : Will be set to the value of the previous
                               element 
    \return                    true if the new element is valid, false
                               otherwise. If false list will not be set
*/
template<class T> bool CList<T>::CIter::previous(T* list)
{
    // Switch to the next element
    --(*this);

    // If the new (previous item)
    // has not reached the end
    // of the list assign the value
    // if requested and make sure
    // a true is returned. Otherwise
    // return false
    bool result=false;
    if(_current)
    {
        if(list)
        {
            *list=_current->value;
        }
        result=true;
    }

    return result;
}

/*! Reset the iterator to point to the beginning of
    the list.
*/
template<class T> void CList<T>::CIter::reset()
{
    _current=_l._begin;
}


/*! Constructor
*/
template <class T> CList<T>::CList()
    : _begin(NULL)
    , _end(NULL)
    , _size(0)
{
}

/*! Copy constructor. Clears the current linked list of this
    objects and uses a copy of all the list elements in
    in the passed argument instead. Uses operator=
    for this.

    \param other             : List that will be copied
*/
template <class T> CList<T>::CList(const CList<T> &other)
    : _begin(NULL)
    , _end(NULL)
    , _size(0)
{
    *this=other;
}

/*! Destructor
*/
template <class T> CList<T>::~CList()
{
    //lint -sem(CList::clear, cleanup)
    clear();
}

/*! Assignment operator. Clears the current linked list of this
    objects and uses a copy of all the list elements in
    in the passed argument instead.

    \param other             : List that will be copied
*/
template <class T> CList<T> & CList<T>::operator= (const CList<T> & other)
{
       // protect against self-assignment
    if (this != &other)    
    {
        // Clear the old data
        clear();
        // Either begin and end are set or none of both are
        assert((other._begin && other._end) || !(other._begin || other._end));
        LIST_ELEMENT const * tmpOther=other._begin;
        // Make a deep copy of the values in the other object
        while(tmpOther)
        {
            if(push(tmpOther->value))
            {
                // Either next exists or the current element is the end of the list
                assert(tmpOther->next  || (!tmpOther->next && tmpOther==other._end));
                tmpOther=tmpOther->next;
            }
        }
        // The size of the copy and the original must be the same
        assert(_size==other._size);
    }
    return *this;
}

/*! Get the number of elements currently stored in the linked list

    \return                  Number of elements in the list
*/
template <class T> size_t CList<T>::getSize()
{
    return _size;    
}

/*! Get the front element of the list without modifying it

    \param list            : Pointer to the variable that must be overwritten
                             with the front value on success. Must not be NULL
    \return                  true on success, false otherwise
*/
template <class T> bool CList<T>::front(T *list, pthread_mutex_t * m /* = NULL */ ) const
{
    bool result=false;
    if(m)
    {
        pthread_mutex_lock(m);
        result=front_no_lock(list);
        pthread_mutex_unlock(m);
    }
    else
    {
        result=front_no_lock(list);
    }

    return result;
}

/*! Get the front element of the list without modifying it or locking

    \param list            : Pointer to the variable that must be overwritten
                             with the front value on success. Must not be NULL
    \return                  true on success, false otherwise
*/
template <class T> bool CList<T>::front_no_lock(T *list) const
{
    bool result=false;
    if(_begin && list)
    {
        assert(_begin && _end && _size);
        *list=_begin->value;
        result=true;
    }
    return result;
}

/*! Get the back element of the list without modifying it

    \param list            : Pointer to the variable that must be overwritten
                             with the back value on success. Must not be NULL
    \return                  true on success, false otherwise
*/
template <class T> bool CList<T>::back(T *list, pthread_mutex_t * m /* = NULL */) const
{
    bool result=false;
    if(m)
    {
        pthread_mutex_lock(m);
        result=back_no_lock(list);
        pthread_mutex_unlock(m);
    }
    else
    {
        result=back_no_lock(list);
    }

    return true;
}

/*! Get the back element of the list without modifying it or locking

    \param list            : Pointer to the variable that must be overwritten
                             with the back value on success. Must not be NULL
    \return                  true on success, false otherwise
*/
template <class T> bool CList<T>::back_no_lock(T *list) const
{
    bool result=false;
    if(_end && list)
    {
        assert(_begin && _end && _size);
        *list=_end->value;
    }
    return true;
}

/*! Get the front element of the list and remove that element afterwards

    \param list            : Pointer to the variable that must be overwritten
                             with the front value on success. May be NULL
    \return                  true on success and false otherwise
*/
template <class T> bool CList<T>::pop(T *list /* = NULL */, pthread_mutex_t * m /* = NULL */)
{
    bool result=false;

    if(m)
    {
        pthread_mutex_lock(m);
        result=pop_no_lock(list);
        pthread_mutex_unlock(m);
    }
    else
    {
        result=pop_no_lock(list);
    }
    return result;
}

/*! Get the front element of the list and remove that element afterwards
    without locking

    \param list            : Pointer to the variable that must be overwritten
                             with the front value on success. May be NULL
    \return                  true on success and false otherwise
*/
template <class T> bool CList<T>::pop_no_lock(T *list /* = NULL */)
{
    bool result=false;
    // Get the front element, if required.
    // If successful everything is good. Otherwise
    // list must be NULL and _begin must not be NULL
    // to be able to continue
    if(front_no_lock(list) || (!list && _begin))
    {
        assert(_size >= 1 && _begin && _end);
        LIST_ELEMENT *tmp=_begin;
        // Make next element top of the list
        // (cleans up reference to _begin if _begin->next==NULL)
        _begin=_begin->next;

        // Still something in the list?
        if(_begin)
        {
            // Set value for the previous
            _begin->previous=NULL;
            assert(_begin->next || (!_begin->next && _end==_begin));
        }
        else
        {
            // The list is empty
            _end=NULL;
        }

        // Clean up
        delete tmp;
        --_size;
        result=true;
    }

    return result;
}

/*! Add an entry to the back of the list

    \param entry           : The entry of which a copy should be
                             stored in the list
    \return                  true on success, false otherwise
*/
template <class T> bool CList<T>::push(T entry, pthread_mutex_t * m /* = NULL */)
{
    bool result=false;
    if(m)
    {
        pthread_mutex_lock(m);
        result=push_no_lock(entry);
        pthread_mutex_unlock(m);
    }
    else
    {
        result=push_no_lock(entry);
    }

    return result;
}

/*! Add an entry to the back of the list without locking

    \param entry           : The entry of which a copy should be
                             stored in the list
    \return                  true on success, false otherwise
*/
template <class T> bool CList<T>::push_no_lock(T entry)
{
    bool result=false;
    LIST_ELEMENT * tmp=new(std::nothrow) LIST_ELEMENT;
    if(tmp)
    {
        tmp->next=NULL;
        tmp->previous=NULL;
        tmp->value=entry;

        // Are already entries in the list?
        // If yes append this element as the last
        // and set the old end to be the previous
        if(_end)
        {
            assert(_begin && !_end->next);
            tmp->previous=_end;
            _end->next=tmp;
            _end=tmp;
        }
        else
        {
            assert(!_begin && !_size);
            _begin=tmp;
            _end=tmp;
        }
        ++_size;
        result=true;
    }

    return result;
}

/*! Clear all the entries in the list
*/
//lint -e{1578}
template <class T> void CList<T>::clear(pthread_mutex_t * m /* = NULL */)
{
    if(m)
    {
        pthread_mutex_lock(m);
        clear_no_lock();
        pthread_mutex_unlock(m);
    }
    else
    {
        clear_no_lock();
    }
}

/*! Clear all the entries in the list
*/
//lint -e{1578}
template <class T> void CList<T>::clear_no_lock()
{
    while(pop_no_lock());
    assert(!_begin && !_end && !_size);
}

#endif //UBX_LIST

