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
/*!
  \file
  \brief  Local TCP Port Handling

  This file provides the implementation of local ports. A master (i.e. what is
  implemented here) broadcasts data it has received to local clients which wait
  for TCP traffic on a defined port.

  If clients talk, the traffic is received in here and delivered to the
  upper layers.

  Connection handling, keep-alive  timeouting is also being done in here
*/

#ifdef TCP_SERVER_PORT
#include <cassert>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>

#include "../agnss/helper/helperFunctions.h"
#include "ubx_log.h"
#include "ubx_tcpServer.h"
#include "ubx_timer.h"

const UBX_MSG_TYPE CTcpServer::_allowed[] = {
  { 0x0A, 0x04 }, // Allow MON-VER poll message
};

// The pass phrase must contain of at least one byte that has not the value
// 0x00, otherwise it will not work
const unsigned char CTcpServer::_passPhrase[] = {
  0x00, 0x00, 0x00, 0x00
};                                               // Fill in passphrase to enable this feature

CTcpServer::CTcpServer()
  : _numConn(0), _maxConn(0), _fdUsrIf(-1), _fdSocket(-1), _port(0), _localhostOnly(true),
    _echo(false)
{
}

CTcpServer::~CTcpServer() { this->stopServer(); }

int CTcpServer::startServer(uint16_t port,
                            uint8_t maxConn,
                            bool echo /* = false */,
                            bool localhostOnly /* = true */)
{
  if (_fdUsrIf != -1)
    return -1;

  _maxConn = maxConn;
  _numConn = 0;
  _port = port;
  _localhostOnly = localhostOnly;
  _echo = echo;

  assert(_fdSocket == -1);
  _fdUsrIf = open();
  assert((_fdSocket != -1 && _fdUsrIf != -1) || (_fdSocket == -1 && _fdUsrIf == -1));

  return _fdUsrIf;
}

void CTcpServer::stopServer()
{
  this->close();
  _fdUsrIf = -1;
  assert(_fdSocket == -1 && _fdUsrIf == -1);
}

ssize_t CTcpServer::onNewMsg(const unsigned char *pBuf, size_t len) const
{
  if (_fdUsrIf == -1 || !pBuf || !len)
    return -1;

  return write(_fdUsrIf, pBuf, len);
}

ssize_t CTcpServer::sizeNextMsg() const
{
  if (_fdUsrIf == -1)
    return -1;

  ssize_t result = 0;

  int bytes_available;
  if (ioctl(_fdUsrIf, FIONREAD, &bytes_available) == 0 && bytes_available > 0)
  {
    result = (ssize_t)bytes_available;
  }

  return result;
}

ssize_t CTcpServer::getNextMsg(unsigned char *buf, size_t count) const
{
  if (_fdUsrIf == -1)
    return -1;

  return read(_fdUsrIf, buf, count);
}

bool CTcpServer::fdSet(fd_set &rfds, int &rMaxFd) const
{
  if (_fdUsrIf < 0)
    return false;

  if ((_fdUsrIf + 1) > rMaxFd)
    rMaxFd = _fdUsrIf + 1;

  FD_SET(_fdUsrIf, &rfds);

  return true;
}

bool CTcpServer::fdIsSet(fd_set &rfds) const
{
  bool result = false;
  if ((_fdUsrIf >= 0) && (FD_ISSET(_fdUsrIf, &rfds)))
    result = true;

  return result;
};

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
    \return             On success 1 will be returned,
                        otherwise -1
*/
int CTcpServer::impl_open(int **fds, int *timeout_ms)
{
  assert(_fdSocket == -1);
  if (!fds || !timeout_ms)
    return -1;

  int *tmpFds = (int *)malloc(sizeof(int));
  if (!tmpFds)
    return -1;

  /* open socket */
  if ((*tmpFds = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, IPPROTO_TCP)) == -1)
  {
    free(tmpFds);
    return -1;
  }

  struct sockaddr_in si_me; // Structures for me with socket addr info
  memset((char *)&si_me, 0, sizeof(si_me));

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons((uint16_t)_port);
  si_me.sin_addr.s_addr = htonl((uint32_t)(_localhostOnly ? INADDR_LOOPBACK : INADDR_ANY));

  int result = -1;
  if (!bind(*tmpFds, (struct sockaddr *)(void *)&si_me, sizeof(si_me)) && !listen(*tmpFds, 256))
  {
    *fds = tmpFds;       // Pass the pointer back to the caller
    _fdSocket = *tmpFds; // Store the fd for closing later on

    // One second timeout ought
    // to be enough for everyone
    *timeout_ms = 1000;
    UBX_LOG(LCAT_VERBOSE,
            "%s TCP Server started at %u! Maximum connections: %u (fd = %d)",
            _localhostOnly ? "Local" : "Global",
            _port,
            _maxConn,
            _fdSocket);
    result = 1;
  }
  else
  {
    UBX_LOG(LCAT_ERROR, "Unable to start TCP/IP server: %s\n", strerror(errno));
    ::close(*tmpFds);
    free(tmpFds);
  }

  return result;
}

/*! Implements the purely virtual function from the base

    \return            Will return 0
*/
int CTcpServer::impl_close()
{
  // Open file descriptors are closed by CThreadedPacketIf
  _fdSocket = -1;
  return 0;
}

/*! Implements the purely virtual function from the base and
    processes within the thread setup by the base
    data coming from the non-threaded user side of the
        interface as well as TCP clients.

    \param source              : Specifies which file descriptor the data has
                                 been retrieved.
    \param buf                 : The data that must be processed. May not be
   NULL
    \param size                : Size of buf in bytes. May not be 0
    \return                    : The number of bytes processed. Must be
                                 smaller than size. If less bytes are
                                 processed than passed to the function,
                                 the remaining bytes will be passed again
                                 during the next call to the function together
                                 with additional data if available
*/
size_t CTcpServer::impl_process(size_t source, unsigned char *buf, size_t size)
{
  assert(buf);
  assert(size);
  assert(source != 1);

  ssize_t numNotReqElem = 0;

  if (source == 0) // This is a message from the receiver
  {
    _authMutex.lock();
    for (auto &i : _authList)
    {
      if (i.authSucc)
        writeTo(i.source, buf, size);
    }
    _authMutex.unlock();
    numNotReqElem = size;
  }
  else // This is a client message, send it to the receiver
  {
    bool auth = isSourceAuth(source);
    // If not authenticated, autenticate the source
    if (!auth)
      numNotReqElem = authSource(source, buf, size, &auth);

    // If authenticated, search for UBX messages in
    // all the code again (the passphrase can be a valid message too!)
    if (auth && numNotReqElem >= 0)
    {
      numNotReqElem = findAndProcessUbx(0, buf, size); // Destination is 0
    }
  }

  // Calculate the number of bytes processed. On error,
  // clear the whole buffer anyway.
  size_t result = 0;
  if (numNotReqElem >= 0)
  {
    result = (size_t)numNotReqElem;
  }
  else
  {
    // An error occured while writing, clean the complete buffer
    result = size;
  }

  return result;
}

/*! Will implement the notification handling as defined by the base.
    This implementation will initiate accepting new connections for
        source 1 and will make sure all other source data is processed
*/
bool CTcpServer::impl_notify(size_t source, CComThread::NOTIF_t type)
{
  bool result = true;
  switch (type)
  {
  case CComThread::DATA_AVAILABLE:
  {
    // If source==1 this is the open socket and a client would like to create
    // a new connection and additional processing is required and a false
    // will be returned in any case
    if (source == 1)
    {
      result = false; // Don't try to read from this source!
      acceptConnection();
    }
    break;
  }
  case CComThread::SOURCE_CLOSED:
  default:
  {
    // It does not make sense that the main socket closed from the other side
    assert(source != 1);
    if (source != 1 && source != 0)
    {
      UBX_LOG(LCAT_VERBOSE,
              "Connection to TCP client closed! ID: %i. Total "
              "active connections: %u/%u",
              source,
              --_numConn,
              _maxConn);
      // Delete the list entry
      {
        std::lock_guard<std::mutex> lock(_authMutex);
        for (auto i = _authList.begin(); (i != _authList.end()); i++)
        {
          if (i->source == source)
          {
            _authList.erase(i);
            break;
          }
        }
      }
    }
  }
  }
  return result;
}

/*! Implements the purely virtual function from the base and
    handles timeouts. Timeouts are not of interest for this class
        and will be ignored
*/
void CTcpServer::impl_timeout() {}

/*! Will implement the error handling as defined by the base
    and print out corresponding error messages
*/
void CTcpServer::impl_errorOccurred(int operation, ssize_t fdindex, int correrrno)
{
  switch (operation)
  {
  case COMTHREAD_ERR_READ:
  {
    UBX_LOG(LCAT_ERROR,
            "An error occured while trying to read from file"
            " descriptor index %i:"
            " '%s'",
            fdindex,
            strerror(correrrno));
    break;
  }
  case COMTHREAD_ERR_PROC:
  {
    UBX_LOG(LCAT_ERROR,
            "Processing data from file descriptor"
            " index %i failed: %s",
            fdindex,
            strerror(correrrno));
    break;
  }
  case COMTHREAD_ERR_POLL:
  {
    UBX_LOG(LCAT_ERROR,
            "An error occured"
            " while waiting for data input from"
            " the managed interfaces. Trying again."
            " Error description: %s",
            strerror(correrrno));
    break;
  }
  default:
  {
    UBX_LOG(LCAT_ERROR,
            "An unknown error occured in the"
            " processing thread!");
  }
  }
}

/*! Will accept new clients on the TCP socket
*/
void CTcpServer::acceptConnection()
{
  assert(_fdSocket != -1);
  struct sockaddr_in client;
  socklen_t clientLen = sizeof(client);
  int fd = accept(_fdSocket, (struct sockaddr *)&client, &clientLen);
  if (fd >= 0)
  {
    if (_numConn < _maxConn)
    {
      ssize_t connId = add(fd);
      if (connId >= 0)
      {
        ++_numConn;
        char address[64] = "UNKNOWN";
        char port[8] = "UNKNOWN";
        getnameinfo((struct sockaddr *)&client,
                    clientLen,
                    address,
                    sizeof(address),
                    port,
                    sizeof(port),
                    NI_NUMERICHOST | NI_NUMERICSERV);
        UBX_LOG(LCAT_VERBOSE,
                "Connected to new TCP client! ID: %i. Source: "
                "%s:%s. Total active connections: %u/%u",
                connId,
                address,
                port,
                _numConn,
                _maxConn);
        AUTH_SOURCE_t tmpAuthSource;
        tmpAuthSource.source = connId;
        tmpAuthSource.authSucc = false;
        _authList.push_back(tmpAuthSource); // Create entry in the validation list
      }
      else
      {
        UBX_LOG(LCAT_WARNING,
                "An error occured trying to add a file descriptor"
                " to te communication thread!");
        ::close(fd);
      }
    }
    else
    {
      UBX_LOG(LCAT_WARNING,
              "Too many active connections on the TCP Server "
              "(%u/%u)! No additional connections will be "
              "accepted until others are closed!",
              _numConn,
              _maxConn);
    }
  }
  else // An error occured
  {
    if (errno != EAGAIN && errno != EWOULDBLOCK)
    {
      UBX_LOG(LCAT_WARNING,
              "An error occured while trying to accept a new client:"
              " %s",
              strerror(errno));
    }
  }
}

size_t CTcpServer::findAndProcessUbx(size_t dest, unsigned char *buf, size_t size)
{
  assert(buf);
  assert(size);
  ssize_t numNotReqElem = 0;
  typedef enum
  {
    UBX_NOT_FOUND = 0,
    UBX_NOT_ENOUGH_DATA = 1,
    UBX_FOUND = 2
  } UBX_IN_BUF_t;

  size_t ubxStart = 0;
  ssize_t ubxSize = 0;
  UBX_IN_BUF_t ubxInBuf = UBX_NOT_FOUND;
  // Find a Ubx message if valid
  while (ubxStart < size && ubxInBuf == UBX_NOT_FOUND)
  {
    ubxSize = isUbxMessage(buf + ubxStart, size - ubxStart, true);
    if (ubxSize > 0)
    {
      if (isAllowedUbxMsg(
            buf + ubxStart, ubxSize, _allowed, sizeof(_allowed) / sizeof(_allowed[0])))
      {
        ubxInBuf = UBX_FOUND;
      }
      else
      {
        // Ignore messages not allowed
        ++ubxStart;
      }
    }
    else if (ubxSize == HELPER_ERR_ARG_TOO_SHORT || ubxSize == HELPER_ERR_ENTRY_TOO_SHORT)
    {
      ubxInBuf = UBX_NOT_ENOUGH_DATA;
    }
    else
    {
      // Ignore non-UBX data
      ++ubxStart;
    }
  }

  // If a valid UBX message was found, send it to the receiver
  // Otherwise get rid of the bytes that don't belong to a valid UBX message
  if (ubxInBuf == UBX_FOUND)
  {
    assert(ubxSize >= 8);

    numNotReqElem = writeTo(dest, buf + ubxStart, ubxSize);

    if (_echo) // If this fails, we ignore it.
    {
      // SSIZE_MAX+1 -> send to all file descriptors
      // SSIZE_MAX+2 -> send to all file descriptors, except the first
      // SSIZE_MAX+3 -> send to all file descriptors, except the first two
      writeTo(((size_t)SSIZE_MAX) + 3, buf + ubxStart, ubxSize);
    }
  }

  // Get rid of all the data written and not including UBX messages
  if (numNotReqElem >= 0)
  {
    numNotReqElem += ubxStart;
  }
  return numNotReqElem;
}

bool CTcpServer::isSourceAuth(size_t source)
{
  bool result = false;
  bool found = false;
  _authMutex.lock();
  for (auto i = _authList.begin(); (i != _authList.end()) && !found; ++i)
  {
    if (i->source == source)
    {
      result = i->authSucc;
      found = true;
    }
  }
  assert(found);
  _authMutex.unlock();
  return result;
}

ssize_t CTcpServer::authSource(size_t source, unsigned char *buf, size_t size, bool *auth)
{
  assert(buf);
  assert(size);
  assert(auth);
  *auth = false;

  // Is the passphrase valid? Otherwise let nothing pass
  bool validPassPhrase = false;
  for (size_t i = 0; i < sizeof(_passPhrase) && !validPassPhrase; ++i)
  {
    validPassPhrase = (_passPhrase[i] != 0x00);
  }
  if (!validPassPhrase)
    return -1;

  ssize_t result = 0;
  // If the passphrase is valid, search for it
  const size_t elemPassPhrase = sizeof(_passPhrase) / sizeof(_passPhrase[0]);
  // Search pass phrase
  if (elemPassPhrase <= size)
  {
    // Check for passphrase
    size_t i = 0;
    for (; i < size - elemPassPhrase + 1 && !*auth; ++i)
    {
      *auth = (0 == memcmp(_passPhrase, buf + i, elemPassPhrase));
    }
    // These bytes are not required any more.
    // Keep the passphrase in the buffer, it might be an UBX message
    result = i;
  }

  // Change the corresponding item to true
  if (*auth)
  {
    bool found = false;
    _authMutex.lock();
    for (auto i = _authList.begin(); (i != _authList.end()) && !found; ++i)
    {
      if (i->source == source)
      {
        i->authSucc = true;
        found = true;
      }
    }
    assert(found);
    _authMutex.unlock();
  }
  return result;
}

#endif // ifdef TCP_SERVER_PORT
