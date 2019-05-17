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
  \brief  SUPL connection API

  Handles socket and SSL connections to the SUPL server. Used by
  suplSMmanager.cpp and uplsend.cpp.
*/

#include "supl_connection.h"
#include "std_lang_def.h"
#include "std_macros.h"
#include "std_types.h"
#include "ubx_log.h"

#ifdef SUPL_ENABLED

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <openssl/err.h>
#include <openssl/hmac.h>
#include <openssl/ssl.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions & Types

struct SUPL_CONNECTION_STRUCT //!< State for a SUPL connection session
{
  int sock;         //!< socket file handle
  int use_tls;      //!< 1 - SSL will be used, 0 - unencrypted connection
  SSL_CTX *ssl_ctx; //!< context pointer to SSL_CTX
  BIO *bio;         //!< context pointer to BIO
  SSL *ssl;         //!< context pointer to SSL
};

//! Flag for initializing SSL library only once.
static int supl_connection_only_once = 1;

/*! Initializes a new SUPL connection. To be called before any other
    functions of this API. The returned context pointer must be used in
    further calls.

    \return context pointer, or 0 if malloc fails.

  */
SUPL_CONNECTION *supl_connection_init(void)
{
  SUPL_CONNECTION *supl_connection;

  if (supl_connection_only_once)
  {
    supl_connection_only_once = 0;
    SSL_library_init();
  }

  supl_connection = (SUPL_CONNECTION *)malloc(sizeof(SUPL_CONNECTION));
  if (!supl_connection)
    return 0;

  supl_connection->sock = 0;
  supl_connection->ssl_ctx = 0;
  supl_connection->bio = 0;
  supl_connection->ssl = 0;
  supl_connection->use_tls = 0;

  return supl_connection;
}

static int _connect_error_can_retry(const char *error)
{
  UBX_LOG(LCAT_ERROR, "SUPL connection failed: %s", error);
  return -1;
}

static int _connect_error(const char *error)
{
  UBX_LOG(LCAT_ERROR, "SUPL connection failed: %s", error);
  return -2;
}

static int _fcntl_flag(int sock, int set_or_unset, int flag)
{
  int r;

  r = fcntl(sock, F_GETFL, 0);
  if (r < 0)
    return -1;
  if (set_or_unset)
    r |= flag;
  else
    r &= ~((unsigned int)flag);
  r = fcntl(sock, F_SETFL, r);
  if (r < 0)
    return -1;

  return 0;
}

/*! Connects an initialized SUPL connection.

    \param supl_connection  : context pointer, as returned by
                              supl_connection_init.
    \param timeout_seconds  : timeout for connect in seconds.
    \param supl_server_name : string containing name of SUPL server
    \param supl_server_port : port number to connect to of SUPL server
    \param use_tls          : 0 for unencrypted connection, non-zero for TLS
    \param cert_file        : string containing path to optional certificate
                              file, only for TLS. Can be 0 or "".

    \return 0 - connection ok, -1 - unsuccessful connect (retry possible),
    -2 - other connection error, -3 - TLS error.

  */
int supl_connection_connect(SUPL_CONNECTION *supl_connection,
                            int timeout_seconds,
                            const char *supl_server_name,
                            int supl_server_port,
                            int use_tls,
                            const char *cert_file)
{
  int r, sock;
  char port_string[21];
  struct addrinfo hint
  {
  };
  struct addrinfo *result{};

  supl_connection->use_tls = use_tls;

  if (use_tls)
    supl_connection->ssl_ctx = SSL_CTX_new(SSLv23_client_method());
  if (supl_connection->ssl_ctx == nullptr)
  {
    return _connect_error("SSL_CTX_new - couldn't create ssl ctx from library method");
  }

  if (use_tls && cert_file && *cert_file)
  {
    r = SSL_CTX_load_verify_locations(supl_connection->ssl_ctx, cert_file, NULL);
    if (!r)
    {
      UBX_LOG(LCAT_ERROR, "Could not load the certificate");
      return -3;
    }
    UBX_LOG(LCAT_VERBOSE, "Certificate loaded");
  }

  UBX_LOG(LCAT_VERBOSE, "Connecting to SUPL server");

  hint.ai_family = AF_UNSPEC;
  hint.ai_socktype = SOCK_STREAM;

  snprintf(port_string, sizeof(port_string), "%d", supl_server_port);

  r = getaddrinfo(supl_server_name, port_string, &hint, &result);
  if ((r != 0) || (result == nullptr))
    return _connect_error("getaddrinfo()");

  sock = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
  if (sock < 0)
  {
    freeaddrinfo(result);
    return _connect_error("socket()");
  }

  r = _fcntl_flag(sock, 1, O_NONBLOCK);
  if (r < 0)
    return _connect_error("fcntl() set non-blocking");

  r = connect(sock, result->ai_addr, result->ai_addrlen);
  if (r < 0 && errno != EINPROGRESS)
  {
    freeaddrinfo(result);
    close(sock);
    return _connect_error_can_retry("connect()");
  }

  if (r < 0 && errno == EINPROGRESS)
  {
    fd_set write_fds;
    struct timeval tv;
    int so_error;
    socklen_t so_error_length;

    FD_ZERO(&write_fds);
    FD_SET(sock, &write_fds);

    tv.tv_sec = timeout_seconds;
    tv.tv_usec = 0;

    r = select(sock + 1, 0, &write_fds, 0, &tv);
    // Here, !FD_ISSET(sock, &write_fds) should not happen, for r >
    // 0, but let's still check.
    if (r <= 0 || !FD_ISSET(sock, &write_fds))
    {
      freeaddrinfo(result);
      close(sock);
      if (r == 0)
        return _connect_error_can_retry("select() timeout");
      if (r < 0)
        return _connect_error_can_retry("select() error");

      // sock is not set in write_fds, which again, should not
      // happen, if r > 0.

      return _connect_error_can_retry("select() - no error, no timeout, but still not writable");
    }

    // Now the socket is writable. We still need to check for an
    // error with getsockopt()

    so_error_length = sizeof(so_error);
    r = getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &so_error_length);
    if (r < 0)
      return _connect_error_can_retry("getsockopt() < 0");
    if (so_error)
      return _connect_error_can_retry("getsockopt() - SO_ERROR");
  }

  r = _fcntl_flag(sock, 0, O_NONBLOCK);
  if (r < 0)
    return _connect_error("fcntl() set blocking");

  freeaddrinfo(result);
  supl_connection->sock = sock;

  if (use_tls)
  {
    supl_connection->bio = BIO_new_socket(supl_connection->sock, BIO_CLOSE);
    supl_connection->ssl = SSL_new(supl_connection->ssl_ctx);
    if (supl_connection->ssl == nullptr)
    {
      return _connect_error("SSL_new - couldn't create ssl connection from context");
    }
    SSL_set_verify(supl_connection->ssl, SSL_VERIFY_PEER, nullptr);
    SSL_CTX_set_verify(supl_connection->ssl_ctx, SSL_VERIFY_PEER, nullptr);
#if PLATFORM_SDK_VERSION < 23
    SSL_set_mode(supl_connection->ssl, SSL_MODE_AUTO_RETRY);
#endif
    SSL_set_bio(supl_connection->ssl, supl_connection->bio, supl_connection->bio);
    UBX_LOG(LCAT_VERBOSE, "TLS connect");
    r = SSL_connect(supl_connection->ssl);
    if (r != 1)
    {
      r = SSL_get_error(supl_connection->ssl, r);
      UBX_LOG(LCAT_ERROR, "SSL connect failed: %d", r);
      close(supl_connection->sock);
      supl_connection->sock = 0;
      SSL_free(supl_connection->ssl);
      supl_connection->ssl = 0;
      supl_connection->bio = 0;
      return -3;
    }
  }

  UBX_LOG(LCAT_VERBOSE, "Connected to SUPL server");

  return 0;
}

/*! Returns file descriptor (socket) for a connected SUPL connection.

    \param supl_connection  : context pointer, as returned by
                              supl_connection_init.
*/
int supl_connection_get_fd(SUPL_CONNECTION *supl_connection)
{
  if (!supl_connection)
  {
    UBX_LOG(LCAT_ERROR, "SUPL connection is null. ");
    return -1;
  }

  return supl_connection->sock;
}

/*! Dealloactes and ends a SUPL connection. The context pointer can not
    be use after this call.

    \param supl_connection  : context pointer, as returned by
                              supl_connection_init.
*/
void supl_connection_dealloc(SUPL_CONNECTION *supl_connection)
{
  if (!supl_connection)
  {
    UBX_LOG(LCAT_ERROR, "SUPL connection is null. ");
    return;
  }

  SSL_CTX_free(supl_connection->ssl_ctx);
  if (supl_connection->ssl)
    SSL_free(supl_connection->ssl);

  if (supl_connection->sock)
    close(supl_connection->sock);

  free(supl_connection);
}

/*! Reads data from a connected SUPL connection.

    \param supl_connection  : context pointer, as returned by
                              supl_connection_init.
    \param buffer           : buffer to read into, allocated by caller.
    \param amount           : amount of data in bytes to read

    \return amount of data read, or <0 for error.
*/
int supl_connection_read(SUPL_CONNECTION *supl_connection, unsigned char *buffer, long amount)
{
  int r;

  UBX_LOG(LCAT_VERBOSE, "supl_connection_read: %d", amount);

  if (!supl_connection)
  {
    UBX_LOG(LCAT_ERROR, "SUPL connection is null. ");
    return -1;
  }

  if (supl_connection->use_tls)
  {
    UBX_LOG(LCAT_VERBOSE, "SSL_read");
    r = SSL_read(supl_connection->ssl, buffer, amount);
    UBX_LOG(LCAT_VERBOSE, "SSL_read: %d", r);
    if (r < 0)
      UBX_LOG(LCAT_ERROR, "SSL_read error. ");

    return r;
  }

  UBX_LOG(LCAT_VERBOSE, "Socket read");
  r = read(supl_connection->sock, buffer, amount);
  UBX_LOG(LCAT_VERBOSE, "Socket read: %d", r);

  return r;
}

/*! Writes data to a connected SUPL connection.

    \param supl_connection  : context pointer, as returned by
                              supl_connection_init.
    \param buffer           : buffer to write from, allocated by caller.
    \param amount           : amount of data in bytes to write

    \return amount of data written, or <0 for error.
*/
int supl_connection_write(SUPL_CONNECTION *supl_connection,
                          const unsigned char *buffer,
                          long amount)
{
  int r;

  UBX_LOG(LCAT_VERBOSE, "supl_connection_write: %d", amount);

  if (!supl_connection)
  {
    UBX_LOG(LCAT_ERROR, "SUPL connection is null. ");
    return -1;
  }

  if (supl_connection->use_tls)
  {
    UBX_LOG(LCAT_VERBOSE, "SSL_write");
    r = SSL_write(supl_connection->ssl, buffer, amount);
    UBX_LOG(LCAT_VERBOSE, "SSL_write: %d", r);
    if (r < 0 || r != amount)
      UBX_LOG(LCAT_ERROR, "SSL_write error (%d). ", r);

    return r;
  }

  UBX_LOG(LCAT_VERBOSE, "Socket write");
  r = write(supl_connection->sock, buffer, amount);
  UBX_LOG(LCAT_VERBOSE, "Socket write: %d", r);

  return r;
}

#endif // ifdef SUPL_ENABLED

// supl_connection.cpp
