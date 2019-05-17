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
 * $Id: ubx_udpServer.cpp 94373 2015-04-16 13:28:53Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_udpServer.cpp $
 *****************************************************************************/
/*!
  \file
  \brief  Local UDP Port Handling

  This file provides the implementation of local ports. A master (i.e. what is
  implemented here) broadcasts data it has received to local clients which wait
  for UDP traffic on a defined port.

  If clients talk, the traffic is received in here and delivered to the
  upper layers.

  Connection handling, keep-alive  timeouting is also being done in here
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <time.h>

#include "ubx_log.h"
#include "ubx_mux.h"
#include "ubx_timer.h"
#include "ubx_udpServer.h"


int CUdpServer::openLocalPort(int udpPort)
{
    struct sockaddr_in si_me; // Structures for me with socket addr info

    if (m_fd != -1)
		close(m_fd);
	/* open socket */
	if ((m_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        UBX_LOG(LCAT_WARNING, "unable to create socket: %s\n", strerror(errno));
        return -1;
    }
    memset((char *) &si_me, 0, sizeof(si_me));
    memset((char *) m_udpConn, 0, sizeof(m_udpConn)); // set all connection info to zero
    si_me.sin_family = AF_INET;
#if defined (_lint)
    ((void)(udpPort));
    si_me.sin_port = 0;
    si_me.sin_addr.s_addr = 0;
#else
    si_me.sin_port = htons((unsigned short) udpPort);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
#endif

    if (bind(m_fd, (struct sockaddr *) (void *) &si_me, sizeof(si_me))==-1)
    {
        UBX_LOG(LCAT_WARNING, "unable to bind socket: %s\n", strerror(errno));
		close(m_fd);
		m_fd = -1;
    }
	else
		UBX_LOG(LCAT_VERBOSE, "Udp port opened, fd = %d", m_fd);
//lint -e{438} remove Last value assigned to variable '_x'
    return m_fd;
}

int CUdpServer::recvPort(char * pBuf,int buflen)
{
    // Enter here if select has indicated that we can read
    // from the local client
    // handle new connections from clients / ping replies
	
    struct sockaddr_in si_other; // Structures for me and others with socket addr info

    memset(&si_other, 0, sizeof(si_other));

    int slen = sizeof(si_other);
    int conn;
    int len = recvfrom(m_fd,
                       pBuf,
                       (unsigned int) buflen,
                       0,
                       (struct sockaddr *) (void *) &si_other,
                       (socklen_t *)&slen);
	//UBX_LOG(LCAT_VERBOSE, "Receiving data (%i)", len);
    if (len < 0)
    {
        UBX_LOG(LCAT_WARNING, "recvfrom failed %s", strerror(errno));
    }

    if (len == 0)
    {
        UBX_LOG(LCAT_WARNING, "zero length packet no more allowed... ");
    }

    conn = findConnection(si_other);
    if (conn == -1) // not known connection
    {
        /* we check only if new connection request! */
        if ( (len == 3) && (pBuf[0] == UDP_MUX_PACKET_HEADER) && (pBuf[1] == UDP_MUX_START_SESSION) )
        {
            conn = findEmptySlot();
            if (conn == -1)
            {
                UBX_LOG(LCAT_WARNING, "No empty slot found!");
            }
            else
            {

                /* record the source IP and port */
                memcpy( (char *) &m_udpConn[conn].conn, (char *)&si_other,sizeof(si_other));

                /* set the feature of this connection */
                m_udpConn[conn].flags = (unsigned char) pBuf[2];

                /* update the time stamp */
                m_udpConn[conn].seen = getMonotonicMsCounter();

#if !defined (_lint)
                UBX_LOG(LCAT_VERBOSE, "new connection %i (%s:%i), feature 0x%.2X",
                      conn,
                      inet_ntoa(m_udpConn[conn].conn.sin_addr),
                      ntohs(m_udpConn[conn].conn.sin_port),
                      m_udpConn[conn].flags);
#endif
					
                /* Send the notification to the clients that has requested to be informed */
                int a = activeConnections();
                unsigned char buf[4] = {'X', UDP_MUX_NUM_CLIENT, (unsigned char)(a & 0xFF), (unsigned char)((a>>8) & 0xFF) };
                sendPort(buf, sizeof(buf));
			}
        }
        else
        {
            /* UDP message not recognized!!! */
            UBX_LOG(LCAT_WARNING, "The message has not been recognized: len = %d, code = %d", len, (len>0 ? pBuf[0] : 0) );
        }

        /* return 0, not forwarding the message! */
        return 0;
    }

    /* here we are only if the connection has been found! */
    m_udpConn[conn].seen = getMonotonicMsCounter(); // update timeout data, as we have received data from this client
    if (len > 0)
    {
        if ( !( pBuf[0] == UDP_MUX_PACKET_HEADER && len == 2 && pBuf[1] == UDP_MUX_PING ) )
        {
#if !defined (_lint)
            /* exclude ping messagges */
            UBX_LOG(LCAT_VERBOSE, "from %i (%s:%i) len=%i",conn,
                  inet_ntoa(m_udpConn[conn].conn.sin_addr),
                  ntohs(m_udpConn[conn].conn.sin_port),len);
#endif
        }

        if (pBuf[0] == UDP_MUX_PACKET_HEADER)
        {

            if (len == 2 && pBuf[1] == UDP_MUX_PING)
            {
                /* Ping replay!!!! keep silent! */
            }
            if (len == 3 && pBuf[1] == UDP_MUX_START_SESSION)
            {
                /* reset the feature of this connection */
                m_udpConn[conn].flags = (unsigned char) pBuf[2];

#if !defined (_lint)
                UBX_LOG(LCAT_VERBOSE, "connection renegotiation %i (%s:%i), feature 0x%.2X",
                      conn,
                      inet_ntoa(m_udpConn[conn].conn.sin_addr),
                      ntohs(m_udpConn[conn].conn.sin_port),
                      m_udpConn[conn].flags);
#endif

            }
            if ((len == 2) && (pBuf[1] == UDP_MUX_END_SESSION))
            {
                UBX_LOG(LCAT_VERBOSE, "end session for connection %i", conn);
                m_udpConn[conn].conn.sin_port = 0; // clear this entry
                // inform other clients
                int a = activeConnections();
                unsigned char buf[4] = {'X', UDP_MUX_NUM_CLIENT, (unsigned char)(a & 0xFF), (unsigned char)((a>>8) & 0xFF) };
                sendPort(buf, sizeof(buf));
            }

            /* the MUX packets are always not forwarded */
            len = 0;
        }

        /* forward all the packets for the clients in trasparent mode */
        int conn_1;

        for(conn_1=0; conn_1 < MAXUDPCONN; conn_1++)
        {
            /* it exclues the packets the clients itself has send */
            if ( (m_udpConn[conn_1].conn.sin_port) && (conn_1 != conn) && (m_udpConn[conn_1].flags & CLIENT_TRASPARENT_MODE))
            {
                ssize_t res;
                res = sendto(m_fd,
                             pBuf,
                             (unsigned int) len,
                             0,
                             (struct sockaddr *) (void *) &m_udpConn[conn_1].conn,
                             sizeof(m_udpConn[conn_1].conn));
                if (res == -1)
                {
                    UBX_LOG(LCAT_WARNING, "Error in sending UDP packet, \"%s\"\n", strerror(errno));
                }
                else if (res != len)
                {
                    UBX_LOG(LCAT_WARNING, "Error in sending UDP packet, wrong lenght %d\n", (int) res);
                }
            }
        }
    }

    return len;
}

void CUdpServer::sendPort(const unsigned char * pBuf, int len) const
{
    int conn;
    if (m_fd < 0)
	{
        return;
	}
	//UBX_LOG(LCAT_VERBOSE, "Sending %i bytes", len);
    for(conn = 0; conn < MAXUDPCONN; conn++)
    {
        if (m_udpConn[conn].conn.sin_port)
        {
            ssize_t res = 0;
            if (pBuf[0] == '$' && ((m_udpConn[conn].flags & CLIENT_NMEA_ENABLED)==CLIENT_NMEA_ENABLED))
            {
                /* send to non null client NMEA message */
                res = sendto(m_fd,
                             pBuf,
                             (unsigned int) len,
                             0,
                             (struct sockaddr *) (void *) &m_udpConn[conn].conn,
                             sizeof(m_udpConn[conn].conn));
            }
            else if (((unsigned char) pBuf[0] == 0xB5) &&
                     ((m_udpConn[conn].flags & CLIENT_UBX_ENABLED) == CLIENT_UBX_ENABLED))
            {
                /* send to non null client UBX message */
                res = sendto(m_fd,
                             pBuf,
                             (unsigned int) len,
                             0,
                             (struct sockaddr *) (void *) &m_udpConn[conn].conn,
                             sizeof(m_udpConn[conn].conn));
            }
            else if (pBuf[0] == 'X' &&
                     ((m_udpConn[conn].flags & CLIENT_MUX_ENABLED) == CLIENT_MUX_ENABLED))
            {
                /* send to non null client MUX message */
                res = sendto(m_fd,
                             pBuf,
                             (unsigned int) len,
                             0,
                             (struct sockaddr *) (void *) &m_udpConn[conn].conn,
                             sizeof(m_udpConn[conn].conn));
            }
			else
			{
				//UBX_LOG(LCAT_VERBOSE, "Nothing sent. Client on %i not showing any interest (0x%.2X) ", conn, m_udpConn[conn].flags);
				// As this is not an error, stop error message being reported
				res = len;
			}
			
            if (res == -1)
            {
                UBX_LOG(LCAT_WARNING, "Error in sending UDP packet on %i, \"%s\"\n", conn, strerror(errno));
            }
            else if (res != len)
            {
                UBX_LOG(LCAT_WARNING, "Error in sending UDP packet, wrong length %d (%d)\n", (int) res, len);
            }
        }
    }
}

void CUdpServer::checkPort(int slaveOpen)
{
    int conn;
	//UBX_LOG(LCAT_VERBOSE, "Checking port (%i)", slaveOpen);
    for(conn=0; conn < MAXUDPCONN; conn++)
    {
        if (m_udpConn[conn].conn.sin_port)
        {
            int64_t now = getMonotonicMsCounter();

            if ((now - m_udpConn[conn].seen) >= SESSION_TIMEOUT_MS)
            {
#if !defined (_lint)
                UBX_LOG(LCAT_VERBOSE, "timeout on connection %i (%s:%i): %lld\n",conn,
                      inet_ntoa(m_udpConn[conn].conn.sin_addr),
                      ntohs(m_udpConn[conn].conn.sin_port),
                      (now -  m_udpConn[conn].seen) );
#endif
                m_udpConn[conn].conn.sin_port = 0; // clear this entry

                /* Send the notification to the clients that has requested to be informed */
                m_slaveStatus = slaveOpen;
                int a = activeConnections();
                unsigned char buf[4] = {'X', UDP_MUX_NUM_CLIENT, (unsigned char)(a & 0xFF), (unsigned char)((a>>8) & 0xFF) };
                sendPort(buf, sizeof(buf));
            }
			else
			{
                /* send the ping package! */
                ssize_t res;
                char buf[] = {'X',0x00};
                res = sendto(m_fd,
                             buf,
                             sizeof(buf),
                             0,
                             (struct sockaddr *) (void *) &m_udpConn[conn].conn,
                             sizeof(m_udpConn[conn].conn)); // send next ping
                if (res == -1)
                {
                    UBX_LOG(LCAT_WARNING, "Error in sending UDP packet, \"%s\"\n",strerror(errno));
                }
                else if (res != (ssize_t) sizeof(buf))
                {
                    UBX_LOG(LCAT_WARNING, "Error in sending UDP packet, wrong lenght %d\n", (int) res);
                }
            }
        }
    }

    if (m_slaveStatus != slaveOpen)
    {
        /* notification! for sure it has not been already sent */
        /* Send the notification to the clients that has requested to be informed */
        m_slaveStatus = slaveOpen;
        int a = activeConnections();
        unsigned char buf[4] = {'X', UDP_MUX_NUM_CLIENT, (unsigned char)(a & 0xFF), (unsigned char)((a>>8) & 0xFF) };
        sendPort(buf, sizeof(buf));
    }
}

int CUdpServer::findConnection (struct sockaddr_in si_other) const
{
    int conn;
    // search this connection
    for(conn = 0; conn < MAXUDPCONN; conn++)
    {
        if ((m_udpConn[conn].conn.sin_port == si_other.sin_port) &&
            (m_udpConn[conn].conn.sin_addr.s_addr == si_other.sin_addr.s_addr))
            return conn;
    }

    /* failure... */
    return -1;
//lint -e{1746} remove  could be made const reference
}

int CUdpServer::findEmptySlot (void) const
{
    int conn;
    for(conn = 0; conn < MAXUDPCONN; conn++)
    {
        if (m_udpConn[conn].conn.sin_port == 0) {
            return conn;
        }
    }

    return -1;
}

int CUdpServer::activeConnections (void) const
{
    int conn;
    int cnt = 0;
    for(conn = 0; conn < MAXUDPCONN; conn++)
    {
        if (m_udpConn[conn].conn.sin_port != 0) {
            cnt |= (0x01<<conn);
        }
    }

    if (m_slaveStatus)
    {
        cnt |= 0x8000;
    }

    return cnt;
}
