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
 * $Id: ubx_agpsIf.cpp 113891 2016-04-12 13:30:15Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_agpsIf.cpp $
 *****************************************************************************/

#include "string.h"
#include "ubx_agpsIf.h"
#include "ubx_cfg.h"


///////////////////////////////////////////////////////////////////////////////
// Definitions & Types

///////////////////////////////////////////////////////////////////////////////
// Local data
static CAgpsIf s_myIf;		//!< Private instance of the CAgpsIf class - 'singleton'

const AGpsInterface CAgpsIf::s_interface = {		//!< Agps interface jump table
    size:                              sizeof(AGpsInterface),
    init:                              CAgpsIf::init,
    data_conn_open:                    CAgpsIf::dataConnOpen,
    data_conn_closed:                  CAgpsIf::dataConnClosed,
    data_conn_failed:                  CAgpsIf::dataConnFailed,
    set_server:                        CAgpsIf::setServer,
#if (PLATFORM_SDK_VERSION >= 21 /* >= 5.0 */ )
    data_conn_open_with_apn_ip_type:   NULL
#endif //PLATFORM_SD_VERSION >= 21 /* >= 5.0 */
};

///////////////////////////////////////////////////////////////////////////////
CAgpsIf::CAgpsIf()
{
    m_ready = false;
	m_agpsServersCnt = 0;
	m_certificateFileName = NULL;
    m_connectRetries = 0;;

	CCfg cfg;
	cfg.load("/system/etc/u-blox.conf");
	setCertificateFileName(cfg.get("SUPL_CACERT", (const char*)NULL));
	setConnectRetries(cfg.get("SUPL_CONNECT_RETRIES", 0));
    memset(&m_callbacks, 0, sizeof(m_callbacks));
    memset(m_agpsServers, 0, sizeof(m_agpsServers));
}

CAgpsIf::~CAgpsIf()
{
	if (m_certificateFileName)
	{
		free(m_certificateFileName);
		m_certificateFileName = NULL;
	}		
}

CAgpsIf* CAgpsIf::getInstance()
{
	return &s_myIf;
}

void CAgpsIf::init(AGpsCallbacks* callbacks)
{
    if (s_myIf.m_ready)
        UBX_LOG(LCAT_ERROR, "already initialized");
    UBX_LOG(LCAT_VERBOSE, "");
    s_myIf.m_callbacks = *callbacks;
	s_myIf.m_ready = true;
//lint -e{818} remove "could be declared as pointing to const"
}

int CAgpsIf::dataConnOpen(const char* apn)
{
    UBX_LOG(LCAT_DEBUG, "apn='%s'", apn);

	return 0;
}


int CAgpsIf::dataConnClosed(void)
{
    UBX_LOG(LCAT_DEBUG, "");

    return 0;
}

int CAgpsIf::dataConnFailed()
{
    UBX_LOG(LCAT_DEBUG, "");

    return 0;
}


int CAgpsIf::setServer(AGpsType type, const char* hostname, int port)
{
	UBX_LOG(LCAT_DEBUG, "type=%i(%s) host=%s port=%i", type, _LOOKUPSTR(type, AGpsType), hostname, port);
	int cnt = s_myIf.m_agpsServersCnt;
	if (cnt < NUM_AGPS_SERVERS)
	{
		s_myIf.m_agpsServers[cnt].type = type;
		s_myIf.m_agpsServers[cnt].port = port;
		strncpy(s_myIf.m_agpsServers[cnt].hostname, hostname, MAX_HOSTNAME);
		s_myIf.m_agpsServersCnt = cnt +1;
	}
	else
	{
		UBX_LOG(LCAT_ERROR, "no space");
	}
	
    return 0;
}

int CAgpsIf::numServer(AGpsType type) const
{
	int i;
	int cnt = 0;
	for (i = 0; i < m_agpsServersCnt; i ++)
	{
		if (m_agpsServers[i].type == type)
			cnt ++;
	}
	UBX_LOG(LCAT_DEBUG, "num=%d", cnt);
	return cnt;
}

void CAgpsIf::getSuplServerInfo(char** ppSuplServerAddress, int *pSuplPort)
{
	*ppSuplServerAddress = NULL;
	*pSuplPort = -1;

	int i;
	for (i = 0; i < m_agpsServersCnt; i ++)
	{
		if (m_agpsServers[i].type == AGPS_TYPE_SUPL)
		{
			*ppSuplServerAddress = m_agpsServers[i].hostname;
			*pSuplPort = m_agpsServers[i].port;
		}
	}
	UBX_LOG(LCAT_VERBOSE, "Address:Port %s:%d", *ppSuplServerAddress, *pSuplPort);
}

void CAgpsIf::setCertificateFileName(const char* pCert)
{
    if (m_certificateFileName)
    {
        free(m_certificateFileName);
        m_certificateFileName = NULL;
    }
    m_certificateFileName = pCert ? strdup(pCert) : NULL;
}

const char* CAgpsIf::getCertificateFileName(void) const
{
    return m_certificateFileName;
}

bool CAgpsIf::isTlsActive(void) const
{
    return (m_certificateFileName != NULL);
}

int CAgpsIf::getConnectRetries(void) const
{
    return m_connectRetries;
};

void CAgpsIf::setConnectRetries(int connectRetries)
{
    m_connectRetries = connectRetries;
};

