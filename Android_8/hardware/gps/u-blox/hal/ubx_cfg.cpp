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
 * $Id: ubx_cfg.cpp 83032 2014-07-14 16:14:58Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_cfg.cpp $
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <new>

#include "std_types.h"
#include "ubx_cfg.h"
#include "ubx_log.h"

CCfg::CCfg()
{
	m_num = 0;
	memset(m_itemName, 0, sizeof(m_itemName));
	memset(m_itemData, 0, sizeof(m_itemData));
}

CCfg::~CCfg()
{
	while (m_num --)
	{
		delete [] m_itemName[m_num];
		delete [] m_itemData[m_num];
	}
	m_num = 0;
}

void CCfg::load(const char* fileName)
{
	FILE* file = fopen(fileName, "r");
	UBX_LOG(LCAT_DEBUG, "fileName=\"%s\"", fileName);
	if (file == NULL)
    {
        // failed
        UBX_LOG(LCAT_VERBOSE, "Can not open '%s' file : %i", fileName, errno);
        return;
    }

	while(!feof(file))
    {
		char line[1024];
        memset(line, 0, sizeof(line));
        // Load each line
        if (fgets(line, sizeof(line), file) != NULL)
        {
			const char* name;
			const char* data;
			const char* p;
			unsigned int nameLen;
			unsigned int dataLen = 0;
			p = line;
			// skip spaces
			while (isspace(*p))	p++;
			name = p;
			// find end of name
			while (isgraph(*p) && (*p != '=') && (*p != '#'))
				p++;
			nameLen = (unsigned int) (p-name);
			if (nameLen > 0)
			{
				// skip spaces
				while (isspace(*p) && (*p != '#') && (*p != '\r') && (*p != '\n'))	p ++;
				// skip equal sign
				if (*p == '=') p ++;
				// skip spaces
				while (isspace(*p) && (*p != '#') && (*p != '\r') && (*p != '\n'))	p ++;
				data = p;
				// take all until we find end of line or comment start
				while ((*p != '\0') && (*p != '#') && (*p != '\r') && (*p != '\n'))
				{
					if (isgraph(*p))
						dataLen = (unsigned int) (p - data + 1);
					p ++;
				}
				if (m_num < MAX_ITEM)
				{
					m_itemName[m_num] = new(std::nothrow) char[nameLen+1];
					if (m_itemName[m_num])
					{
						strncpy(m_itemName[m_num], name, nameLen);
						m_itemName[m_num][nameLen] = '\0'; 			
					}
					
					m_itemData[m_num] = new(std::nothrow) char[dataLen+1];
					if (m_itemData[m_num])
					{
						strncpy(m_itemData[m_num], data, dataLen);
						m_itemData[m_num][dataLen] = '\0'; 				
					}
					
					//UBX_LOG(LCAT_DEBUG, "index=%d name=\"%s\" data=\"%s\"", m_num, m_itemName[m_num], m_itemData[m_num]);
					m_num ++;
				}
				else
					UBX_LOG(LCAT_ERROR, "items list full");
			}
		}
	}
	fclose(file);
}

int CCfg::get(const char* item, int def) const
{
	int val = def;
	for (int i = 0; i < m_num; i ++)
	{
		if (strcmp(m_itemName[i], item) == 0)
		{
			char* end;
			if ((item[0] == '0') && (item[1] == 'x' || item[1] == 'X'))
				val = strtol(m_itemData[i]+2, &end, 16);
			else if ((item[0] == '0') && (item[1] == 'b' || item[1] == 'B'))
				val = strtol(m_itemData[i]+2, &end, 2);
			else
				val = strtol(m_itemData[i], &end, 10);
			if (*end != 0)
				val = def;
			break;
		}
	}
	UBX_LOG(LCAT_DEBUG, "item=\"%s\" def=%d => %d", item, def, val);
	return val;
}

const char* CCfg::get(const char* item, const char* def) const
{
	const char*	val = def;
	for (int i = 0; i < m_num; i ++)
	{
		if (strcmp(m_itemName[i], item) == 0)
		{
			val = m_itemData[i];
			break;
		}
	}
	UBX_LOG(LCAT_DEBUG, "item=\"%s\" def=\"%s\" => \"%s\"", item, def ? def : "NULL", val ? val : "NULL");
	return val;
}

