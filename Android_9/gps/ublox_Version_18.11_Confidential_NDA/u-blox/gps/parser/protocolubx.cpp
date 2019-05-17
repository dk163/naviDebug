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
 * Project: libParser
 * Purpose: Library providing functions to parse u-blox GNSS receiver messages.
 *
 ******************************************************************************
 * $Id: protocolubx.cpp 108172 2015-12-14 13:03:19Z fabio.robbiani $
 * $HeadURL:
 *http://svn.u-blox.ch/GPS/SOFTWARE/PRODUCTS/libParser/tags/libParser_v1.04/protocolubx.cpp
 *$
 *****************************************************************************/

#include <new>
#include <stdlib.h>

#include "gpsconst.h"
#include "parserbuffer.h"
#include "protocolubx.h"
#include "ubx_log.h"
#include "ubx_messageDef.h"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <vector>

std::unordered_multimap<MessageId, std::function<void(const Message &message)>>
  CProtocolUBX::messageHandler_{};

int CProtocolUBX::receiverGen = 7;

namespace
{
  static const int UBX_MAX_SIZE = 2 * 1024;
  static std::vector<MessageId> validMessageHandlerIds = {
    UBXID_ACK_ACK,  UBXID_ACK_NAK,
#ifdef RAW_MEAS_ENABLED
    UBXID_RXM_RAWX, UBXID_NAV_CLOCK, UBXID_RXM_SFRBX,
#endif
  };

  bool isValidMessageHandlerId(MessageId id)
  {
    return std::find(validMessageHandlerIds.begin(), validMessageHandlerIds.end(), id) !=
           validMessageHandlerIds.end();
  }

  U2 getMessageId(const unsigned char *pBuffer) { return (pBuffer[2] << 8) | pBuffer[3]; }

  bool messageSizeInvalid(int messageSize) { return messageSize < 0; }

  bool messageTooSmall(int receivedSize, unsigned int messageSize)
  {
    if (messageSizeInvalid(messageSize))
    {
      return true;
    }

    if (receivedSize < (int)(messageSize + GPS_UBX_FRAME_SIZE))
    {
      // Not enough data received
      return true;
    }

    return false;
  }

  bool bufferWouldOverrun(const unsigned char *const pBuffer,
                          const unsigned char *const pBufferEnd,
                          unsigned int Size)
  {
    return (pBuffer + Size < pBufferEnd) ? false : true;
  }

  const unsigned char *getBufferAtOffset(const unsigned char *pBuffer, int offset)
  {
    return pBuffer + offset;
  }

  const unsigned char *skipMessageHead(const unsigned char *pBuffer)
  {
    return pBuffer += GPS_UBX_HEAD_SIZE;
  }

  template <typename T>
  bool copyMessagePartWithSize(const unsigned char *&pBuffer,
                               const unsigned char *const pBufferEnd,
                               T &messagePart,
                               size_t copySize)
  {
    if (bufferWouldOverrun(pBuffer, pBufferEnd, static_cast<unsigned>(copySize)))
      return false;
    memcpy(&messagePart, pBuffer, copySize);
    pBuffer += copySize;
    return true;
  }

  template <typename T>
  bool copyMessagePart(const unsigned char *&pBuffer,
                       const unsigned char *const pBufferEnd,
                       std::vector<T> &messagePart,
                       size_t elementNum)
  {

    messagePart.resize(elementNum);
    return copyMessagePartWithSize(
      pBuffer, pBufferEnd, *(messagePart.data()), sizeof(T) * elementNum);
  }

  template <typename T>
  bool copyMessagePart(const unsigned char *&pBuffer,
                       const unsigned char *const pBufferEnd,
                       T &messagePart)
  {
    return copyMessagePartWithSize(pBuffer, pBufferEnd, messagePart, sizeof(T));
  }

  template <typename T>
  void notifyDatabaseAboutNavPvt(CDatabase *pDatabase, const Message *const msg)
  {
    typedef const T *const PtrType;
    pDatabase->Set(CDatabase::DATA_UBX_ACCURACY_HORIZONTAL_M,
                   static_cast<double>((static_cast<PtrType>(msg))->hAcc) * 1e-3);
    pDatabase->Set(CDatabase::DATA_UBX_ACCURACY_VERTICAL_M,
                   static_cast<double>((static_cast<PtrType>(msg))->vAcc) * 1e-3);
    pDatabase->Set(CDatabase::DATA_UBX_ACCURACY_SPEED_MPS,
                   static_cast<double>((static_cast<PtrType>(msg))->sAcc) * 1e-3);
    pDatabase->Set(CDatabase::DATA_UBX_ACCURACY_HEADING_DEG,
                   static_cast<double>((static_cast<PtrType>(msg))->headAcc) * 1e-5);
  }

  template <typename T>
  void processNavPvtGeneration(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase)
  {
    T message;
    if (messageTooSmall(iSize, sizeof(T)))
    {
      return;
    }

    UBX_LOG(LCAT_VERBOSE, "Processing NAV_PVT message...");

    // calculate end of buffer
    const unsigned char *pBufferEnd = getBufferAtOffset(pBuffer, iSize);
    pBuffer = skipMessageHead(pBuffer);

    if (!copyMessagePart(pBuffer, pBufferEnd, message))
    {
      UBX_LOG(LCAT_VERBOSE, "error in copy navPvt");
      return;
    }
    notifyDatabaseAboutNavPvt<T>(pDatabase, &message);
  }

} // namespace

/**
@brief process "simple" message which have no substructs like e.g. NAV_CLOCK
*/
template <typename T>
void CProtocolUBX::processMessage(const unsigned char *pBuffer, int iSize, MessageId id)
{
  T message;
  if (messageTooSmall(iSize, sizeof(message)))
  {
    return;
  }

  UBX_LOG(LCAT_VERBOSE, "Processing templated message...");

  // calculate end of buffer
  const unsigned char *pBufferEnd = getBufferAtOffset(pBuffer, iSize);
  pBuffer = skipMessageHead(pBuffer);
  if (!copyMessagePart(pBuffer, pBufferEnd, message))
  {
    UBX_LOG(LCAT_VERBOSE, "error in copy templated message");
    return;
  }

  callHandlersForMessage(id, message);
}

#ifdef RAW_MEAS_ENABLED
template <>
void CProtocolUBX::processMessage<UBX_RXM_RAWX>(const unsigned char *pBuffer,
                                                int iSize,
                                                MessageId id)
{
  UBX_RXM_RAWX rawMessage;
  if (messageTooSmall(iSize, sizeof(UBX_RXM_RAWX_SUMMARY)))
  {
    return;
  }

  UBX_LOG(LCAT_VERBOSE, "Processing RXM_RAWX message...");

  // calculate end of buffer
  const unsigned char *pBufferEnd = getBufferAtOffset(pBuffer, iSize);
  pBuffer = skipMessageHead(pBuffer);
  if (!copyMessagePart(pBuffer, pBufferEnd, rawMessage.summary))
  {
    UBX_LOG(LCAT_VERBOSE, "error in copy rawMessage summary");
    return;
  }

  if (!copyMessagePart(pBuffer, pBufferEnd, rawMessage.individual, rawMessage.summary.numMeas))
  {
    UBX_LOG(LCAT_VERBOSE, "error in copy rawMessage measurements");
    return;
  }

  callHandlersForMessage(id, rawMessage);
}

template <>
void CProtocolUBX::processMessage<UBX_RXM_SFRBX>(const unsigned char *pBuffer,
                                                 int iSize,
                                                 MessageId id)
{
  UBX_RXM_SFRBX sfrMessage;

  if (messageTooSmall(iSize, sizeof(UBX_RXM_SFRBX_SUMMARY)))
  {
    return;
  }

  UBX_LOG(LCAT_VERBOSE, "Processing RXM_SFRBX message...");

  // calculate end of buffer
  const unsigned char *pBufferEnd = getBufferAtOffset(pBuffer, iSize);
  pBuffer = skipMessageHead(pBuffer);

  if (!copyMessagePart(pBuffer, pBufferEnd, sfrMessage.summary))
  {
    UBX_LOG(LCAT_VERBOSE, "error in copy sfrMessage summary");
    return;
  }

  if (!copyMessagePart(pBuffer, pBufferEnd, sfrMessage.data_wd, sfrMessage.summary.numWords))
  {
    UBX_LOG(LCAT_VERBOSE, "error in copy sfrMessage measurements");
    return;
  }

  callHandlersForMessage(id, sfrMessage);
}
#endif

int CProtocolUBX::Parse(const unsigned char *pBuffer, int iSize)
{
  return ParseUbx(pBuffer, iSize);
}

int CProtocolUBX::ParseUbx(const unsigned char *const pBuffer, const int iSize)
{
  if (iSize == 0)
    return CParserBuffer::WAIT;
  if (pBuffer[0] != GPS_UBX_SYNC_CHAR_1)
    return CParserBuffer::NOT_FOUND;
  if (iSize == 1)
    return CParserBuffer::WAIT;
  if (pBuffer[1] != GPS_UBX_SYNC_CHAR_2)
    return CParserBuffer::NOT_FOUND;
  if (iSize < 6)
    return CParserBuffer::WAIT;
  U2 iLength = (U2)(((U2)pBuffer[4]) + (((U2)pBuffer[5]) << 8));
  // filter out all large messages (with the exception of the tunneling class
  // messages)
  if ((iLength > UBX_MAX_SIZE) && (pBuffer[2] != 0x08 /*tunneling class*/))
    return CParserBuffer::NOT_FOUND;
  if (iSize < iLength + 6)
    return CParserBuffer::WAIT;
  // calculate the cksum
  U1 ckA = 0;
  U1 ckB = 0;
  for (int i = 2; i < iLength + 6; i++)
  {
    ckA += pBuffer[i];
    ckB += ckA;
  }
  // check the cksum
  if (iSize < static_cast<int>(iLength + GPS_UBX_FRAME_SIZE - 1))
    return CParserBuffer::WAIT;
  if (pBuffer[iLength + 6] != ckA)
    return CParserBuffer::NOT_FOUND;
  if (iSize < static_cast<int>(iLength + GPS_UBX_FRAME_SIZE))
    return CParserBuffer::WAIT;
  if (pBuffer[iLength + 7] != ckB)
    return CParserBuffer::NOT_FOUND;
  return iLength + GPS_UBX_FRAME_SIZE;
}

void CProtocolUBX::Process(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase)
{
#ifdef RAW_MEAS_ENABLED
  UBX_LOG(LCAT_VERBOSE, "New message with ID: 0x%04x", getMessageId(pBuffer));
#endif
  switch (getMessageId(pBuffer))
  {
  case UBXID_NAV_STATUS:
    pDatabase->Process(CDatabase::MSG_UBX_NAV_STATUS, pBuffer, iSize, &ProcessNavStatus);
    break;
  case UBXID_NAV_SVINFO:
    pDatabase->Process(CDatabase::MSG_UBX_NAV_SVINFO, pBuffer, iSize, &ProcessNavSvInfo);
    break;
  case UBXID_NAV_PVT:
    pDatabase->Process(CDatabase::MSG_UBX_NAV_PVT, pBuffer, iSize, &ProcessNavPvt);
    break;
  case UBXID_MON_HW:
    pDatabase->Process(CDatabase::MSG_UBX_MON_HW, pBuffer, iSize, &ProcessMonHw);
    break;
  case UBXID_ACK_ACK:
    processMessage<GPS_UBX_ACK_ACK_t>(pBuffer, iSize, UBXID_ACK_ACK);
    break;
  case UBXID_ACK_NAK:
    processMessage<GPS_UBX_ACK_NAK_t>(pBuffer, iSize, UBXID_ACK_NAK);
    break;
  case UBXID_NAV_SAT:
    pDatabase->Process(CDatabase::MSG_UBX_NAV_SAT, pBuffer, iSize, &ProcessNavSat);
    break;
#ifdef RAW_MEAS_ENABLED
  case UBXID_RXM_RAWX:
    processMessage<UBX_RXM_RAWX>(pBuffer, iSize, UBXID_RXM_RAWX);
    break;
  case UBXID_RXM_SFRBX:
    processMessage<UBX_RXM_SFRBX>(pBuffer, iSize, UBXID_RXM_SFRBX);
    break;
  case UBXID_NAV_CLOCK:
    processMessage<GPS_UBX_NAV_CLOCK_t>(pBuffer, iSize, UBXID_NAV_CLOCK);
    break;
#endif
#ifdef SUPL_ENABLED
  case UBXID_RXM_MEAS:
    pDatabase->Process(CDatabase::MSG_UBX_RXM_MEAS, pBuffer, iSize, &ProcessRxmMeas);
    break;
#endif // SUPL_ENABLED
  default:
    UBX_LOG(LCAT_VERBOSE, "Could not find handler for messageId: 0x%04x", getMessageId(pBuffer));
    break;
  }
}

unsigned int CProtocolUBX::NewMsg(
  U1 classId, U1 msgId, const void *pPayload, unsigned int iPayloadSize, unsigned char **ppMsg)
{
  unsigned int nRetVal = iPayloadSize + GPS_UBX_FRAME_SIZE;

  if (ppMsg)
  {
    U1 *pBuffer = new (std::nothrow) U1[nRetVal];

    if (pBuffer)
    {
      *ppMsg = pBuffer;

      pBuffer[0] = GPS_UBX_SYNC_CHAR_1;
      pBuffer[1] = GPS_UBX_SYNC_CHAR_2;
      pBuffer[2] = classId;
      pBuffer[3] = msgId;
      pBuffer[4] = (U1)iPayloadSize;
      pBuffer[5] = (U1)(iPayloadSize >> 8);
      memcpy(&pBuffer[6], pPayload, iPayloadSize);

      // calculate the cksum
      U1 ckA = 0;
      U1 ckB = 0;
      for (unsigned int i = 2; i < iPayloadSize + 6; i++)
      {
        ckA += pBuffer[i];
        ckB += ckA;
      }
      pBuffer[6 + iPayloadSize] = ckA;
      pBuffer[7 + iPayloadSize] = ckB;
    }
    else
    {
      nRetVal = 0;
      *ppMsg = NULL;
    }
  }
  else
  {
    nRetVal = 0;
  }

  return nRetVal;
}

void CProtocolUBX::ProcessNavStatus(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase)
{
  if (iSize == (int)(sizeof(GPS_UBX_NAV_STATUS_t) + GPS_UBX_FRAME_SIZE))
  {
    GPS_UBX_NAV_STATUS_t s, *p = &s;
    memcpy(p, &pBuffer[GPS_UBX_HEAD_SIZE], sizeof(s));
    if (p->ttff) // A TTFF of 0ms implies that no TTFF exists yet -> no fix
    {
      pDatabase->Set(CDatabase::DATA_UBX_TTFF, p->ttff);
    }
  }
}

void CProtocolUBX::ProcessNavSvInfo(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase)
{
  GPS_UBX_NAV_SVINFO_t perMsgData;
  if (messageTooSmall(iSize, sizeof(perMsgData)))
  {
    // Not enough data received
    return;
  }

  memcpy(&perMsgData, &pBuffer[GPS_UBX_HEAD_SIZE], sizeof(perMsgData));

  GPS_UBX_NAV_SVINFO_CHN_t currCh;

  if (iSize != (int)(sizeof(perMsgData) + GPS_UBX_FRAME_SIZE + perMsgData.numCh * sizeof(currCh)))
  {
    // The number of channels indicated does not match the size
    // of the message
    return;
  }

  int ixInView = 0; // enumerates the valid SV in view
                    // as well as the location in the DB
  int ixUsed = 0;   // enumerates the valid SV in view used
  const size_t STATIC_DATA_OFFSET = (size_t)GPS_UBX_HEAD_SIZE + sizeof(perMsgData);

  // Go through all channels in the received message and store the
  // valid data in the DB as long as there is space in the DB and channels
  // in the message
  for (int i = 0; i < perMsgData.numCh && ixInView <= CDatabase::MAX_SATELLITES_IN_VIEW; i++)
  {
    // Get current channel data
    memcpy(&currCh, &pBuffer[STATIC_DATA_OFFSET + i * sizeof(currCh)], sizeof(currCh));

    // Data is stored in the DB in NMEA 4.1 extended format, so
    // conver the UBX svid to this enumeration type
    CDatabase::NMEA_t nmea41;
    CDatabase::ConvertUbxSvidToNmea41e((int)currCh.svid, &nmea41);
    bool validSvId = CDatabase::ConvertUbxSvidToNmea41e((int)currCh.svid, &nmea41);

    // Only use valid satellites of which the location is valid
    if (validSvId)
    {
      // Fill in SV and GNSS ID into the DB (Unknown GLONASS is -1)
      pDatabase->Set(DATA_SATELLITES_IN_VIEW_GNSSID_(ixInView), nmea41.gnssid);
      pDatabase->Set(DATA_SATELLITES_IN_VIEW_SVID_(ixInView), nmea41.svid);

      // Fill in valid SV positions into the DB
      if (currCh.azim >= -180 && currCh.azim <= 360 && currCh.elev >= -90 && currCh.elev <= 90)
      {
        pDatabase->Set(DATA_SATELLITES_IN_VIEW_ELEVATION_(ixInView), (double)currCh.elev);
        pDatabase->Set(DATA_SATELLITES_IN_VIEW_AZIMUTH_(ixInView),
                       CDatabase::Degrees360((double)currCh.azim));
      }

      // Fill in valid CNOs into the DB
      if (currCh.cno)
      {
        pDatabase->Set(DATA_SATELLITES_IN_VIEW_STN_RATIO_(ixInView), (double)currCh.cno);
      }

      // Count the satellites in use
      if (currCh.flags & GPS_UBX_NAV_SVINFO_CHN_FLAGS_SVUSED_MASK)
      {
        ++ixUsed;
      }

      // Fill in NAV status into the DB
      int navsta =
        ((currCh.flags & GPS_UBX_NAV_SVINFO_CHN_FLAGS_SVUSED_MASK) ? CDatabase::IS_USED : 0) |
        ((currCh.flags & GPS_UBX_NAV_SVINFO_CHN_FLAGS_ORBITAVAIL_MASK) ? CDatabase::HAS_ALM : 0) |
        ((currCh.flags & GPS_UBX_NAV_SVINFO_CHN_FLAGS_ORBITEPH_MASK) ? CDatabase::HAS_EPH : 0);

      pDatabase->Set(DATA_UBX_SATELLITES_IN_VIEW_NAV_STA_(ixInView), navsta);

      // Use a new DB field the next time around
      ixInView++;
    }
  }

  // Store the number of satellites in View and the ones ussed for the fix
  // into the DB
  pDatabase->Set(CDatabase::DATA_SATELLITES_IN_VIEW, ixInView);
  pDatabase->Set(CDatabase::DATA_SATELLITES_USED_COUNT, ixUsed);
}

void CProtocolUBX::ProcessNavPvt(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase)
{
  if (receiverGen >= 8)
  {
    processNavPvtGeneration<GPS_UBX_NAV_PVT_8_s>(pBuffer, iSize, pDatabase);
  }
  else
  {
    processNavPvtGeneration<GPS_UBX_NAV_PVT_7_s>(pBuffer, iSize, pDatabase);
  }
}

void CProtocolUBX::ProcessMonHw(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase)
{
  if (messageTooSmall(iSize, sizeof(GPS_UBX_MON_HW_t)))
  {
    return;
  }

  pBuffer = skipMessageHead(pBuffer);
  GPS_UBX_MON_HW_t msg;
  memcpy(&msg, pBuffer, sizeof(GPS_UBX_MON_HW_t));

  pDatabase->Set(CDatabase::DATA_UBX_AGC, msg.agcCnt);
}

void CProtocolUBX::ProcessNavSat(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase)
{
  UBX_NAV_SAT_DATA0_t perMsgData;
  if (messageTooSmall(iSize, sizeof(perMsgData)))
  {
    // Not enough data received
    UBX_LOG(LCAT_ERROR, "Message too small");
    return;
  }

  memcpy(&perMsgData, &pBuffer[GPS_UBX_HEAD_SIZE], sizeof(perMsgData));

  UBX_NAV_SAT_DATA0_GNSSID_t currSv;

  if (iSize != (int)(sizeof(perMsgData) + GPS_UBX_FRAME_SIZE + perMsgData.numSvs * sizeof(currSv)))
  {
    // The number of satellites indicated does not match the size
    // of the message
    UBX_LOG(LCAT_ERROR, "Mismatch in satellite number");
    return;
  }

  int ixInView = 0; // enumerates the valid SV in view
                    // as well as the location in the DB
  int ixUsed = 0;   // enumerates the valid SV in view used
  const size_t STATIC_DATA_OFFSET = (size_t)GPS_UBX_HEAD_SIZE + sizeof(perMsgData);

  // Go through all channels in the received message and store the
  // valid data in the DB as long as there is space in the DB and channels
  // in the message
  for (int i = 0; i < perMsgData.numSvs && ixInView <= CDatabase::MAX_SATELLITES_IN_VIEW; i++)
  {
    // Get current satellite data
    memcpy(&currSv, &pBuffer[STATIC_DATA_OFFSET + i * sizeof(currSv)], sizeof(currSv));

    // Fill in only valid CNOs into the DB
    if (currSv.cno == 0)
      continue;

    // Data is stored in the DB in NMEA 4.1 extended format, so
    // convert the UBX gnssId:svId to this enumeration type (table Satellite Numbering in protocol spec)
    if (currSv.gnssId == CDatabase::GNSSID_GLO)
    {
      if (currSv.svId != 255)
      {
        currSv.svId += 64;
      }
    }
    else if (currSv.gnssId == CDatabase::GNSSID_SBAS)
    {
      currSv.svId = (currSv.svId - 120 + 33);
    }

    // Only use valid satellites of which the location is valid
    if ((currSv.svId < 0) || (currSv.svId > 255))
      continue;

    // Fill in SV and GNSS ID into the DB (Unknown GLONASS is -1)
    pDatabase->Set(DATA_SATELLITES_IN_VIEW_GNSSID_(ixInView), currSv.gnssId);
    pDatabase->Set(DATA_SATELLITES_IN_VIEW_SVID_(ixInView), currSv.svId);

    // Fill in valid SV positions into the DB
    if (currSv.azim >= -180 && currSv.azim <= 360 && currSv.elev >= -90 && currSv.elev <= 90)
    {
      pDatabase->Set(DATA_SATELLITES_IN_VIEW_ELEVATION_(ixInView), (double)currSv.elev);
      pDatabase->Set(DATA_SATELLITES_IN_VIEW_AZIMUTH_(ixInView),
                     CDatabase::Degrees360((double)currSv.azim));
    }

    pDatabase->Set(DATA_SATELLITES_IN_VIEW_STN_RATIO_(ixInView), (double)currSv.cno);

    // Count the satellites in use
    if (currSv.flags & UBX_NAV_SAT_DATA0_GNSSID_FLAGS_SVUSED_MASK)
    {
      ++ixUsed;
    }

    // Fill in NAV status into the DB
    int navsta =
      ((UBX_NAV_SAT_DATA0_GNSSID_FLAGS_SVUSED_GET(currSv.flags)) ? CDatabase::IS_USED : 0) |
      ((UBX_NAV_SAT_DATA0_GNSSID_FLAGS_ALMAVAIL_GET(currSv.flags)) ? CDatabase::HAS_ALM : 0) |
      ((UBX_NAV_SAT_DATA0_GNSSID_FLAGS_EPHAVAIL_GET(currSv.flags)) ? CDatabase::HAS_EPH : 0);

    pDatabase->Set(DATA_UBX_SATELLITES_IN_VIEW_NAV_STA_(ixInView), navsta);

    // Use a new DB field the next time around
    ixInView++;
  }

  // Store the number of satellites in View and the ones ussed for the fix
  // into the DB
  pDatabase->Set(CDatabase::DATA_SATELLITES_IN_VIEW, ixInView);
  pDatabase->Set(CDatabase::DATA_SATELLITES_USED_COUNT, ixUsed);
}

#ifdef SUPL_ENABLED
void CProtocolUBX::ProcessRxmMeas(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase)
{
  if (messageTooSmall(iSize, sizeof(GPS_UBX_RXM_MEAS_t)))
  {
    // No enough data received
    return;
  }

  pBuffer = skipMessageHead(pBuffer);
  GPS_UBX_RXM_MEAS_t rmxMeasHeader;
  memcpy(&rmxMeasHeader, pBuffer, sizeof(rmxMeasHeader));
  pBuffer += sizeof(rmxMeasHeader);

  I4 svCount = rmxMeasHeader.info & 0xFF;
  I4 dopCenter = ((I4)rmxMeasHeader.info) >> 8;

  if (messageTooSmall(
        iSize, sizeof(GPS_UBX_RXM_MEAS_t) + (svCount * (int)sizeof(GPS_UBX_RXM_MEAS_SVID_t))))
  {
    // Not enough data received
    return;
  }

  pDatabase->Set(CDatabase::DATA_UBX_GNSS_TOW, rmxMeasHeader.gnssTow);
  pDatabase->Set(CDatabase::DATA_UBX_GNSS_DOP_CENTER, dopCenter);

  I4 gpsSvCount = 0;
  GPS_UBX_RXM_MEAS_SVID_t svData;
  for (I4 i = 0; i < svCount; i++)
  {
    memcpy(&svData, pBuffer, sizeof(GPS_UBX_RXM_MEAS_SVID_t));
    pBuffer += sizeof(GPS_UBX_RXM_MEAS_SVID_t);

    if ((svData.svid > 0) && (svData.svid <= 32))
    {
      pDatabase->Set(DATA_UBX_SATELLITES_IN_MEAS_(gpsSvCount), svData.svid);
      pDatabase->Set(DATA_UBX_SATELLITES_IN_MEAS_CNO_(gpsSvCount), svData.cno);

      R8 prRms = svData.prRms * 0.5;
      pDatabase->Set(DATA_UBX_SATELLITES_IN_MEAS_PRRMS_(gpsSvCount), prRms);
      pDatabase->Set(DATA_UBX_SATELLITES_IN_MEAS_MULTIPATH_IND_(gpsSvCount), svData.mpInd);
      pDatabase->Set(DATA_UBX_SATELLITES_IN_MEAS_REL_CODEPHASE_(gpsSvCount), svData.redSigtow);
      pDatabase->Set(DATA_UBX_SATELLITES_IN_MEAS_DOPPLER_(gpsSvCount), svData.doppler);
      gpsSvCount++;

      //			LOGV("%s: satId %3d - cno %2d - doppler %8.2f
      // chips %10.5f",
      //				 __FUNCTION__, svData.svid, svData.cno,
      //				 (double) svData.doppler / (1<<12),
      //				 ((double)(svData.redSigtow & 0x1FFFFF)
      //* 1023.0) / 0x200000);
    }
  }

  pDatabase->Set(CDatabase::DATA_UBX_SATELLITES_IN_MEAS_COUNT, gpsSvCount);
  //	LOGV("%s: gpsSvCount %d", __FUNCTION__, gpsSvCount);
}
#endif

bool CProtocolUBX::insertHandler(const MessageId messageId,
                                 std::function<void(const Message &message)> handler)
{
  if (!isValidMessageHandlerId(messageId))
    return false;
  messageHandler_.emplace(messageId, handler);
  return true;
}

void CProtocolUBX::callHandlersForMessage(MessageId messageId, const Message &message)
{
  auto range = messageHandler_.equal_range(messageId);
  for (auto it = range.first; it != range.second; ++it)
  {
    if (it->second)
    {
      UBX_LOG(LCAT_VERBOSE, "Calling Handler for message 0x%04x", it->first);
      it->second(message);
    }
  }
}

void CProtocolUBX::resetHandlers() { messageHandler_.clear(); }

void CProtocolUBX::setReceiverGeneration(int gen) { receiverGen = gen; }
