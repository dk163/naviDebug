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
 * $Id: protocolubx.h 107926 2015-12-10 13:15:23Z fabio.robbiani $
 * $HeadURL:
 *http://svn.u-blox.ch/GPS/SOFTWARE/PRODUCTS/libParser/tags/libParser_v1.04/protocolubx.h
 *$
 *****************************************************************************/

#pragma once

#include "protocol.h"

#include "ubx_messageDef.h"

#include <functional>
#include <unordered_map>
#include <vector>

class CProtocolUBX : public CProtocol
{
public:
  /**
   * @brief  Parse provided message
   *
   * @param pBuffer Pointer to message
   * @param iSize Size of message
   *
   * @return  Full size of message if sucessfully parsed, 0 if message is not
   * yet complete. -1 on error
   */
  virtual int Parse(const unsigned char *pBuffer, int iSize);

  /**
   * @brief  Process provided Message
   * Will process the provided message.
   * It will call the corresponding message processors if available.
   *
   * @param pBuffer Pointer to message
   * @param iSize Size of message
   * @param pDatabase Pointer to database object used for logging the message
   *
   */
  virtual void Process(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);

  /**
   * @brief Return type of this Protocol
   *
   * @return  UBX protocol type
   */
  virtual PROTOCOL_t GetType(void) { return UBX; }

  /**
   * @brief  Parse provided Ubx message
   *
   * @param pBuffer Pointer to message
   * @param iSize Size of message
   *
   * @return  Full size of message if sucessfully parsed, 0 if message is not
   * yet complete. -1 on error
   */
  static int ParseUbx(const unsigned char *pBuffer, int iSize);

  /**
   * @brief Creates new Message on heap from provided parameters
   * Sets ppMsg to pointer to newly created array on heap.
   * It is the responsibility of the user of this message to free the heap space
   * allocated by this function.
   *
   * @param classId Class Id
   * @param msgId Message Id
   * @param pPayload Payload
   * @param iPayloadSize Size of the payload
   * @param ppMsg Pointer which will be set to newly created heap array
   *
   * @return  Size of newly created array if success, zero otherwise
   */
  static unsigned int NewMsg(
    U1 classId, U1 msgId, const void *pPayload, unsigned int iPayloadSize, unsigned char **ppMsg);

  static bool insertHandler(const MessageId messageId,
                            std::function<void(const Message &message)> handler);

  static void resetHandlers();

  static void setReceiverGeneration(int gen);

protected:
  static void ProcessNavStatus(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessNavSvInfo(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessNavPvt(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessMonHw(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  static void ProcessNavSat(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
  template <typename T>
  static void processMessage(const unsigned char *pBuffer, int iSize, MessageId id);

#ifdef SUPL_ENABLED
  static void ProcessRxmMeas(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase);
#endif
private:
  typedef unsigned int MessageId;
  static void callHandlersForMessage(MessageId messageId, const Message &message);
  static std::unordered_multimap<MessageId, std::function<void(const Message &message)>>
    messageHandler_;
  static int receiverGen;
};
