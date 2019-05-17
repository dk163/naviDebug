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

#pragma once

/*!
    \file
    Definition of \ref CUbxGpsState

    \brief
    This file contains the definition of \ref CUbxGpsState, which handles
    the state management for the u-blox GNSS receiver.
*/

#include "agnss/Agnss.h"
#include "gps_thread.h"
#include "std_inc.h"
#include "std_types.h"
#include "ubx_messageDef.h"
#include "ubx_serial.h"
#include "ubx_udpServer.h"
#include <array>
#include <memory>
#include <mutex>
#include <string>

// Forward declarations
class CCfg;

/*! \class CUbxGpsState

    \brief
    This class contain the state management for the u-blox GNSS receiver.
    It handles sending of commands to the receiver, collection and sending of
    aiding information.

    It has implemented the following feature:
    - Time and Position Injection with UBX-AID-INI
    - Local Aiding of Ephemeris, Almanac, Ionosphere, UTC data and Health
    - AssistNow Autonomous Aiding
    - AssistNow Offline (Server based, not Flash based)
    - Configuration of the receiver (e.g Rate, Baud-rate, Messages)
*/
// lint -sem(CUbxGpsState::lock,thread_lock)
// lint -sem(CUbxGpsState::unlock,thread_unlock)
class CUbxGpsState
{
public:
  //! Defines the states of the receiver
  typedef enum {
    OFF = 0, //!< The receiver is off
    ON = 1   //!< The receiver is on
  } RECV_STATE_t;

  // Constructor
  CUbxGpsState();
  // Destructor
  virtual ~CUbxGpsState();

  static CUbxGpsState *getInstance(void);

  // Event handling
  bool init(CSerialPort *pSer);
  void init(CCfg &pConfig);
  void updateConfig(CCfg &pConfig);
  bool turnRecv(RECV_STATE_t newMode);
  bool isRecv(RECV_STATE_t mode);
  bool isRecvInTransTo(RECV_STATE_t mode);
  void onNewMsg(const unsigned char *pMsg, unsigned int iMsg);

  // Navigation Rate Control
  void putRate(int16_t rateMs);
  int16_t getRate(void) const { return m_Db.rateMs; };

  // Local Aiding EPH,ALM,AOP,HUI
  bool writeUbxNavX5AopEnable(void);

  // Position And Time Aiding/Injection
  void putTime(GpsUtcTime timeUtcMs, int64_t timeRefMs, int accMs);
  void putPos(double latDeg, double lonDeg, float accM);
  bool sendPosAndTime();

  // AssistNow Offline
  bool putNewOfflineData(const unsigned char *pData, unsigned int iData);

#if defined UDP_SERVER_PORT
  void setUdp(CUdpServer *pUdpServer) { m_pUdpServer = pUdpServer; };
#endif

  const char *getSerialDevice(void) const { return m_pSerialDevice.c_str(); };
  int getBaudRate(void) const { return m_baudRate; };
  int getBaudRateDefault(void) const { return m_baudRateDef; };
  TX_READY_CONF_t const *getI2cTxReady(void) { return &m_i2cTxReady; };
  int getStoppingTimeoutMs(void) const { return m_stoppingTimeoutMs; };
  int getReceiverGeneration(void) const { return m_receiverGeneration; };
  const char *getAgnssToken(void) const { return m_pAgnssToken.c_str(); };

  // Threading help
  void lock(void);
  void unlock(void);

  bool writeUbxCfgRst(int resetMode, unsigned short flags);

#ifdef SUPL_ENABLED
  void sendEph(const void *pData, int size);
  void sendAidingData(const GPS_UBX_AID_INI_U5__t *pAidingData);
  void sendUtcModel(const GPS_UBX_AID_HUI_t *pUtcModel);

  // SUPL requests
  int getAlamanacRequest(void) const { return m_almanacRequest; };
  int getUtcModelRequest(void) const { return m_utcModelRequest; };
  int getIonosphericModelRequest(void) const { return m_ionosphericModelRequest; };
  int getDgpsCorrectionsRequest(void) const { return m_dgpsCorrectionsRequest; };
  int getRefLocRequest(void) const { return m_refLocRequest; };
  int getRefTimeRequest(void) const { return m_refTimeRequest; };
  int getAcquisitionAssistRequest(void) const { return m_acquisitionAssistRequest; };
  int getRealTimeIntegrityRequest(void) const { return m_realTimeIntegrityRequest; };
  int getNavigationModelRequest(void) const { return m_navigationModelRequest; };
  int getFakePhone(void) const { return m_fakePhone; };
  int getNiUiTimeout(void) const { return m_niUiTimeout; };
  int getNiResponseTimeout(void) const { return m_niResponseTimeout; };
  bool getLogSuplMessages(void) const { return m_logSuplMessages; };
  bool getCmccLogActive(void) const { return m_cmccLogActive; };
  bool getSuplMsgToFile(void) const { return m_suplMsgToFile; };
#endif

protected:
  // Set in the configuration file
  std::string m_pSerialDevice;         //!< Serial device path connecting Gps receiver
  int m_baudRate;                      //!< General baud rate to communicate with receiver
  int m_baudRateDef;                   //!< Initial baud rate to communicate with receiver
  TX_READY_CONF_t m_i2cTxReady;        //!< The configuration of the TX-Ready feature for I2C
  int m_stoppingTimeoutMs;             //!< Maximum time (in ms) to wait for receiver
                                       //! acknowlegements during 'stopping'
  int m_receiverGeneration;            //!< Receiver generation of the attached u-blox GNSS
                                       //! product
  std::string m_pAgnssToken;           //!< Token to authenticate to u-blox services
  int m_agnssStrategy;                 //!< Which AssistNow strategy is enabled?
  int m_agnssPersistence;              //!< Persistence flag. True - save alp database to
                                       //! file. False - don't
  std::string m_pAgnssPersistenceFile; //!< Path and filename to store alp data
  int m_timeSource;                    //!< Time source to use for aiding
  int m_agnssOnlineInterval;           //!< Interval between polling AssistNow Offline
                                       //! server (in ms)
  int m_agnssOfflineInterval;          //!< Interval between polling AssistNow Offline
                                       //! server (in ms)
  std::string m_pAgnssOnlineServer1;   //!< Primary AssistNow Online server
  std::string m_pAgnssOnlineServer2;   //!< Secondary AssistNow Online server
  std::string m_pAgnssOfflineServer1;  //!< Primary AssistNow Offline server
  std::string m_pAgnssOfflineServer2;  //!< Secondary AssistNow Offline server

  // Set dynamically
  int64_t m_timeLastOfflineReq{0}; //!< Limit Xtra requests over time
  int64_t m_timeLastOnlineReq{0};  //!< Limit Online requests over time
  std::array<bool, CAgnss::_NUM_SERVICES_>
    m_transferRequired{}; //!< Flags to check if transfer is required
  std::array<uint16_t, CAgnss::_NUM_SERVICES_>
    m_currentChecksum{};                     //!< Checksum of last data transfer sent to receiver
  bool m_dbChanged{false};                   //!< Indicates if data changed
  size_t m_consFailedTransf{0};              //!< Counts the number
                                             //!< of consecutive
                                             //!< failed transfers
  RECV_STATE_t m_currState{OFF};             //!< The state of the receiver
  bool m_inTransition{false};                //!< Is the receiver in transition to the other state
  bool m_pollRecvStateFinished{false};       //!< Has a receiver state poll request finished?
  int64_t m_timeoutStart{0};                 //!< Time the shutdown timer started to count
  static const size_t m_maxConsFailedTransf; //!< Max number of
                                             //!< consecutively
                                             //!< failed transfers

  CSerialPort *m_pSer{}; //!< Pointer to serial communications class instance
#if defined UDP_SERVER_PORT
  CUdpServer *m_pUdpServer{}; //!< Pointer to UDP communications class instance
#endif
  std::mutex m_ubxStateMutex;

  // Return the current Reference Time
  static int64_t currentRefTimeMs(void);

  static const uint8_t DB_VERSION_NO; //!< Version of the database file format
  enum
  {
    NUM_GPS_SVS = 32,     //!< maximum number of SV supported (just GPS)
    MSTIME_ACC_COMM = 200 //!< Default Accuarcy added to the time aiding
  };

  //! The full assistance database
  struct DB_s
  {
    uint16_t rateMs;                    //!< The requested measurement rate
    BUF_t aopG[NUM_GPS_SVS];            //!< per Satellite Database of AOP messages
    std::unique_ptr<CAgnss> pAgnssIf{}; //!< AssistNow* interface
  } m_Db;                               //!< The database used for aiding and persistent information

#ifdef SUPL_ENABLED
  int m_almanacRequest;           //!< If true, request almanac assistance data in Supl
                                  //! transaction
  int m_utcModelRequest;          //!< If true, request utc model assistance data in Supl
                                  //! transaction
  int m_ionosphericModelRequest;  //!< If true, request iono model assistance
                                  //! data in Supl transaction
  int m_dgpsCorrectionsRequest;   //!< If true, request dgps correction assistance
                                  //! data in Supl transaction
  int m_refLocRequest;            //!< If true, request reference location assistance data
                                  //! in Supl transaction
  int m_refTimeRequest;           //!< If true, request reference time assistance data in
                                  //! Supl transaction
  int m_acquisitionAssistRequest; //!< If true, request aquisition assistance
                                  //! data in Supl transaction
  int m_realTimeIntegrityRequest; //!< If true, request real time integrity
                                  //! assistance data in Supl transaction
  int m_navigationModelRequest;   //!< If true, request navigational model
                                  //! assistance data in Supl transaction
  int m_fakePhone;                //!< Fake phone info flag. True - Use contrived IMSI, MCC,
                                  //! MNC, LAC etc. False - get from framework
  int m_niUiTimeout;              //!< Time out (in seconds) for a Network Initiated UI
                                  //! response
  int m_niResponseTimeout;        //!< Time out (in seconds) for a NI Notify/Verify
                                  //! dialog response
  bool m_logSuplMessages;         //!< If true, log SUPL & RRLP messages
  bool m_cmccLogActive;           //!< If true, generate CMCC logging
  bool m_suplMsgToFile;           //!< If true, redirect Supl & RRLP messages to log file
#endif

  void loadConfiguration();
  void loadConfiguration(const CCfg &cfg);
  void updateConfiguration(const CCfg &cfg);
  void hardResetDevice();
  void initializeAiding();
  // Handle receiver state changes
  bool changeState(RECV_STATE_t targetState, bool transition);
  void prepareTurnOff(void);
  void turnOn(void);
  void forceTurnOff(void);

  // Reconfigure update rate
  bool writeUbxCfgRate(void);

  // Reconfigure baud rate
  void setBaudRate();

  // Get latest data from server
  void initDownloadIfReq();

  // Transfer data to the receiver
  void initTransferIfReq();

  // configuration of the services
  bool isAopEnabled();
  bool isOfflineEnabled();
  bool isOnlineEnabled();

  void saveAiding(void);
  bool loadAiding(void);
  void destroyAgnssIf(void);

  bool saveDatabase(int fd, struct DB_s *pDb);
  bool loadDatabase(int fd, struct DB_s *pDb);

  // aop functions
  void sendAopIfRequired();

  // UBX Message creation and writing
  char const *strUbxCfgMsg(unsigned char clsId, unsigned char msgId);
  bool writeUbxCfgPort(int portId, int baudRate);
  bool writeUbxCfgMsg(unsigned char clsId, unsigned char msgId, bool enable = true);
  static void crcUbx(unsigned char crc[2], const unsigned char *pData, int iData);
  bool writeUbx(unsigned char classID,
                unsigned char msgID,
                const void *pData0,
                size_t iData0,
                const void *pData1 = NULL,
                size_t iData1 = 0);

  // Transfer setting/getting
  void setTransferRequired(CAgnss::SERVICE_t service, bool required);
  bool isTransferPossibleAndRequired(CAgnss::SERVICE_t service);

  // Checksum setting/getting: default (lock/unlock), locked, lockless
  void setCurrentChecksum(CAgnss::SERVICE_t service, uint16_t checksum);
  bool isCurrentChecksum(CAgnss::SERVICE_t service, uint16_t checksum);

  // DB changed setting/getting: default (lock/unlock), locked, lockless
  void setDbChanged(bool changed);
  bool hasDbChanged();

  // Power handling
  static bool powerOn(void);
  static void powerOff(void);

  // Accessed by CAgnss
  static ssize_t writeToRcv(void const *context, unsigned char const *buf, size_t size);
  ssize_t writeToRcv(unsigned char const *buf, size_t size);
  static void finished(void const *context,
                       CAgnss::SERVICE_t service,
                       CAgnss::ACTION_t action,
                       bool success,
                       uint16_t dataId);
  void finished(CAgnss::SERVICE_t service, CAgnss::ACTION_t action, bool success, uint16_t dataId);
  static int printStd(void const *context, const char *str);
  int printStd(const char *str);
  static int printErr(void const *context, const char *str);
  int printErr(const char *str);
};
