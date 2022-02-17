/*****************************************************************************************
  This is implementation of CAN BUS ASCII protocol based on LAWICEL v1.3 serial protocol
   of CAN232/CANUSB device (http://www.can232.com/docs/can232_v3.pdf)

  Made for Arduino with Seeduino/ElecFreaks CAN BUS Shield based on MCP2515

  Copyright (C) 2015 Anton Viktorov <latonita@yandex.ru>
                                     https://github.com/latonita/can-ascii

  This library is free software. You may use/redistribute it under The MIT License terms.

*****************************************************************************************/

#include <SPI.h>
#include "can-232.h"
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;
CAN_message_t msg1;

uint8_t  _messageRcvd = 0;
uint8_t  _printMsgDetails = 0;

//#define LOGGING_ENABLED

#ifdef LOGGING_ENABLED
#define dbg_begin(x) debug.begin(x)
#define dbg0(x)   debug.print(x)
#define dbg1(x)   debug.println(x)
#define dbg2(x,y) debug.print(x); debug.println(y)
#define dbgH(x)   debug.print(x,HEX)
#else
#define dbg_begin(x)
#define dbg0(x)
#define dbg1(x)
#define dbg2(x,y)
#define dbgH(x)
#endif

#ifdef LOGGING_ENABLED
#define debug Serial
#endif

Can232* Can232::_instance = 0;

Can232* Can232::instance() {
  if (_instance == 0)
    _instance = new Can232();
  return _instance;
}

void Can232::init(INT32U defaultCanSpeed) {
  dbg_begin(LW232_DEFAULT_BAUD_RATE); // logging through software serial
  dbg1("CAN ASCII. Welcome to debug");

  instance()->lw232CanSpeedSelection = defaultCanSpeed;

  Can0.begin();
  Can0.setBaudRate(defaultCanSpeed);
  Serial.printf("Baud rate set to: %d\n", defaultCanSpeed);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();

  //Can0.enableFIFO(0);
  //Can0.enableFIFOInterrupt(0);
  //Can0.setMBFilter(ACCEPT_ALL);

  instance()->initFunc();
}

void Can232::setFilter(INT8U (*userFunc)(INT32U)) {
  instance()->setFilterFunc(userFunc);
}

void Can232::loop() {
  Can0.events();
  instance()->loopFunc();
}

void Can232::serialEvent() {
  instance()->serialEventFunc();
}

void Can232::initFunc() {
  if (!inputString.reserve(LW232_INPUT_STRING_BUFFER_SIZE)) {
    dbg0("inputString.reserve failed in initFunc. less optimal String work is expected");
  }
  // lw232AutoStart = true; //todo: read from eeprom
  // lw232AutoPoll = false; //todo: read from eeprom
  //  lw232TimeStamp = //read from eeprom
  //    lw232Message[0] = 'Z';    lw232Message[1] = '1'; exec();
  //if (lw232AutoStart) {
  inputString = "O\0x0D";
  stringComplete = true;
  loopFunc();
  //}
}

void Can232::setFilterFunc(INT8U (*userFunc)(INT32U)) {
  instance()->userAddressFilterFunc = userFunc;
}

void Can232::loopFunc() {
  if (stringComplete) {
    int len = inputString.length();
    if (len > 0 && len < LW232_FRAME_MAX_SIZE) {
      strcpy((char*)lw232Message, inputString.c_str());
      exec();
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  if (lw232CanChannelMode != LW232_STATUS_CAN_CLOSED) {
    int recv = 0;
    while (CAN_MSGAVAIL == checkReceive() && recv++ < 5) {
      dbg0('+');
      if (CAN_OK == receiveSingleFrame()) {
        Serial.write(LW232_CR);
      }
    }
    Serial.flush();
  }
}
void Can232::serialEventFunc() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == LW232_CR) {
      stringComplete = true;
    }
  }
}

INT8U Can232::exec() {
  dbg2("Command received:", inputString);
  lw232LastErr = parseAndRunCommand();
  switch (lw232LastErr) {
    case LW232_OK:
      Serial.write(LW232_RET_ASCII_OK);
      break;
    case LW232_OK_SMALL:
      Serial.write(LW232_RET_ASCII_OK_SMALL);
      Serial.write(LW232_RET_ASCII_OK);
      break;
    case LW232_OK_BIG:
      Serial.write(LW232_RET_ASCII_OK_BIG);
      Serial.write(LW232_RET_ASCII_OK);
      break;
    case LW232_ERR_NOT_IMPLEMENTED:
      // Choose behavior: will it fail or not when not implemented command comes in. Some can monitors might be affected by this selection.
      Serial.write(LW232_RET_ASCII_ERROR);
      //Serial.write(LW232_RET_ASCII_OK);
      break;
    default:
      Serial.write(LW232_RET_ASCII_ERROR);
  }
  return 0;
}

INT8U Can232::parseAndRunCommand() {
  INT8U ret = LW232_OK;
  INT8U idx = 0;
  INT8U err = 0;

  lw232LastErr = LW232_OK;

  switch (lw232Message[0]) {
    case LW232_CMD_SETUP:
      // Sn[CR] Setup with standard CAN bit-rates where n is 0-9.
      if (lw232CanChannelMode == LW232_STATUS_CAN_CLOSED) {
        idx = HexHelper::parseNibbleWithLimit(lw232Message[1], LW232_CAN_BAUD_NUM);
        lw232CanSpeedSelection = lw232CanBaudRates[idx];
      }
      else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_SETUP_BTR:
      // sxxyy[CR] Setup with BTR0/BTR1 CAN bit-rates where xx and yy is a hex value.
      ret = LW232_ERR; break;
    case LW232_CMD_OPEN:
      // O[CR] Open the CAN channel in normal mode (sending & receiving).
      if (lw232CanChannelMode == LW232_STATUS_CAN_CLOSED) {
        ret = openCanBus();
        if (ret == LW232_OK) {
          lw232CanChannelMode = LW232_STATUS_CAN_OPEN_NORMAL;
        }
      }
      else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_LISTEN:
      // L[CR] Open the CAN channel in listen only mode (receiving).
      if (lw232CanChannelMode == LW232_STATUS_CAN_CLOSED) {
        ret = openCanBus();
        if (ret == LW232_OK) {
          lw232CanChannelMode = LW232_STATUS_CAN_OPEN_LISTEN;
        }
      }
      else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_CLOSE:
      // C[CR] Close the CAN channel.
      if (lw232CanChannelMode != LW232_STATUS_CAN_CLOSED) {
        lw232CanChannelMode = LW232_STATUS_CAN_CLOSED;
      }
      else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_TX11:
      // tiiildd...[CR] Transmit a standard (11bit) CAN frame.
      if (lw232CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
        parseCanStdId();
        lw232PacketLen = HexHelper::parseNibbleWithLimit(lw232Message[LW232_OFFSET_STD_PKT_LEN], LW232_FRAME_MAX_LENGTH + 1);
        for (; idx < lw232PacketLen; idx++) {
          lw232Buffer[idx] = HexHelper::parseFullByte(lw232Message[LW232_OFFSET_STD_PKT_DATA + idx * 2], lw232Message[LW232_OFFSET_STD_PKT_DATA + idx * 2 + 1]);
        }
        INT8U mcpErr = sendMsgBuf(lw232CanId, 0, 0, lw232PacketLen, lw232Buffer);
        if (mcpErr != CAN_OK) {
          ret = LW232_ERR;
        } else if (lw232AutoPoll) {
          ret = LW232_OK_SMALL;
        }
      }
      else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_TX29:
      // Tiiiiiiiildd...[CR] Transmit an extended (29bit) CAN frame
      if (lw232CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
        parseCanExtId();
        lw232PacketLen = HexHelper::parseNibbleWithLimit(lw232Message[LW232_OFFSET_EXT_PKT_LEN], LW232_FRAME_MAX_LENGTH + 1);
        for (; idx < lw232PacketLen; idx++) {
          lw232Buffer[idx] = HexHelper::parseFullByte(lw232Message[LW232_OFFSET_EXT_PKT_DATA + idx * 2], lw232Message[LW232_OFFSET_EXT_PKT_DATA + idx * 2 + 1]);
        }
        if (CAN_OK != sendMsgBuf(lw232CanId, 1, 0, lw232PacketLen, lw232Buffer)) {
          ret = LW232_ERR;
        } else if (lw232AutoPoll) {
          ret = LW232_OK_BIG;
        } else {
          ret = LW232_OK;
        }
      }
      break;
    case LW232_CMD_RTR11:
      // riiil[CR] Transmit an standard RTR (11bit) CAN frame.
      if (lw232CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
        parseCanStdId();
        lw232PacketLen = HexHelper::parseNibbleWithLimit(lw232Message[LW232_OFFSET_STD_PKT_LEN], LW232_FRAME_MAX_LENGTH + 1);
        if (CAN_OK != sendMsgBuf(lw232CanId, 0, 1, lw232PacketLen, lw232Buffer)) {
          ret = LW232_ERR;
        }
        else if (lw232AutoPoll) {
          ret = LW232_OK_SMALL;
        }
      }
      else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_RTR29:
      // Riiiiiiiil[CR] Transmit an extended RTR (29bit) CAN frame.
      if (lw232CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
        parseCanExtId();
        lw232PacketLen = HexHelper::parseNibbleWithLimit(lw232Message[LW232_OFFSET_EXT_PKT_LEN], LW232_FRAME_MAX_LENGTH + 1);
        if (CAN_OK != sendMsgBuf(lw232CanId, 1, 1, lw232PacketLen, lw232Buffer)) {
          ret = LW232_ERR;
        }
        else if (lw232AutoPoll) {
          ret = LW232_OK_SMALL; // not a typo. strangely can232_v3.pdf tells to return "z[CR]", not "Z[CR]" as in 29bit. todo: check if it is error in pdf???
        }
      } else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_POLL_ONE:
      // P[CR] Poll incomming FIFO for CAN frames (single poll)
      if (lw232CanChannelMode != LW232_STATUS_CAN_CLOSED && lw232AutoPoll == LW232_AUTOPOLL_OFF) {
        if (CAN_MSGAVAIL == checkReceive()) {
          ret = receiveSingleFrame();
        }
      } else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_POLL_MANY:
      // A[CR] Polls incomming FIFO for CAN frames (all pending frames)
      if (lw232CanChannelMode != LW232_STATUS_CAN_CLOSED && lw232AutoPoll == LW232_AUTOPOLL_OFF) {
        while (CAN_MSGAVAIL == checkReceive()) {
          ret = ret ^ receiveSingleFrame();
          if (ret != CAN_OK)
            break;
          Serial.write(LW232_CR);
        }
        if (ret == CAN_OK)
          Serial.print(LW232_ALL);
      } else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_FLAGS:
      // F[CR] Read Status Flags.
      // LAWICEL CAN232 and CANUSB have some specific errors which differ from MCP2515/MCP2551 errors. We just return MCP2515 error.
      Serial.print(LW232_FLAG);
      //if (lw232CAN.checkError(&err) == CAN_OK)
      err = 0;
      HexHelper::printFullByte(err & 0xF8);
      break;
    case LW232_CMD_AUTOPOLL:
      // Xn[CR] Sets Auto Poll/Send ON/OFF for received frames.
      if (lw232CanChannelMode == LW232_STATUS_CAN_CLOSED) {
        lw232AutoPoll = (lw232Message[1] == LW232_ON_ONE) ? LW232_AUTOPOLL_ON : LW232_AUTOPOLL_OFF;
        //todo: save to eeprom
      } else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_FILTER:
      // Wn[CR] Filter mode setting. By default CAN232 works in dual filter mode (0) and is backwards compatible with previous CAN232 versions.
      ret = LW232_ERR_NOT_IMPLEMENTED; break;
    case LW232_CMD_ACC_CODE:
      // Mxxxxxxxx[CR] Sets Acceptance Code Register (ACn Register of SJA1000). // we use MCP2515,
      ret = LW232_ERR_NOT_IMPLEMENTED; break;
    case LW232_CMD_ACC_MASK:
      // mxxxxxxxx[CR] Sets Acceptance Mask Register (AMn Register of SJA1000).
      ret = LW232_ERR_NOT_IMPLEMENTED; break;
    case LW232_CMD_UART:
      // Un[CR] Setup UART with a new baud rate where n is 0-6.
      idx = HexHelper::parseNibbleWithLimit(lw232Message[1], LW232_UART_BAUD_NUM);
      Serial.begin(lw232SerialBaudRates[idx]);
      break;
    case LW232_CMD_VERSION1:
    case LW232_CMD_VERSION2:
      // V[CR] Get Version number of both CAN232 hardware and software
      Serial.print(LW232_LAWICEL_VERSION_STR);
      break;
    case LW232_CMD_SERIAL:
      // N[CR] Get Serial number of the CAN232.
      Serial.print(LW232_LAWICEL_SERIAL_NUM);
      break;
    case LW232_CMD_TIMESTAMP:
      // Zn[CR] Sets Time Stamp ON/OFF for received frames only. Z0 - OFF, Z1 - Lawicel's timestamp 2 bytes, Z2 - arduino timestamp 4 bytes.
      if (lw232CanChannelMode == LW232_STATUS_CAN_CLOSED) {
        // lw232TimeStamp = (lw232Message[1] == LW232_ON_ONE);
        if (lw232Message[1] == LW232_ON_ONE) {
          lw232TimeStamp = LW232_TIMESTAMP_ON_NORMAL;
        }
        else if (lw232Message[1] == LW232_ON_TWO) {
          lw232TimeStamp = LW232_TIMESTAMP_ON_EXTENDED;
        }
        else {
          lw232TimeStamp = LW232_TIMESTAMP_OFF;
        }
      }
      else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_AUTOSTART:
      // Qn[CR] Auto Startup feature (from power on).
      if (lw232CanChannelMode != LW232_STATUS_CAN_CLOSED) {
        if (lw232Message[1] == LW232_ON_ONE) {
          lw232AutoStart = LW232_AUTOSTART_ON_NORMAL;
        }
        else if (lw232Message[1] == LW232_ON_TWO) {
          lw232AutoStart = LW232_AUTOSTART_ON_LISTEN;
        }
        else {
          lw232AutoStart = LW232_AUTOSTART_OFF;
        }
        //todo: save to eeprom
      }
      else {
        ret = LW232_ERR;
      }
      break;
    case LW232_CMD_CUSTOM:
      if(_printMsgDetails) {
        _printMsgDetails = 0;
      } else {
        _printMsgDetails = 1;
      }
      break;
    default:
      ret = LW232_ERR_UNKNOWN_CMD;
  }

  return ret;
}

INT8U Can232::checkReceive() {
  if ( _messageRcvd ) {
    _messageRcvd = 0;
    return CAN_MSGAVAIL;
  }
  return CAN_NOMSG;
}

void Can232::canSniff(const CAN_message_t &msg) {
  _messageRcvd = 1;
  if(_printMsgDetails) {
    //Serial.print("  MB "); Serial.print(msg.mb);
    Serial.print(" ID: "); Serial.print(msg.id, HEX);
    Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" EXT: "); Serial.print(msg.flags.extended);
    Serial.print(" RTR: "); Serial.print(msg.flags.remote);
    Serial.print(" TS: "); Serial.print(msg.timestamp);
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.print(msg.buf[i], HEX); Serial.print(" ");
    } Serial.println();
  }
  msg1 = msg;
}


INT8U Can232::readMsgBufID(INT32U *ID, INT8U *len, INT8U buf[]) {
  *ID = msg1.id;
  //*ext = storage_mcp_sim.flags.extended;
  *len = msg1.len;
  //*rtrBit = storage_mcp_sim.flags.remote;
  for ( uint8_t i = 0; i < 8; i++ ) buf[i] = msg1.buf[i];
  return CAN_OK;
}



INT8U Can232::receiveSingleFrame() {
  INT8U ret = LW232_OK;
  INT8U idx = 0;
  if (CAN_OK == readMsgBufID(&lw232CanId, &lw232PacketLen, lw232Buffer)) {
    if (lw232CanId > 0x1FFFFFFF) {
      ret = LW232_ERR; // address if totally wrong
    }
    else if (checkPassFilter(lw232CanId)) {// do we want to skip some addresses?
      if (isExtendedFrame()) {
        Serial.print(LW232_TR29);
        HexHelper::printFullByte(HIGH_BYTE(HIGH_WORD(lw232CanId)));
        HexHelper::printFullByte(LOW_BYTE(HIGH_WORD(lw232CanId)));
        HexHelper::printFullByte(HIGH_BYTE(LOW_WORD(lw232CanId)));
        HexHelper::printFullByte(LOW_BYTE(LOW_WORD(lw232CanId)));
      }
      else {
        Serial.print(LW232_TR11);
        HexHelper::printNibble(HIGH_BYTE(LOW_WORD(lw232CanId)));
        HexHelper::printFullByte(LOW_BYTE(LOW_WORD(lw232CanId)));
      }
      //write data len
      HexHelper::printNibble(lw232PacketLen);
      //write data
      for (idx = 0; idx < lw232PacketLen; idx++) {
        HexHelper::printFullByte(lw232Buffer[idx]);
      }
      //write timestamp if needed
      if (lw232TimeStamp != LW232_TIMESTAMP_OFF) {
        INT32U time = millis();
        if (lw232TimeStamp == LW232_TIMESTAMP_ON_NORMAL) {
          // standard LAWICEL protocol. two bytes.
          time %= 60000;
        } else {
          // non standard protocol - 4 bytes timestamp
          HexHelper::printFullByte(HIGH_BYTE(HIGH_WORD(time)));
          HexHelper::printFullByte(LOW_BYTE(HIGH_WORD(time)));
        }
        HexHelper::printFullByte(HIGH_BYTE(LOW_WORD(time)));
        HexHelper::printFullByte(LOW_BYTE(LOW_WORD(time)));
      }
    }
  }
  else {
    ret = LW232_ERR;
  }
  Serial.println();
  return ret;
}


INT8U Can232::isExtendedFrame() {
  return msg1.flags.extended;
}


INT8U Can232::checkPassFilter(INT32U addr) {
  if (userAddressFilterFunc == 0)
    return LW232_FILTER_PROCESS;

  return (*userAddressFilterFunc)(addr);
}

INT8U Can232::openCanBus() {
  INT8U ret = LW232_OK;
  Serial.println(lw232CanSpeedSelection);
  Can0.setBaudRate(lw232CanSpeedSelection);
  Serial.println(lw232CanSpeedSelection);
  return ret;
}


INT8U Can232::sendMsgBuf(INT32U id, INT8U extflag, INT8U rtrflag, INT8U len, INT8U *buf) {
  //return lw232CAN.sendMsgBuf(id, ext, rtr, len, buf);
  CAN_message_t frame;
  frame.id = id;
  frame.flags.extended = extflag;
  frame.flags.remote = rtrflag;
  frame.len = len;
  memmove(&frame.buf, &buf[0], len);
  return Can0.write(frame);
}


void Can232::parseCanStdId() {
  lw232CanId = (((INT32U)HexHelper::parseNibble(lw232Message[1])) << 8)
               + (((INT32U)HexHelper::parseNibble(lw232Message[2])) << 4)
               + (((INT32U)HexHelper::parseNibble(lw232Message[3])));
  lw232CanId &= 0x7FF;
}

void Can232::parseCanExtId() {
  lw232CanId = (((INT32U)HexHelper::parseNibble(lw232Message[1])) << 28)
               + (((INT32U)HexHelper::parseNibble(lw232Message[2])) << 24)
               + (((INT32U)HexHelper::parseNibble(lw232Message[3])) << 20)
               + (((INT32U)HexHelper::parseNibble(lw232Message[4])) << 16)
               + (((INT32U)HexHelper::parseNibble(lw232Message[5])) << 12)
               + (((INT32U)HexHelper::parseNibble(lw232Message[6])) << 8)
               + (((INT32U)HexHelper::parseNibble(lw232Message[7])) << 4)
               + (((INT32U)HexHelper::parseNibble(lw232Message[8])));
  lw232CanId &= 0x1FFFFFFF;
}

void HexHelper::printFullByte(INT8U b) {
  if (b < 0x10) {
    Serial.print('0');
    // dbg0('0');
  }
  Serial.print(b, HEX);
  //dbgH(b);
}

void HexHelper::printNibble(INT8U b) {
  Serial.print(b & 0x0F, HEX);
  //dbgH(b & 0x0F);
}


INT8U HexHelper::parseNibble(INT8U hex) {
  INT8U ret = 0;
  if (hex >= '0' && hex <= '9') {
    ret = hex - '0';
  } else if (hex >= 'a' && hex <= 'f') {
    ret = hex - 'a' + 10;
  } else if (hex >= 'A' && hex <= 'F') {
    ret = hex - 'A' + 10;
  } // else error, return 0
  return ret;
}

INT8U HexHelper::parseFullByte(INT8U H, INT8U L) {
  return (parseNibble(H) << 4) + parseNibble(L);
}

INT8U HexHelper::parseNibbleWithLimit(INT8U hex, INT8U limit) {
  INT8U ret = parseNibble(hex);
  if (ret < limit)
    return ret;
  else
    return 0;
}
