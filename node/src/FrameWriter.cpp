#include "FrameWriter.hpp"

using std::uint8_t;
using std::uint16_t;

namespace {
  // Radio setup, Adafruit LoRa feather
  constexpr uint8_t RFM95_CS = 8;
  constexpr uint8_t RFM95_RST = 4;
  constexpr uint8_t RFM95_INT = 3;
  constexpr float   RFM95_FREQ = 915.0; // MHz

  // Message constants
  constexpr uint16_t LEN = 256;
  constexpr uint8_t STX = 0x01;
  constexpr uint8_t ETX = 0x03;
  constexpr uint8_t ESC = 0xfe;
}

namespace SensorNode {

FrameWriter::FrameWriter() :
    _radio(RH_RF95(RFM95_CS, RFM95_INT)) {}

void FrameWriter::sendMsg(uint8_t *vals, uint16_t length) {
  _setUpBuffer(vals, length);

  if(!_isSetUp)
    _setup();

  _radio.send(_buffer, _cursor);
}

bool FrameWriter::_setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if(!_radio.init())
    return false;

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if(!_radio.setFrequency(RFM95_FREQ))
    return false;
  if(!_radio.setModemConfig(RH_RF95::ModemConfigChoice::Bw500Cr45Sf128))
    return false;
  _radio.setTxPower(23, false);

  _isSetUp = true;
  return true;
}

void FrameWriter::_setUpBuffer(uint8_t *vals, uint16_t length) {
  _addToBuffer(STX);

  for(uint16_t i = 0; i < length; i++) {
    _addMessageByte(vals[i]);
  }

  _addToBuffer(ETX);
}

bool FrameWriter::_valNeedsEscape(uint8_t val) {
  switch(val) {
    case 0x01:
    case 0x03:
    case 0xfe:
      return true;
    default:
      return false;
  }
}

void FrameWriter::_addMessageByte(uint8_t val) {
  if(_valNeedsEscape(val))
    _addToBuffer(ESC);

  _addToBuffer(val);
}

void FrameWriter::_addToBuffer(uint8_t val) {
  if(_cursor < LEN) {
    _buffer[_cursor] = val;
    _cursor++;
  }
}

}