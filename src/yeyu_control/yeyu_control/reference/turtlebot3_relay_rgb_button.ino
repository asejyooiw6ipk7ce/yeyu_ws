#include <Arduino.h>

const uint32_t SERIAL_BAUD = 115200;

// Pin map
const uint8_t PIN_RELAY = 10;
const uint8_t PIN_RGB_R = 5;
const uint8_t PIN_RGB_G = 6;
const uint8_t PIN_RGB_B = 9;
const uint8_t PIN_BUTTON = 2;

// Set true when using common-anode RGB LED.
const bool COMMON_ANODE_RGB = false;

// Protocol constants
const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE_1 = 0xAA;
const uint8_t END_BYTE_2 = 0xEE;

const uint8_t PKT_SENSOR_STATE = 0x31;
const uint8_t PKT_SET_RELAY    = 0x41;
const uint8_t PKT_SET_RGB      = 0x42;
const uint8_t PKT_SET_ALL      = 0x43;
const uint8_t PKT_PING         = 0x7F;

const uint8_t MAX_PAYLOAD_LEN = 32;

// Device states
uint8_t relay_state = 0;
uint8_t button_state = 0;
uint8_t rgb_r = 0;
uint8_t rgb_g = 0;
uint8_t rgb_b = 0;

uint8_t tx_sequence = 0;

// Button debounce
uint8_t last_button_raw = HIGH;
uint8_t stable_button_raw = HIGH;
unsigned long last_button_change_ms = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 30;

// Periodic publish
unsigned long last_publish_ms = 0;
const unsigned long PUBLISH_PERIOD_MS = 100;

// RX parser state
enum ParserState
{
  WAIT_START_1,
  WAIT_START_2,
  WAIT_START_3,
  READ_PACKET_ID,
  READ_LENGTH,
  READ_SEQUENCE,
  READ_PAYLOAD,
  READ_CHECKSUM,
  READ_END_1,
  READ_END_2
};

ParserState parser_state = WAIT_START_1;

uint8_t rx_packet_id = 0;
uint8_t rx_length = 0;
uint8_t rx_sequence = 0;
uint8_t rx_payload[MAX_PAYLOAD_LEN];
uint8_t rx_payload_index = 0;
uint8_t rx_checksum = 0;

uint8_t calcChecksum(uint8_t packet_id,
                     uint8_t length,
                     uint8_t sequence,
                     const uint8_t *payload)
{
  uint16_t sum = 0;

  sum += packet_id;
  sum += length;
  sum += sequence;

  for (uint8_t i = 0; i < length; i++)
  {
    sum += payload[i];
  }

  return (uint8_t)(sum & 0xFF);
}

void sendPacket(uint8_t packet_id,
                const uint8_t *payload,
                uint8_t length)
{
  uint8_t checksum = calcChecksum(packet_id, length, tx_sequence, payload);

  Serial.write(START_BYTE);
  Serial.write(START_BYTE);
  Serial.write(START_BYTE);

  Serial.write(packet_id);
  Serial.write(length);
  Serial.write(tx_sequence);

  for (uint8_t i = 0; i < length; i++)
  {
    Serial.write(payload[i]);
  }

  Serial.write(checksum);

  Serial.write(END_BYTE_1);
  Serial.write(END_BYTE_2);

  tx_sequence++;
}

void applyRelay(uint8_t state)
{
  relay_state = state ? 1 : 0;
  digitalWrite(PIN_RELAY, relay_state ? HIGH : LOW);
}

void writeRgbPin(uint8_t pin, uint8_t value)
{
  if (COMMON_ANODE_RGB)
  {
    analogWrite(pin, 255 - value);
  }
  else
  {
    analogWrite(pin, value);
  }
}

void applyRgb(uint8_t r, uint8_t g, uint8_t b)
{
  rgb_r = r;
  rgb_g = g;
  rgb_b = b;

  writeRgbPin(PIN_RGB_R, rgb_r);
  writeRgbPin(PIN_RGB_G, rgb_g);
  writeRgbPin(PIN_RGB_B, rgb_b);
}

void readButtonDebounced()
{
  uint8_t raw = digitalRead(PIN_BUTTON);
  unsigned long now = millis();

  if (raw != last_button_raw)
  {
    last_button_change_ms = now;
    last_button_raw = raw;
  }

  if ((now - last_button_change_ms) > BUTTON_DEBOUNCE_MS)
  {
    stable_button_raw = raw;
  }

  // INPUT_PULLUP mode:
  // LOW means pressed.
  button_state = (stable_button_raw == LOW) ? 1 : 0;
}

void sendSensorState()
{
  uint8_t payload[5];

  payload[0] = relay_state;
  payload[1] = button_state;
  payload[2] = rgb_r;
  payload[3] = rgb_g;
  payload[4] = rgb_b;

  sendPacket(PKT_SENSOR_STATE, payload, 5);
}

void handlePacket(uint8_t packet_id,
                  uint8_t length,
                  uint8_t sequence,
                  const uint8_t *payload)
{
  (void)sequence;

  if (packet_id == PKT_SET_RELAY)
  {
    if (length != 1)
    {
      return;
    }

    applyRelay(payload[0]);
    sendSensorState();
  }
  else if (packet_id == PKT_SET_RGB)
  {
    if (length != 3)
    {
      return;
    }

    applyRgb(payload[0], payload[1], payload[2]);
    sendSensorState();
  }
  else if (packet_id == PKT_SET_ALL)
  {
    if (length != 4)
    {
      return;
    }

    applyRelay(payload[0]);
    applyRgb(payload[1], payload[2], payload[3]);
    sendSensorState();
  }
  else if (packet_id == PKT_PING)
  {
    // Echo current state as a simple response.
    sendSensorState();
  }
}

void resetParser()
{
  parser_state = WAIT_START_1;
  rx_packet_id = 0;
  rx_length = 0;
  rx_sequence = 0;
  rx_payload_index = 0;
  rx_checksum = 0;
}

void parseByte(uint8_t byte_in)
{
  switch (parser_state)
  {
    case WAIT_START_1:
      if (byte_in == START_BYTE)
      {
        parser_state = WAIT_START_2;
      }
      break;

    case WAIT_START_2:
      if (byte_in == START_BYTE)
      {
        parser_state = WAIT_START_3;
      }
      else
      {
        parser_state = WAIT_START_1;
      }
      break;

    case WAIT_START_3:
      if (byte_in == START_BYTE)
      {
        parser_state = READ_PACKET_ID;
      }
      else
      {
        parser_state = WAIT_START_1;
      }
      break;

    case READ_PACKET_ID:
      rx_packet_id = byte_in;
      parser_state = READ_LENGTH;
      break;

    case READ_LENGTH:
      rx_length = byte_in;

      if (rx_length > MAX_PAYLOAD_LEN)
      {
        resetParser();
        return;
      }

      rx_payload_index = 0;
      parser_state = READ_SEQUENCE;
      break;

    case READ_SEQUENCE:
      rx_sequence = byte_in;

      if (rx_length == 0)
      {
        parser_state = READ_CHECKSUM;
      }
      else
      {
        parser_state = READ_PAYLOAD;
      }
      break;

    case READ_PAYLOAD:
      rx_payload[rx_payload_index] = byte_in;
      rx_payload_index++;

      if (rx_payload_index >= rx_length)
      {
        parser_state = READ_CHECKSUM;
      }
      break;

    case READ_CHECKSUM:
      rx_checksum = byte_in;
      parser_state = READ_END_1;
      break;

    case READ_END_1:
      if (byte_in == END_BYTE_1)
      {
        parser_state = READ_END_2;
      }
      else
      {
        resetParser();
      }
      break;

    case READ_END_2:
      if (byte_in == END_BYTE_2)
      {
        uint8_t calculated = calcChecksum(rx_packet_id,
                                          rx_length,
                                          rx_sequence,
                                          rx_payload);

        if (calculated == rx_checksum)
        {
          handlePacket(rx_packet_id,
                       rx_length,
                       rx_sequence,
                       rx_payload);
        }
      }

      resetParser();
      break;

    default:
      resetParser();
      break;
  }
}

void readSerial()
{
  while (Serial.available() > 0)
  {
    uint8_t b = (uint8_t)Serial.read();
    parseByte(b);
  }
}

void setup()
{
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  applyRelay(0);
  applyRgb(0, 0, 0);

  Serial.begin(SERIAL_BAUD);

  delay(500);
  sendSensorState();
}

void loop()
{
  readSerial();
  readButtonDebounced();

  unsigned long now = millis();

  if ((now - last_publish_ms) >= PUBLISH_PERIOD_MS)
  {
    last_publish_ms = now;
    sendSensorState();
  }
}