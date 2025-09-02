#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"
#include <avr/wdt.h>
#include <ArduinoSTL.h>

// A lot of this file is duplicated on the brain-side of things.
// Code here is a bit more oriented toward the Arduino infrastructure, though.
// For a better understanding, also look at the brain side software for communication.

QwiicOTOS otos;
const int timeout{ 5 };

// Used for interpretting data in the form of individual bytes.
template<typename T>
union PiecedData {
  T value;
  uint8_t bytes[sizeof(T)];
};

struct Packet {
  Packet() = default;

  Packet(const uint8_t iID, const std::vector<uint8_t> &iData = {})
    : id{ iID }, data{ iData } {
    checksum = id;
    for (uint8_t byte : data) {
      checksum ^= byte;
    }
  }

  uint8_t id{ 0 };
  uint8_t checksum{ 0 };
  std::vector<uint8_t> data{};

  // Determines if the partity byte matches the rest of the packet.
  bool correct() const {
    uint8_t parityByte{ id };
    parityByte ^= checksum;
    for (uint8_t byte : data) {
      parityByte ^= byte;
    }
    return !parityByte;
  }

  // Writes the packet to serial.
  void write() {
    digitalWrite(2, HIGH);  // Allow writing to the RS485 chip.
    Serial.write(id);
    Serial.write(checksum);
    for (uint8_t byte : data) {
      Serial.write(byte);
    }
  }
};

// Reads a single packet from serial.
Packet readPacket() {
  Packet packet;
  packet.id = Serial.read();
  packet.checksum = Serial.read();
  while (Serial.available()) {
    packet.data.push_back(Serial.read());
  }
  return packet;
}

enum Command {
  Initialize,
  Calibrate,
  IsCalibrating,
  Reset,
  ResetTracking,
  SetOffset,
  SetPosition,
  GetPosition,
  GetVelocity,
  Check,
  SelfTest,
  Invalid  // ALL NEW COMMANDS SHOULD BE PLACED ABOVE THIS ONE!
};

enum Response {
  Success,
  Error,
  Waiting,
  Unknown
};

void setup() {
  Serial.begin(115200);

  // Clearing the input buffer.
  while (Serial.available()) {
    Serial.read();
  }

  Wire.begin();

  otos.begin();

  pinMode(2, OUTPUT);
}

void loop() {
  digitalWrite(2, LOW);  // Allow reading from the RS485 chip.
  // Wait for message ID and checksum to get through.
  if (Serial.available() >= 2) {
    const int startTime{ millis() };
    while (Serial.available() < getExpectedBytes() && millis() - startTime < timeout) {}
    Packet packet{ readPacket() };
    if (!packet.correct()) {
      packet.id = Command::Invalid;
    }

    switch (packet.id) {
      case Command::Initialize:
        {
          // Hard resets the Arduino. Should be called upon program start and given a few milliseconds to perform.
          // You should expect this command to timeout on the cortex side.
          wdt_disable();
          wdt_enable(WDTO_15MS);
          while (true) {}
        }
        break;
      case Command::Calibrate:
        {
          const bool success{ !otos.calibrateImu(255, false) };
          Packet{ success ? Response::Success : Response::Error }.write();
        }
        break;
      case Command::IsCalibrating:
        {
          uint8_t numOfSamples;
          const bool success{ !otos.getImuCalibrationProgress(numOfSamples) };
          if (success) {
            Packet{ numOfSamples ? Response::Waiting : Response::Success }.write();
          } else {
            Packet{ Response::Error }.write();
          }
        }
        break;
      case Command::Reset:
        {
          const bool success{ !otos.resetTracking() };
          Packet{ success ? Response::Success : Response::Error }.write();
        }
        break;
      case Command::ResetTracking:
        {
          const bool success{ !otos.resetTracking() };
          Packet{ success ? Response::Success : Response::Error }.write();
        }
        break;
      case Command::SetOffset:
        {
          PiecedData<sfe_otos_pose2d_t> pos;
          for (int i { 0 }; i < 12; i++) {
            pos.bytes[i] = packet.data[i];
          }
          const bool success { !otos.setOffset(pos.value) };
          Packet{ success ? Response::Success : Reponse::Unknown }.write();
        }
        break;
      case Command::SetPosition:
        {
          PiecedData<sfe_otos_pose2d_t> pos;
          for (int i{ 0 }; i < 12; i++) {
            pos.bytes[i] = packet.data[i];
          }
          const bool success{ !otos.setPosition(pos.value) };
          Packet{ success ? Response::Success : Response::Unknown }.write();
        }
        break;
      case Command::GetPosition:
        {
          PiecedData<sfe_otos_pose2d_t> pos;
          const bool success{ !otos.getPosition(pos.value) };
          std::vector<uint8_t> posVector(pos.bytes, pos.bytes + sizeof(pos));
          Packet{ success ? Response::Success : Response::Error, posVector }.write();
        }
        break;
      case Command::GetVelocity:
        {
          PiecedData<sfe_otos_pose2d_t> pos;
          const bool success{ !otos.getVelocity(pos.value) };
          std::vector<uint8_t> posVector(pos.bytes, pos.bytes + sizeof(pos));
          Packet{ success ? Response::Success : Reponse::Error, posVector }.write();
        }
        break;
      case Command::Check:
        {
          sfe_otos_status_t status;
          bool success{ !otos.getStatus(status) };
          success = success && !status.errorLsm && !status.errorPaa;
          Packet{ success ? Response::Success : Response::Error }.write();
        }
        break;
      case Command::SelfTest:
        Packet{ otos.selfTest() >= 0 ? Response::Success : Response::Error }.write();
        break;
      default:
        Packet{ Response::Unknown }.write();
        break;
    }

    Serial.flush();
    // Clear input buffer.
    while (Serial.available()) {
      Serial.read();
    }
  }
}

int getExpectedBytes() {
  switch (Serial.peek()) {
    case Command::SetPosition: return 14; break;
    default: return 2; break;
  }
}
