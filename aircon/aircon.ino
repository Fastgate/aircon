#include "can.h"
#include "hvac.h"

void onCarduinoSerialTimeout();
void onCarduinoSerialEvent(uint8_t type, uint8_t id, BinaryBuffer *payloadBuffer);
void canCallback(const CAN_message_t &message);

Can can(&Serial);
Carduino carduino(&Serial, onCarduinoSerialEvent, onCarduinoSerialTimeout);
Hvac hvac(&can);

void setup() {
    can.setup(500000);
}

void loop() {
    can.update(canCallback);
    hvac.update();
}

void canCallback(const CAN_message_t &message) {
    hvac.receiveCan(message);
}

void onCarduinoSerialTimeout() {
    // NOTHING TO DO HERE
}

void onCarduinoSerialEvent(uint8_t type, uint8_t id, BinaryBuffer *payloadBuffer) {
    // NOTHING TO DO HERE
}