#ifndef HVAC_H
#define HVAC_H

#define HVAC_AIRDUCT_WINDSHIELD 0xA8
#define HVAC_AIRDUCT_WINDSHIELD_FEET 0xA0
#define HVAC_AIRDUCT_FACE 0x88
#define HVAC_AIRDUCT_FACE_FEET 0x90
#define HVAC_AIRDUCT_FEET 0x98

#include <SPI.h>
#include <arduinoIO.h>

#include "bitfield.h"

union ClimateControl {
    unsigned char data[4];
    BitFieldMember<0, 1> isAcOn;
    BitFieldMember<1, 1> isAirductAuto;
    BitFieldMember<2, 1> isAirductWindshield;
    BitFieldMember<3, 1> isAirductFace;
    BitFieldMember<4, 1> isAirductFeet;
    BitFieldMember<5, 1> isWindshieldHeating;
    BitFieldMember<6, 1> isRearWindowHeating;
    BitFieldMember<7, 1> isRecirculation;
    BitFieldMember<8, 8> fanLevel;
    BitFieldMember<16, 8> desiredTemperature;
    BitFieldMember<24, 1> isAutoFan;
};

union CanData1 {
    unsigned char data[8];
    BitFieldMember<0, 32> padding1;
    BitFieldMember<32, 8> desiredTemperature;
    BitFieldMember<40, 16> padding2;
    BitFieldMember<56, 8> roll;
};

union CanData2 {
    unsigned char data[8];
    BitFieldMember<0, 14> padding1;
    BitFieldMember<14, 1> isAutoFan;
    BitFieldMember<15, 1> padding2;
    BitFieldMember<16, 8> airductSetting;
    BitFieldMember<24, 1> isWindshieldHeating;
    BitFieldMember<25, 6> padding3;
    BitFieldMember<31, 1> isRecirculation;
    BitFieldMember<32, 8> fanLevel;
    BitFieldMember<40, 16> padding4;
    BitFieldMember<56, 8> roll;
};

union CanData3 {
  unsigned char data[8];
  BitFieldMember<0, 16> padding1;
  BitFieldMember<16, 1> isAcOn;
  BitFieldMember<17, 47> padding2;
};

union CanData4 {
  unsigned char data[8];
  BitFieldMember<0, 7> padding1;
  BitFieldMember<7, 1> isRearWindowHeating;
  BitFieldMember<8, 56> padding2;
};

union ButtonStates {
    unsigned char data[2];
    BitFieldMember<0, 1> offButton;
    BitFieldMember<1, 1> defrostButton;
    BitFieldMember<2, 1> rearHeaterButton;
    BitFieldMember<3, 1> recirculationButton;
    BitFieldMember<4, 1> modeButton;
    BitFieldMember<5, 1> autoButton;
    BitFieldMember<6, 1> acButton;
    BitFieldMember<7, 1> fanKnob;
    BitFieldMember<8, 1> tempKnob;
    BitFieldMember<9, 7> padding;
};

class Hvac {
   public:
    Hvac(Can *can) {
        this->climateControl->payload()->isAcOn = false;
        this->climateControl->payload()->isAutoFan = false;
        this->climateControl->payload()->isAirductWindshield = false;
        this->climateControl->payload()->isAirductFace = false;
        this->climateControl->payload()->isAirductFeet = false;
        this->climateControl->payload()->isWindshieldHeating = false;
        this->climateControl->payload()->isRearWindowHeating = false;
        this->climateControl->payload()->isRecirculation = false;
        this->climateControl->payload()->fanLevel = 0;
        this->climateControl->payload()->desiredTemperature = 21 * 2;
        this->climateControl->payload()->isAirductAuto = false;

        this->canData1.desiredTemperature = 21 * 2;
        this->canData3.isAcOn = false;
        this->canData2.isAutoFan = false;
        this->canData2.airductSetting = HVAC_AIRDUCT_FEET;

        this->outputMessage1.id = 0x54A;
        this->outputMessage2.id = 0x54B;
        this->outputMessage3.id = 0x54C;
        this->outputMessage4.id = 0x625;

        this->can = can;
        SPI.begin();

        this->setAirduct(1);
        this->setFanLevel(2);
        this->setTemperature(18);
    }
    void update() {
        this->rearHeaterButton->update();
        this->recirculationButton->update();
        this->airConditionButton->update();

        if (millis() - this->lastUpdate > UpdateRate) {
            this->climateControl->payload()->isRecirculation =
                this->recirculationLed->getState();
            this->climateControl->payload()->isAcOn =
                this->airConditionLed->getState();
            this->climateControl->payload()->isRearWindowHeating =
                this->rearHeaterLed->getState();

            this->canData2.isRecirculation = this->climateControl->payload()->isRecirculation;
            this->canData3.isAcOn = this->climateControl->payload()->isAcOn;
            this->canData4.isRearWindowHeating = this->climateControl->payload()->isRearWindowHeating;

            this->writeOutput(outputMessage1, canData1.data);           
            this->writeOutput(outputMessage2, canData2.data);
            this->writeOutput(outputMessage3, canData3.data);
            this->writeOutput(outputMessage4, canData4.data);

            this->lastUpdate = millis();
        }
    }
    void toggleAirCondition() {
        this->airConditionButton->set(HIGH, ButtonPressDuration);
    }
    void toggleRearHeater() {
        this->rearHeaterButton->set(HIGH, ButtonPressDuration);
    }
    void toggleRecirculation() {
        this->recirculationButton->set(HIGH, ButtonPressDuration);
    }
    void toggleAutomaticFan() {
        this->setAutomaticFan(!this->climateControl->payload()->isAutoFan);
    }
    void toggleAutomaticDuct() {
        this->setAutomaticDuct(!this->climateControl->payload()->isAirductAuto);
    }

    void setAutomaticFan(boolean state) {
        if (state) {
            this->setFanLevel(1);
            // this->setAirduct(0);
        } else {
            this->setFanLevel(this->manualFanSetting);
            // this->setAirduct(this->manualAirductSetting);
        }
        this->climateControl->payload()->isAutoFan = state;
        this->canData2.isAutoFan = true;
    }

    void setAutomaticDuct(boolean state) {
        if (state) {
            // this->setFanLevel(1);
            this->setAirduct(0);
        } else {
            // this->setFanLevel(this->manualFanSetting);
            this->setAirduct(this->manualAirductSetting);
        }
        this->climateControl->payload()->isAirductAuto = state;
        
    }
    void toggleDefrost() {
        setDefrost(this->airductSetting != AirductModeCount - 1);
    }
    void setDefrost(boolean state) {
        if (state) {
            this->setAirduct(AirductModeCount - 1);
        } else {
            this->setAirduct(this->manualAirductSetting);
        }
    }
    void toggleAirduct() {
        uint8_t newMode = this->airductSetting + 1;
        if (newMode >= AirductModeCount - 1) {
            newMode = 1;
        }

        this->setAirduct(newMode);
    }
    void setAirduct(uint8_t mode) {
        if (mode > AirductModeCount - 1) {
            mode = AirductModeCount - 1;
        }

        this->airductSetting = mode;
        this->setDial(this->airductSelect, AirductModes[this->airductSetting]);

        if (this->airductSetting != 0 &&
            this->airductSetting != AirductModeCount - 1) {
            this->manualAirductSetting = this->airductSetting;
        }

        this->climateControl->payload()->isAirductWindshield = false;
        this->climateControl->payload()->isAirductFace = false;
        this->climateControl->payload()->isAirductFeet = false;
        this->climateControl->payload()->isWindshieldHeating = false;

        switch (this->airductSetting) {
            case 0:
                // INTENTIONALLY LEFT BLANK
                break;
            case 1:
                this->climateControl->payload()->isAirductFace = true;
                break;
            case 2:
                this->climateControl->payload()->isAirductFace = true;
                this->climateControl->payload()->isAirductFeet = true;
                break;
            case 3:
                this->climateControl->payload()->isAirductFeet = true;
                break;
            case 4:
                this->climateControl->payload()->isAirductWindshield = true;
                this->climateControl->payload()->isAirductFeet = true;
                break;
            case 5:
                this->climateControl->payload()->isAirductWindshield = true;
                this->climateControl->payload()->isWindshieldHeating = true;
                break;
        }
    }
    void setTemperature(float temperature) {
        if (temperature > TemperatureMaxLevel) {
            temperature = TemperatureMaxLevel;
        }
        if (temperature < TemperatureMinLevel) {
            temperature = TemperatureMinLevel;
        }

        float voltage = (TemperatureMaxVoltage - TemperatureMinVoltage) -
                        ((temperature - TemperatureMinLevel) /
                         (float)(TemperatureMaxLevel - TemperatureMinLevel) *
                         (TemperatureMaxVoltage - TemperatureMinVoltage)) +
                        TemperatureMinVoltage;

        this->climateControl->payload()->desiredTemperature =
            (uint8_t)(temperature * 2);
        this->setDial(this->temperatureSelect, voltage);
    }
    void setFanLevel(uint8_t value) {
        if (value > DialStepsFan) {
            value = DialStepsFan;
        }
        if (value < 0) {
            value = 0;
        }

        float voltage = FanModes[0];

        if (value > 1) {
            voltage = FanModes[2];
            voltage = voltage - (voltage - FanMinVoltage) /
                                    (float)(FanMaxLevel - 1) * (value - 2);
        } else {
            voltage = FanModes[value];
        }

        if (value != 1) {
            this->manualFanSetting = value;
        }

        this->climateControl->payload()->fanLevel =
            value == 0 ? 0 : (value - 1 >= 1 ? value - 1 : 1);
        this->setDial(this->fanSelect, voltage);
    }
    void write(uint8_t buttonId, BinaryBuffer *payloadBuffer) {
        switch (buttonId) {
            case 0x01:  // OFF BUTTON
                if (this->climateControl->payload()->isAutoFan) {
                    this->setAutomaticFan(false);
                }
                this->setFanLevel(0);
                break;
            case 0x02:  // ac button
                this->toggleAirCondition();
                break;
            case 0x03:  // auto button
                this->toggleAutomaticDuct();
                break;
            case 0x04:  // recirculation button
                this->toggleRecirculation();
                break;
            case 0x05:  // windshield heating button
                if (this->climateControl->payload()->isAirductAuto) {
                    this->setAutomaticDuct(false);
                }
                this->toggleDefrost();
                break;
            case 0x06:  // rear window heating button
                this->toggleRearHeater();
                break;
            case 0x07:  // mode button
                if (this->climateControl->payload()->isAirductAuto) {
                    this->setAutomaticDuct(false);
                }
                this->toggleAirduct();
                break;
            case 0x08:  // temperature knob
                if (payloadBuffer->available() > 0) {
                    this->setTemperature(payloadBuffer->readByte().data /
                                         (float)2);
                }
                break;
            case 0x09:  // fan level
                if (payloadBuffer->available() > 0) {
                    if (this->climateControl->payload()->isAutoFan) {
                        this->setAutomaticFan(false);
                    }
                    this->setFanLevel(payloadBuffer->readByte().data + 1);
                }
                break;
            case 0x10:  // auto button
                this->toggleAutomaticFan();
                break;
        }
    }
    void receiveCan(const CAN_message_t &message) {
        if (message.id == 0x540) {
            // Nothing to do here
        } else if (message.id == 0x541) {
            this->buttonStates.modeButton = (message.buf[0] & B00010000) == B00010000;
            this->buttonStates.defrostButton = (message.buf[0] & B00000001) == B00000001;
            this->buttonStates.rearHeaterButton = (message.buf[1] & B10000000) == B10000000;
            this->buttonStates.recirculationButton = (message.buf[1] & B00000010) == B00000010;
            this->buttonStates.tempKnob = (message.buf[2] & B10000000) == B10000000;
            this->buttonStates.acButton = (message.buf[2] & B00000100) == B00000100;
            this->buttonStates.offButton = (message.buf[2] & B00000001) == B00000001;
            this->buttonStates.autoButton = (message.buf[3] & B10000000) == B10000000;
            this->buttonStates.fanKnob = (message.buf[3] & B00010000) == B00010000;

            if (this->buttonStates.modeButton != this->previousButtonStates.modeButton) {
                if (this->climateControl->payload()->isAirductAuto) {
                    this->setAutomaticDuct(false);
                }
                this->toggleAirduct();
            }
            if (this->buttonStates.defrostButton != this->previousButtonStates.defrostButton) {
                if (this->climateControl->payload()->isAirductAuto) {
                    this->setAutomaticDuct(false);
                }
                this->toggleDefrost();
            }
            if (this->buttonStates.rearHeaterButton != this->previousButtonStates.rearHeaterButton && this->buttonStates.rearHeaterButton) {
                this->toggleRearHeater();
            }
            if (this->buttonStates.recirculationButton != this->previousButtonStates.recirculationButton) {
                this->toggleRecirculation();
            }
            if (this->buttonStates.acButton != this->previousButtonStates.acButton) {
                this->toggleAirCondition();
            }
            if (this->buttonStates.offButton != this->previousButtonStates.offButton) {
                if (this->climateControl->payload()->isAutoFan) {
                    this->setAutomaticFan(false);
                }
                this->setFanLevel(0);
            }
            if (this->buttonStates.autoButton != this->previousButtonStates.autoButton) {
                this->toggleAutomaticDuct();
            }
            if (this->buttonStates.fanKnob != this->previousButtonStates.fanKnob) {
                updateFan = true;
            }
            if (this->buttonStates.tempKnob != this->previousButtonStates.tempKnob) {
                updateTemp = true;
            }

            this->previousButtonStates.data[0] = this->buttonStates.data[0];
            this->previousButtonStates.data[1] = this->buttonStates.data[1];
        } else if (message.id == 0x542) {
            if (updateFan) {
                this->setFanLevel(message.buf[0] / 8);
                updateFan = false;
            }
            if (updateTemp) {
                this->setTemperature((float)message.buf[1] / (float)2);
            }
        }
    }

   private:
    void writeOutput(CAN_message_t & output, unsigned char data[8]) {
      output.id = 0x54A;
      for (int i = 0; i < 8; i++) {
        output.buf[i] = data[i];
      }
      this->can->write(output);
    }
    void setDial(DigitalOutput *select, float voltage) {
        select->activate();
        SPI.transfer(0);
        SPI.transfer((int)roundf(voltage / 5.00 * 256 - 1));
        select->deactivate();
    }

    SerialDataPacket<ClimateControl> *climateControl =
        new SerialDataPacket<ClimateControl>(0x73, 0x63);
    Can *can;

    uint8_t airductSetting = 0;
    uint8_t manualAirductSetting = 0;
    uint8_t manualFanSetting = 0;

    unsigned long lastUpdate = 0;
    /*
     * In a real-world application these pointers should be
     * initialized in constructor deleted in destructor.
     * Since this class will only be created once, we do not care. :-P
     */
    DigitalOutput *airductSelect = new DigitalOutput(10, LOW);
    DigitalOutput *temperatureSelect = new DigitalOutput(14, LOW);
    DigitalOutput *fanSelect = new DigitalOutput(30, LOW);
    TimedOutput *rearHeaterButton = new TimedOutput(new DigitalOutput(33));
    TimedOutput *recirculationButton = new TimedOutput(new DigitalOutput(26));
    TimedOutput *airConditionButton = new TimedOutput(new DigitalOutput(25));

    DigitalInput *rearHeaterLed = new DigitalInput(32, 20, LOW, INPUT);
    DigitalInput *freshAirLed = new DigitalInput(31, 20, LOW, INPUT);
    DigitalInput *recirculationLed = new DigitalInput(29, 20, LOW, INPUT);
    DigitalInput *airConditionLed = new DigitalInput(47, 20, LOW, INPUT);

    CanData1 canData1;
    CanData2 canData2;
    CanData3 canData3;
    CanData4 canData4;
    CAN_message_t outputMessage1;
    CAN_message_t outputMessage2;
    CAN_message_t outputMessage3;
    CAN_message_t outputMessage4;
    ButtonStates buttonStates;
    ButtonStates previousButtonStates;

    bool updateFan = false;
    bool updateTemp = false;

    const unsigned int ButtonPressDuration = 300;

    /* 18 - 32 C */
    const uint8_t DialStepsTemperature = 31;
    const uint8_t TemperatureMinLevel = 18;
    const uint8_t TemperatureMaxLevel = 32;
    const float TemperatureMinVoltage = 0.25;
    const float TemperatureMaxVoltage = 4.94;

    /* OFF, AUTO, RANGE */
    static const uint8_t DialStepsFan = 27;
    static const uint8_t FanMaxLevel = 25;
    const float FanMinVoltage = 0.24;
    static const uint8_t FanModeCount = 3;
    const float FanModes[FanModeCount] = {4.95, 4.77, 4.66};

    /* AUTO, FACE, FACE & FEET, FEET, WINDOW & FEET, DEFROST */
    static const uint8_t AirductModeCount = 6;
    const float AirductModes[AirductModeCount] = {4.94, 4.34, 2.92,
                                                  2.48, 1.38, 0.25};

    const unsigned int UpdateRate = 250;
};

#endif