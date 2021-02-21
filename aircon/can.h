#ifndef CAN_H_
#define CAN_H_

#include <carduinotest.h>
#include <FlexCAN_T4.h>

static SerialPacket canInitError(0x65, 0x30);

static SerialPacket canNotInitializedError(0x65, 0x31);
static SerialPacket canTransactionError(0x65, 0x32);

static SerialPacket canSendBufferFull(0x65, 0x33);
static SerialPacket canSendTimeout(0x65, 0x34);

static SerialPacket canControlError(0x65, 0x35);

struct CanData {
  union {
    unsigned char data[4];
    BitFieldMember<0, 32> canId;
  } metaData;
  uint8_t rxBuf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
};

class CanInput { 
  public:
    CanInput(uint32_t canId, uint8_t byteNumber, uint8_t bitMask) {
      this->canId       = canId;
      this->byteNumber  = byteNumber;
      this->bitMask     = bitMask;
    }
    virtual boolean getState() { 
      return this->state;
    }
    void updateCan(const CAN_message_t &message) {
      if (message.id == this->canId) {
        this->state = message.len >= this->byteNumber + 1 && (message.buf[this->byteNumber] & this->bitMask) == this->bitMask;
      }
    }
  private:
    uint32_t canId      = 0;
    uint8_t byteNumber  = 0;
    uint8_t bitMask     = B00000000;
    
    boolean state       = false;
};

class Can {
public:
    Can(Stream * serial) {
        this->sourceCan = new FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>();
        this->serial = serial;
    }
    ~Can() {
        //delete this->sourceCan;
    }
    boolean setup(uint32_t sourceBaudRate) {
        this->sourceCan->begin();
        this->sourceCan->setBaudRate(sourceBaudRate);
        
        this->isInitialized = true;
        return this->isInitialized;
    }


    void startSniffer() {
        this->isSniffing = true;
    }

    void stopSniffer() {
        this->isSniffing = false;
    }

    void update(void (*canCallback)(const CAN_message_t &message)) {
        if (!this->isInitialized) {
            canNotInitializedError.serialize(this->serial, 1000);
            return;
        }
        if (this->sourceCan->read(this->msg)) {
            if (this->isSniffing) {
                this->sniff(this->msg);
            }

            canCallback(this->msg);
        }
    }

    template<uint8_t BYTE_INDEX, uint8_t BIT_MASK, uint8_t COMPARE_VALUE>
    static bool readFlag(uint8_t * data) {
        return (data[BYTE_INDEX] & BIT_MASK) == COMPARE_VALUE;
    }
    template<uint8_t BYTE_INDEX, uint8_t BIT_MASK>
    static inline bool readFlag(uint8_t * data) {
        return Can::readFlag<BYTE_INDEX, BIT_MASK, BIT_MASK>(data);
    }

    void write(const CAN_message_t &message) {
        if (!this->isInitialized) {
            canNotInitializedError.serialize(this->serial, 1000);
            return;
        }

        this->sourceCan->write(message);
    }
private:
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> * sourceCan;
    CAN_message_t msg;

    Stream * serial;

    boolean isInitialized = false;
    boolean isSniffing = false;

    
    void sniff(const CAN_message_t &message) {
        uint32_t flippedCanId = htonl(message.id);
        this->serial->write("{");
        this->serial->write(0x62);
        this->serial->write(0x6d);
        this->serial->write(message.len + 0x04);
        this->serial->write((byte*)&flippedCanId, sizeof(message.id));
        for (uint8_t i = 0; i < message.len; i++) {
            this->serial->write(message.buf[i]);
        }
        this->serial->write("}");
    }
};

#endif /* CAN_H_ */
