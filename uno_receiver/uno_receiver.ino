#include <SoftwareSerial.h>
//---------------- 핀 ----------------
const uint8_t PIN_LINK_RX = 8; //<- Nano D3(TX)
const uint8_t PIN_LINK_TX = 9; //-> Nano D2(RX)
const uint8_t PIN_LED_OK  = 6; //초록: 통신 정상
const uint8_t PIN_LED_ERR = 5; //빨강: 통신 두절
SoftwareSerial link(PIN_LINK_RX, PIN_LINK_TX);
//---------------- 프로토콜 ----------------
//프레임 구조는 송신 측(nano_sender)과 동일
const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE   = 0x55;
const uint8_t TYPE_DATA  = 0x01;
const uint8_t TYPE_ACK   = 0x02;
const uint8_t TYPE_NACK  = 0x03;
const unsigned long COMM_LOST_TIMEOUT_MS = 2000; //이 시간 동안 유효 프레임 없으면 두절 판정
unsigned long lastValidFrameTime = 0;
bool commLost = false;
//중복 판별용: 마지막으로 실제 처리한 SEQ
bool    hasLastSeq = false;
uint8_t lastAcceptedSeq = 0;
//시연용 스위치. FORCE_LOSS_SEQ에서 ACK를 한 번 생략해서
//송신 재전송 -> 수신 중복 판별 흐름을 인위적으로 재현한다.
//ACK 유실이 자연적으로는 드물어서 넣어둔 것. 평소에는 false로 꺼둔다.
const bool    DEBUG_FORCE_ACK_LOSS = false;
const uint8_t FORCE_LOSS_SEQ       = 5;
bool forcedLossDone = false;
// ---------------- 수신 파서 ----------------
enum RxState : uint8_t {
  WAIT_START,
  READ_LEN,
  READ_TYPE,
  READ_SEQ,
  READ_PAYLOAD,
  READ_CRC,
  READ_END
};
RxState rxState = WAIT_START;
uint8_t frameLen, frameType, frameSeq, frameCrc;
uint8_t framePayload[16];
uint8_t payloadIdx = 0;
uint8_t crc8(const uint8_t* data, uint8_t length) {
  uint8_t c = 0x00;
  for (uint8_t i = 0; i < length; i++) {
    c ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (c & 0x80) c = (c << 1) ^ 0x07;
      else c <<= 1;
    }
  }
  return c;
}
void sendResponse(uint8_t respType, uint8_t respSeq) {
  uint8_t len = 2; //TYPE + SEQ, payload 없음
  uint8_t buf[3] = { len, respType, respSeq };
  uint8_t c = crc8(buf, 3);
  link.write(START_BYTE);
  link.write(len);
  link.write(respType);
  link.write(respSeq);
  link.write(c);
  link.write(END_BYTE);
}
void resetParser() {
  rxState = WAIT_START;
  payloadIdx = 0;
}
void handleValidFrame() {
  if (frameType != TYPE_DATA) return; //수신 노드는 DATA만 처리
  lastValidFrameTime = millis();
  if (commLost) {
    commLost = false;
    Serial.println(F("Comm restored"));
  }
  //직전에 처리한 SEQ와 같으면 ACK 유실로 인한 재전송으로 판단.
  //값은 다시 반영하지 않고 ACK만 재전송한다.
  if (hasLastSeq && frameSeq == lastAcceptedSeq) {
    sendResponse(TYPE_ACK, frameSeq);
    Serial.print(F("Duplicate seq="));
    Serial.print(frameSeq);
    Serial.println(F(", re-ACK only"));
    return;
  }
  uint16_t value = framePayload[0] | (framePayload[1] << 8);
  lastAcceptedSeq = frameSeq;
  hasLastSeq = true;
  if (DEBUG_FORCE_ACK_LOSS && frameSeq == FORCE_LOSS_SEQ && !forcedLossDone) {
    forcedLossDone = true;
    Serial.println(F(">>> DEBUG: intentionally dropping ACK <<<"));
    digitalWrite(PIN_LED_OK, HIGH);
    digitalWrite(PIN_LED_ERR, LOW);
    Serial.print(F("Received seq="));
    Serial.print(frameSeq);
    Serial.print(F(" value="));
    Serial.println(value);
    return; //ACK 생략 -> 송신 측 타임아웃 -> 재전송 유도
  }
  sendResponse(TYPE_ACK, frameSeq);
  digitalWrite(PIN_LED_OK, HIGH);
  digitalWrite(PIN_LED_ERR, LOW);
  Serial.print(F("Received seq="));
  Serial.print(frameSeq);
  Serial.print(F(" value="));
  Serial.println(value);
}
void setup() {
  Serial.begin(9600);
  link.begin(9600);
  pinMode(PIN_LED_OK, OUTPUT);
  pinMode(PIN_LED_ERR, OUTPUT);
  lastValidFrameTime = millis();
  Serial.println(F("Uno receiver node ready"));
}
void loop() {
  while (link.available()) {
    uint8_t b = link.read();
    switch (rxState) {
      case WAIT_START:
        if (b == START_BYTE) { payloadIdx = 0; rxState = READ_LEN; }
        break;
      case READ_LEN:
        frameLen = b;
        rxState = READ_TYPE;
        break;
      case READ_TYPE:
        frameType = b;
        rxState = READ_SEQ;
        break;
      case READ_SEQ:
        frameSeq = b;
        //LEN이 손상돼 버퍼를 넘는 값이면 프레임 폐기
        if (frameLen < 2 || (uint8_t)(frameLen - 2) > sizeof(framePayload)) {
          resetParser();
        } else if (frameLen == 2) {
          rxState = READ_CRC;      // payload 없는 프레임 (ACK/NACK류)
        } else {
          rxState = READ_PAYLOAD;
        }
        break;
      case READ_PAYLOAD:
        framePayload[payloadIdx++] = b;
        if (payloadIdx >= (uint8_t)(frameLen - 2)) rxState = READ_CRC;
        break;
      case READ_CRC:
        frameCrc = b;
        rxState = READ_END;
        break;
      case READ_END:
        if (b == END_BYTE) {
          //CRC 검증 구간: LEN, TYPE, SEQ, PAYLOAD
          uint8_t buf[20];
          uint8_t idx = 0;
          buf[idx++] = frameLen;
          buf[idx++] = frameType;
          buf[idx++] = frameSeq;
          for (uint8_t i = 0; i < payloadIdx; i++) buf[idx++] = framePayload[i];
          if (crc8(buf, idx) == frameCrc) {
            handleValidFrame();
          } else {
            sendResponse(TYPE_NACK, frameSeq);
            Serial.print(F("CRC fail seq="));
            Serial.print(frameSeq);
            Serial.println(F(" -> NACK sent"));
          }
        }
        resetParser();
        break;
    }
  }
  //수신 측 자체 감시: 일정 시간 유효 프레임이 없으면 두절 표시
  if (!commLost && millis() - lastValidFrameTime > COMM_LOST_TIMEOUT_MS) {
    commLost = true;
    digitalWrite(PIN_LED_OK, LOW);
    digitalWrite(PIN_LED_ERR, HIGH);
    Serial.println(F("Comm lost: no valid frame in 2s"));
  }
}
