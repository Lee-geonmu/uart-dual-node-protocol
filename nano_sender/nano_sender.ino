#include <SoftwareSerial.h>
//---------------- 핀 ----------------
//하드웨어 UART(D0/D1)는 USB 디버깅용으로 비워두고
//노드 간 링크는 SoftwareSerial(D2/D3)로 분리
const uint8_t PIN_LINK_RX = 2; //<- Uno D9(TX)
const uint8_t PIN_LINK_TX = 3; //-> Uno D8(RX)
const uint8_t PIN_SENSOR  = A0; //가변저항(임의 아날로그 데이터 소스)
const uint8_t PIN_LED_OK  = 6; //초록: 전송 성공
const uint8_t PIN_LED_ERR = 5; //빨강: 재시도 초과 실패
SoftwareSerial link(PIN_LINK_RX, PIN_LINK_TX);
//---------------- 프로토콜 ----------------
//프레임: [START][LEN][TYPE][SEQ][PAYLOAD...][CRC][END]
//LEN : TYPE(1) + SEQ(1) + PAYLOAD(N)
//CRC : LEN부터 PAYLOAD 끝까지 CRC-8 (poly 0x07)
//SEQ : 프레임마다 증가. 재전송 시에는 같은 값 유지 -> 수신 측 중복 판별 근거
const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE   = 0x55;
const uint8_t TYPE_DATA  = 0x01;
const uint8_t TYPE_ACK   = 0x02;
const uint8_t TYPE_NACK  = 0x03;
const uint16_t SEND_INTERVAL_MS = 200; //전송 주기
const uint16_t ACK_TIMEOUT_MS   = 300; //응답 대기 한도
const uint8_t  MAX_RETRY        = 3; //프레임당 최대 전송 횟수
uint8_t txSeq = 0; //uint8이라 255 다음 0으로 자연 순환
unsigned long lastSendTime = 0;
//CRC-8 (poly 0x07, init 0x00)
//프레임이 짧아서 테이블 없이 비트 단위로 계산
uint8_t crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else            crc <<= 1;
    }
  }
  return crc;
}
void sendFrame(uint8_t type, uint8_t seq, uint8_t* payload, uint8_t payloadLen) {
  uint8_t len = payloadLen + 2;  // TYPE + SEQ 포함
  //CRC 계산 구간: LEN, TYPE, SEQ, PAYLOAD
  uint8_t buf[20];
  uint8_t idx = 0;
  buf[idx++] = len;
  buf[idx++] = type;
  buf[idx++] = seq;
  for (uint8_t i = 0; i < payloadLen; i++) buf[idx++] = payload[i];
  uint8_t crc = crc8(buf, idx);
  link.write(START_BYTE);
  link.write(len);
  link.write(type);
  link.write(seq);
  for (uint8_t i = 0; i < payloadLen; i++) link.write(payload[i]);
  link.write(crc);
  link.write(END_BYTE);
}
//ACK/NACK 응답 대기. 1 = ACK, 0 = NACK, -1 = 타임아웃
//응답 프레임은 payload가 없는 고정 길이라 단순 순차 파싱으로 처리
int waitForResponse(uint8_t expectedSeq, uint16_t timeoutMs) {
  unsigned long start = millis();
  uint8_t state = 0;
  uint8_t len = 0, type = 0, seq = 0, crc = 0;
  while (millis() - start < timeoutMs) {
    if (!link.available()) continue;
    uint8_t b = link.read();
    switch (state) {
      case 0: if (b == START_BYTE) state = 1; break;
      case 1: len = b;  state = 2; break;
      case 2: type = b; state = 3; break;
      case 3: seq = b;  state = 4; break;
      case 4: crc = b;  state = 5; break;
      case 5:
        if (b == END_BYTE) {
          uint8_t buf[3] = { len, type, seq };
          if (crc8(buf, 3) == crc && seq == expectedSeq) {
            if (type == TYPE_ACK)  return 1;
            if (type == TYPE_NACK) return 0;
          }
        }
        state = 0; //잘못된 프레임이면 처음부터 다시
        break;
    }
  }
  return -1;
}
void setup() {
  Serial.begin(9600); //USB 디버깅
  link.begin(9600); //노드 간 링크
  pinMode(PIN_LED_OK, OUTPUT);
  pinMode(PIN_LED_ERR, OUTPUT);
  Serial.println(F("Nano sender node ready"));
}
void loop() {
  if (millis() - lastSendTime < SEND_INTERVAL_MS) return;
  lastSendTime = millis();
  uint16_t value = analogRead(PIN_SENSOR);
  uint8_t payload[2] = { (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
  bool success = false;
  for (uint8_t attempt = 0; attempt < MAX_RETRY; attempt++) {
    sendFrame(TYPE_DATA, txSeq, payload, 2);
    int result = waitForResponse(txSeq, ACK_TIMEOUT_MS);
    if (result == 1) {
      success = true;
      break;
    }
    Serial.print(F("Seq "));
    Serial.print(txSeq);
    Serial.print(F(" attempt "));
    Serial.print(attempt + 1);
    Serial.println(result == 0 ? F(": NACK, retrying") : F(": timeout, retrying"));
  }
  if (success) {
    digitalWrite(PIN_LED_OK, HIGH);
    digitalWrite(PIN_LED_ERR, LOW);
    Serial.print(F("Sent seq="));
    Serial.print(txSeq);
    Serial.print(F(" value="));
    Serial.println(value);
  } else {
    digitalWrite(PIN_LED_OK, LOW);
    digitalWrite(PIN_LED_ERR, HIGH);
    Serial.print(F("Seq "));
    Serial.print(txSeq);
    Serial.println(F(" FAILED after max retries"));
  }
  //실패해도 SEQ는 증가시킨다.
  //센서값은 계속 갱신되므로 오래된 값을 재전송하는 것보다
  //다음 주기에 최신 값을 보내는 쪽을 선택
  txSeq++;
}
