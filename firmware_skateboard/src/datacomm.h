// #include "variable.h"

union ByteULLUnion {
  uint8_t bytes[8];
  unsigned long long time_bytes;
};

union ByteInt {
  uint8_t bytes[2];
  unsigned short time_val;
};

union ByteULong {
  uint8_t bytes[4];
  unsigned long mil_val;
};

union ByteEncLong {
  uint8_t bytes[4];
  long enc;
};

union ByteUI64 {
  uint8_t bytes[8];
  uint64_t time_byte;
};

union ByteChar {
  uint8_t bytes[8];
  char cr;
};

void writeSensorStream()
{
  byte header[] = {0xFF, 0xFF, 0x00};
  byte chksum = 0xFE;
  byte _temp;


  //Out data buffer
  outPayload.newPacket();

  outPayload.add( accelx1);
  outPayload.add( accely1);
  outPayload.add( accelz1);
  outPayload.add( gyrox1);

  outPayload.add( gyroy1);
  outPayload.add( gyroz1);

  //  send packet
  header[2] = outPayload.sz() * 4 + 1 + 4 * 4 + 8 + 4 + 1;
  chksum += header[2];

  // Send header
  Serial1.write(header[0]);
  Serial1.write(header[1]);
  Serial1.write(header[2]);

  

  // Sending encoder stream

  // front right
  ByteEncLong _val_fr;
  _val_fr.enc = p_fr;

  for (int i = 0; i < 4; i++) {
    _temp = _val_fr.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }

  // front left
  ByteEncLong _val_fl;
  _val_fl.enc = p_fl;

  for (int i = 0; i < 4; i++) {
    _temp = _val_fl.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }
  //  Serial.println(p_rl);

  // rear right
  ByteEncLong _val_rr;
  _val_rr.enc = p_rr;

  for (int i = 0; i < 4; i++) {
    _temp = _val_rr.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }

  // rear left
  ByteEncLong _val_rl;
  _val_rl.enc = p_rl;

  for (int i = 0; i < 4; i++) {
    _temp = _val_rl.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }


  ByteUI64 _tt;
  _tt.time_byte = _rawTime;
  for (int i = 0; i < 8; i++) {
    _temp = _tt.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }

  //milli seconds
  ByteULong mils_s;
  mils_s.mil_val = _mils;

  for (int i = 0; i < 4; i++) {
    _temp = mils_s.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }

  //sync pulse
  ByteChar _chr;
  _chr.cr = _sync;

  _temp = _chr.bytes[0];
  Serial1.write(_temp);
  chksum += _temp;
  
  //Send checksum
  Serial1.write(chksum);
  // Serial.print(" sent ");

}


void writeEncoderStream()
{
  byte header[] = {0xFF, 0xFF, 0x00};
  byte chksum = 0xFE;
  byte _temp;

  //Out data buffer
  outPayload.newPacket();

  outPayload.add( accelx1);
  outPayload.add( accely1);
  outPayload.add( accelz1);
  outPayload.add( gyrox1);

  outPayload.add( gyroy1);
  outPayload.add( gyroz1);

  //  send packet
  header[2] = outPayload.sz() * 4 + 4 * 4 + 1 + 8 + 4 + 1;  // encoder stream + 1 + rtc value + mills
  chksum += header[2];

  // Send header
  Serial1.write(header[0]);
  Serial1.write(header[1]);
  Serial1.write(header[2]);

  // front right
  ByteEncLong _val_fr;
  _val_fr.enc = p_fr;

  for (int i = 0; i < 4; i++) {
    _temp = _val_fr.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }

  // front left
  ByteEncLong _val_fl;
  _val_fl.enc = p_fl;

  for (int i = 0; i < 4; i++) {
    _temp = _val_fl.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }
  //  Serial.println(p_rl);

  // rear right
  ByteEncLong _val_rr;
  _val_rr.enc = p_rr;

  for (int i = 0; i < 4; i++) {
    _temp = _val_rr.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }

  // rear left
  ByteEncLong _val_rl;
  _val_rl.enc = p_rl;

  for (int i = 0; i < 4; i++) {
    _temp = _val_rl.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }


  ByteUI64 _tt;
  _tt.time_byte = _rawTime;
  for (int i = 0; i < 8; i++) {
    _temp = _tt.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }

  //milli seconds
  ByteULong mils_s;
  mils_s.mil_val = _mils;

  for (int i = 0; i < 4; i++) {
    _temp = mils_s.bytes[i];
    Serial1.write(_temp);
    chksum += _temp;
  }

  //sync pulse
  ByteChar _chr;
  _chr.cr = _sync;

  _temp = _chr.bytes[0];
  Serial1.write(_temp);
  chksum += _temp;
  Serial.print(_temp);

  // sending outdata buffer

  // Send payload
  for (int i = 0; i < outPayload.sz() * 4 ; i++) {
    _temp = outPayload.getByte(i);
    Serial1.write(_temp);
    chksum += _temp;
  }

  // Serial.print(" sent ");

  //Send checksum
  Serial1.write(chksum);

}
