void IMU_Setup(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission();
  //Gyro config
  Wire.beginTransmission(address);        // Contact IMU for setup
  Wire.write(0x1B);                       // GYRO_CONFIG register
  Wire.write(0b00001000);                 // Register bits set to b'00010000 (500dps full scale)
  Wire.endTransmission();                 // End transmission for gyro
  //Acc config
  Wire.beginTransmission(address);        // Contact IMU for setup
  Wire.write(0x1C);                       // ACCEL_CONFIG register
  Wire.write(0b00000000);                 // Setting the accel to +/- 2g
  Wire.endTransmission();
}

float GYRO_Read(int address) {
  int16_t GyZ;
  float GyrZ;

  //Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0x47);                       //Specify address for gyroscope (0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L))
  Wire.endTransmission();
  Wire.requestFrom(address, 2);           // request a total of 2 registers
  if (Wire.available())  GyZ = Wire.read() << 8 | Wire.read();
  //if (!Wire.available()) Wire.end();

  GyrZ = (GyZ / scale) * M_PI / 180;
  return GyrZ;
}

float ACC_Read(int address) {
  int16_t AcX, AcY;
  float AccX, AccY;
  float Accang;

  //Wire.begin();
  Wire.beginTransmission(address);  //I2C address of the MPU
  Wire.write(0x3B);                 //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(address, 4);     //Request 4 Accel Registers

  if (Wire.available()) {
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
  }
  //if (!Wire.available()) Wire.end();

  AccX = AcX / scaleacc;
  AccY = AcY / scaleacc;

  if (address == MPU_addr1) {
    Accang = (atan2(AccX, AccY) - M_PI / 4);
  }
  else if (address == MPU_addr2) {
    Accang = (atan2(AccY, - AccX) - M_PI / 4);
  }
  return Accang;
}
