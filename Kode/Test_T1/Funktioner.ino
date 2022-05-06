float hastighedHjul() {
  return ((analogRead(A5) - 2048.0) * (12000.0 / 4096.0)) * RPM2Rad;
}

void oprejsning() {
  if (ang < 0) {
    FPGA.analogWrite(45, map(55, 0, 100, pow(2, 10), 0));
    if (speedWheel >= 1750 * RPM2Rad) {
      FPGA.analogWrite(45, map(50, 0, 100, pow(2, 10), 0));
      brake.write(brk);
      vTaskDelay(300);
      brake.write(go);
      CubliState = Stabilizing;
    }
  } else {
    FPGA.analogWrite(45, map(45, 0, 100, pow(2, 10), 0));
    if (speedWheel <= -1400 * RPM2Rad) {
      FPGA.analogWrite(45, map(50, 0, 100, pow(2, 10), 0));
      brake.write(brk);
      vTaskDelay(300);
      brake.write(go);
      CubliState = Stabilizing;
    }
  }
}

float interpolate(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float complementaryFilter() {
  float tau = 0.5;
  float samplingperiod_s = samplingperiod / 1000.0;
  float c1 = samplingperiod_s / (samplingperiod_s + 2 * tau);

  speedFrame = (GYRO_Read(MPU_addr1) + GYRO_Read(MPU_addr2)) / 2.0;
  float acc_data = (ACC_Read(MPU_addr1) + ACC_Read(MPU_addr2)) / 2.0;

  float ang_filt = c1 * ( tau * speedFrame + acc_data) + ang_filt_old;
  ang_filt_old = c1 * ( tau * speedFrame + acc_data) - (samplingperiod_s - 2 * tau) / (samplingperiod_s + 2 * tau) * ang_filt;

  return ang_filt;
}

void start_ang() {
  float start_ang = (ACC_Read(MPU_addr1) + ACC_Read(MPU_addr2)) / 2.0; //Gyroskopet ved ikke hvor start position er, så brug accelerometer. ikke nødvendig, hvis accelerometer er præcis
  if (start_ang < 0) {
    ang = -0.77;
    //ang = (float)(analogRead(A1) - 1991.0) * (90.0 / 4096.0) * (M_PI / 180);
  }
  else {
    ang = 0.83;
    //ang = (float)(analogRead(A1) - 1991.0) * (90.0 / 4096.0) * (M_PI / 180);
  }
}

void adjust() {
  if (x < N) {//OBS her hvis det crasher <=
    adjustVal[x] = abs(speedWheel);
    adjustAverage += speedWheel;
    x++;
  }
  else {
    adjustAverage = adjustAverage / N;
    float absAdjustAverage = abs(adjustAverage);
    if (absAdjustAverage >= 50) {
      for (int k = 0; k < N; k++) {
        //if (abs(adjustVal[k]) < abs(adjustAverage) * 0.8 || abs(adjustVal[k]) > abs(adjustAverage) * 1.2 ) reject++;
        if (adjustVal[k] < absAdjustAverage - 20 || adjustVal[k] > absAdjustAverage + 20 ) reject++;
      }
    }
    else {
      for (int k = 0; k < N; k++) {
        //if (abs(adjustVal[k]) < abs(adjustAverage) * 0.8 || abs(adjustVal[k]) > abs(adjustAverage) * 1.2 ) reject++;
        if (adjustVal[k] < absAdjustAverage - 5 || adjustVal[k] > absAdjustAverage + 5) reject++;
      }
    }
    if (reject >= 10) {
      adjustAverage = 0;
      reject = 0;
      x = 0;
    }
    else {
      if (abs(adjustAverage) <= 100 * RPM2Rad ) { // small step, close to equilibrium
      if (adjustAverage < -step_size) angleRef = angleRef + step_size * absAdjustAverage;
      if (adjustAverage > +step_size) angleRef = angleRef - step_size * absAdjustAverage;
      }
      else if (abs(adjustAverage) > 100 * RPM2Rad) { // large step, far away from equilibrium
        if (adjustAverage < -step_size) angleRef = angleRef + large_step_size * absAdjustAverage;//left of eq
        if (adjustAverage > +step_size) angleRef = angleRef - large_step_size * absAdjustAverage;//right of eq
      }
      adjustAverage = 0;
      x = 0;
      reject = 0;
    }
  }
}
