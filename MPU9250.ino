#include <Wire.h>

#define MPU9250_ADDRESS           0x68
#define MAG_ADDRESS               0x0C
 
#define ACC_FULL_SCALE_2_G        0x00  
#define ACC_FULL_SCALE_4_G        0x08
#define ACC_FULL_SCALE_8_G        0x10
#define ACC_FULL_SCALE_16_G       0x18

#define GYRO_FULL_SCALE_250_DPS   0x00  
#define GYRO_FULL_SCALE_500_DPS   0x08
#define GYRO_FULL_SCALE_1000_DPS  0x10
#define GYRO_FULL_SCALE_2000_DPS  0x18

#define MAG_SINGLE_MODE           0x01
#define MAG_CONT_MODE_1           0x02
#define MAG_CONT_MODE_2           0x06
#define MAG_EXT_TRIG_MODE         0x04
#define  MAG_SELF_TEST_MODE       0x08
 
String packet = "";
double acc[3];
int gyr[3];
int mag[3];

void setup(){
  Wire.begin(); //SDA,SCL
  Serial.begin(115200);

  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);
  
  // Set bypass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  // Configure magnetometer measurement mode
  I2CwriteByte(MAG_ADDRESS,0x0A,MAG_CONT_MODE_2);
  Serial.println("\n\nready...");
}

void loop(){
  readAccelGyro(acc, gyr);
  readMag(mag);
  
  // Print
  packet+=String(acc[0])+","+acc[1]+","+acc[2]+","+gyr[0]+","+gyr[1]+","+gyr[2]+","+mag[0]+","+mag[1]+","+mag[2]+",";
  packet.replace(",","\t");//for serial monitor testing
  Serial.println(packet);
  packet = "";
  delay(50); //less than 50ms results in weird behaviour
}

//=========================================
//============Main Functions===============
//=========================================


void readAccelGyro(double* acc, int* gyr){
  /*
   * Reads MPU9250 accelerometer and gyroscope
   * stores X,Y,Z valus in acc[] and gyr[]
   */
  // Read accelerometer and gyroscope
  uint8_t buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,buf);
  
  // Accelerometer
  int16_t ax=buf[0]<<8 | buf[1];
  int16_t ay=buf[2]<<8 | buf[3];
  int16_t az=buf[4]<<8 | buf[5];
  //convert to m/s^2: acceleration = rawData*scale/2^15*9.81
  acc[0]=double(ax)*0x04/0x8000*9.81;
  acc[1]=double(ay)*0x04/0x8000*9.81;
  acc[2]=double(az)*0x04/0x8000*9.81;

  // Gyroscope
  int16_t gx=buf[8]<<8 | buf[9];
  int16_t gy=buf[10]<<8 | buf[11];
  int16_t gz=buf[12]<<8 | buf[13];
  //convert to deg/s: DPS = rawData*scale/2^15
  gyr[0] = gx*250/0x8000;
  gyr[1] = gy*250/0x8000;
  gyr[2] = gz*250/0x8000;
}


void readMag(int* mag){  
  /*
   * Reads MPU9250 magnetometer
   * stores X,Y,Z values in mag[]
   */
  // Read register Status 1 and wait for the DRDY: Data Ready
  uint8_t ST1;
  do{
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t buf[7];
  //I2Cread(uint8_t addr, uint8_t reg, uint8_t Nbytes, uint8_t* data)
  I2Cread(MAG_ADDRESS,0x03,7,buf);
   
  // Create 16 bits values from 8 bits data
 
  // Magnetometer
  int16_t mx=-(buf[1]<<8 | buf[0]);
  int16_t my=-(buf[3]<<8 | buf[2]);
  int16_t mz=-(buf[5]<<8 | buf[4]);
  mag[0] = mx;
  mag[1] = my;
  mag[2] = mz;
}

//=========================================
//============I2C Functions================
//=========================================
void I2CwriteByte(uint8_t addr, uint8_t reg, uint8_t data){
  /*
   * Write a byte (data) in device (addr) at register (reg)
   */
  // Set register address
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void I2Cread(uint8_t addr, uint8_t reg, uint8_t Nbytes, uint8_t* data){
  /*
   * This function read Nbytes bytes from I2C device at address (addr). 
   * Put read bytes starting at register (reg) in the (data) array. 
   */
  // Set register address
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
 
  // Read Nbytes
  Wire.requestFrom(addr, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    data[index++]=Wire.read();
}
