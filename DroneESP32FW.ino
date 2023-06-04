#include "BluetoothSerial.h"
#include <ESP32SPISlave.h>

#define CLK_PIN   14
#define MISO_PIN  12
#define MOSI_PIN  13
#define SS_PIN    15

#define LED_PIN   2

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

ESP32SPISlave slave;
#define BUFFER_SIZE   64
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];
bool ReceiveDone[3] = {false};

BluetoothSerial SerialBT;
char str[512];

// Data variables
uint8_t Throttle[5];
uint8_t Yaw;
uint8_t Pitch;
uint8_t Roll;
uint8_t SWA;
uint8_t SWB;
uint8_t SWC;
uint8_t SWD;
uint8_t VRA;
uint8_t VRB;

float TempData;
float AccData[3];
float GyroData[3];

float BMP_Temp;
float BMP_Pres;
float BMP_Alt;

float MAG_X_RAW;
float MAG_Y_RAW;
float MAG_Z_RAW;
float MAG_dir;

float Distance;

float dec_latitude;
char ns;
float dec_longitude;
char ew;
uint8_t fix;	// 0 = Invalid
      // 1 = GNSS fix (SPS)
      // 2 = DGPS fix
      // 3 = PPS fix
      // 4 = Real Time Kinematic
      // 5 = Float RTK
      // 6 = estimated (dead reckoning) (2.3 feature)
      // 7 = Manual input mode
      // 8 = Simulation mode
uint8_t num_of_satelites;
float horizontal_dilution_of_precision;
float mean_sea_level_altitude;
char altitude_unit;

void AssembleFloatFromUint8s(uint8_t* array, int position, float* n)
{
  memcpy(n, array+position, sizeof(float));
}


void setup()
{
  // Serial Init
  Serial.begin(115200);
  delay(2000);

  // SPI Slave Init
  pinMode(CLK_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(SS_PIN, INPUT);
  slave.setDataMode(SPI_MODE0);
  slave.begin(HSPI, CLK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  // clear buffers
  memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

  // Bluetooth Init
  SerialBT.begin("Drone-ESP32"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  // LED Init
  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  //if (Serial.available()) {
  //  SerialBT.write(Serial.read());
  //}
  //if (SerialBT.available()) {
  //  Serial.write(SerialBT.read());
  //}

  //Serial.printf("Waiting for SPI data...\n");
  // block until the transaction comes from master
  slave.wait(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);
  //Serial.printf("SPI data has arrived!\n");

  while (slave.available())
  {
    sprintf(str, "");

    if (spi_slave_rx_buf[0] == 'i')
    {
      Throttle[0] = spi_slave_rx_buf[1];
      Throttle[1] = spi_slave_rx_buf[2];
      Throttle[2] = spi_slave_rx_buf[3];
      Throttle[3] = spi_slave_rx_buf[4];
      Throttle[4] = spi_slave_rx_buf[5];
      
      Yaw = spi_slave_rx_buf[6];
      Pitch = spi_slave_rx_buf[7];
      Roll = spi_slave_rx_buf[8];
      SWA = spi_slave_rx_buf[9];
      SWB = spi_slave_rx_buf[10];
      SWC = spi_slave_rx_buf[11];
      SWD = spi_slave_rx_buf[12];
      VRA = spi_slave_rx_buf[13];
      VRB = spi_slave_rx_buf[14];

      ns = (char)spi_slave_rx_buf[15];
      ew = (char)spi_slave_rx_buf[16];
      fix = (char)spi_slave_rx_buf[17];
      num_of_satelites = (char)spi_slave_rx_buf[18];
      altitude_unit = (char)spi_slave_rx_buf[19];

      ReceiveDone[0] = true;
    }
    else if (spi_slave_rx_buf[0] == 'f')
    {
      AssembleFloatFromUint8s(spi_slave_rx_buf, 1, &TempData);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 5, AccData);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 9, AccData+1);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 13, AccData+2);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 17, GyroData);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 21, GyroData+1);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 25, GyroData+2);

      AssembleFloatFromUint8s(spi_slave_rx_buf, 29, &BMP_Temp);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 33, &BMP_Pres);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 37, &BMP_Alt);

      AssembleFloatFromUint8s(spi_slave_rx_buf, 41, &MAG_X_RAW);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 45, &MAG_Y_RAW);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 49, &MAG_Z_RAW);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 53, &MAG_dir);

      AssembleFloatFromUint8s(spi_slave_rx_buf, 57, &Distance);

      ReceiveDone[1] = true;
    }
    else if (spi_slave_rx_buf[0] == 'g')
    {
      AssembleFloatFromUint8s(spi_slave_rx_buf, 1, &dec_latitude);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 5, &dec_longitude);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 9, &horizontal_dilution_of_precision);
      AssembleFloatFromUint8s(spi_slave_rx_buf, 13, &mean_sea_level_altitude);

      ReceiveDone[2] = true;
    }

    slave.pop();  // pop the oldest transaction result
  }

  if (ReceiveDone[0] && ReceiveDone[1] && ReceiveDone[2])
  {
    for (int i = 0; i < 3; i++)
      ReceiveDone[i] = false;

    sprintf(str, "Throttle: (%d) %d %d %d %d\r\n",
            Throttle[0], Throttle[1], Throttle[2], Throttle[3], Throttle[4]);
    sprintf(str, "%sYaw: %d\r\n", str, Yaw);
    sprintf(str, "%sPitch: %d\r\n", str, Pitch);
    sprintf(str, "%sRoll: %d\r\n", str, Roll);
    sprintf(str, "%sSWA: %d\r\n", str, SWA);
    sprintf(str, "%sSWB: %d\r\n", str, SWB);
    sprintf(str, "%sSWC: %d\r\n", str, SWC);
    sprintf(str, "%sSWD: %d\r\n", str, SWD);
    sprintf(str, "%sVRA: %d\r\n", str, VRA);
    sprintf(str, "%sVRB: %d\r\n", str, VRB);

    sprintf(str, "%sTemp: %.4f\r\nAcc:  %1.4f ; %1.4f ; %1.4f\r\nGyro: %1.4f ; %1.4f ; %1.4f\r\n",
            str,
            TempData,
            AccData[0], AccData[1], AccData[2],
            GyroData[0], GyroData[1], GyroData[2]);
    
    sprintf(str, "%sBMP_Temp: %.4f\r\nBMP_Pres: %.4f\r\nBMP_Alt: %.4f\r\n",
            str,
            BMP_Temp, BMP_Pres, BMP_Alt);

    sprintf(str, "%sMAG_X_RAW: %.4f\r\nMAG_Y_RAW: %.4f\r\nMAG_Z_RAW: %.4f\r\ndir: %.4f\r\n",
            str,
            MAG_X_RAW, MAG_Y_RAW, MAG_Z_RAW, MAG_dir);

    sprintf(str, "%sDistance: %.0f mm\r\n", str, Distance);

    sprintf(str, "%sGPS:\tLat -> %.4f %c\r\n\tLong -> %.4f %c\r\n\tFix -> %d\r\n\tNOS -> %d\r\n\tHDOP -> %.4f\r\n\tAlt -> %.4f %c\r\n",
            str,
            dec_latitude, ns, dec_longitude, ew, fix, num_of_satelites, horizontal_dilution_of_precision, mean_sea_level_altitude, altitude_unit);

    sprintf(str, "%s\r\n\r\n", str);

    Serial.print(str);
    SerialBT.write((uint8_t*)str, strlen(str));
  }
}