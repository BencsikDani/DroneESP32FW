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
uint8_t spi_slave_rx_buf[BUFFER_SIZE];
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
//bool TuneDataToTransmit = false;
int BluetoothDataBuffer = 0;
uint8_t BluetoothData[16] = { 0 };
bool ReceiveDone[5] = {false};
bool Diag = false;
bool Tune = true;

BluetoothSerial SerialBT;
bool forward_bluetooth_data;
char str[512];

// Data variables
uint16_t Throttle[5];
int16_t Pitch;
int16_t Roll;
int16_t Yaw;
uint16_t SWA;
uint16_t SWB;
uint16_t SWC;
uint16_t SWD;
uint16_t VRA;
uint16_t VRB;

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


float PID_Ro_Kp;
float PID_Ro_Ki;
float PID_Ro_Kd;
int16_t PID_Ro_ref;
float PID_Ro_meas;
float PID_Ro_out;

float PID_Ri_Kp;
float PID_Ri_Ki;
float PID_Ri_Kd;
float PID_Ri_ref;
float PID_Ri_meas;
int16_t PID_Ri_out;

float PID_Po_Kp;
float PID_Po_Ki;
float PID_Po_Kd;
int16_t PID_Po_ref;
float PID_Po_meas;
float PID_Po_out;

float PID_Pi_Kp;
float PID_Pi_Ki;
float PID_Pi_Kd;
float PID_Pi_ref;
float PID_Pi_meas;
int16_t PID_Pi_out;

float PID_Y_Kp;
float PID_Y_Ki;
float PID_Y_Kd;
int16_t PID_Y_ref;
float PID_Y_meas;
int16_t PID_Y_out;

float PID_T_Kp;
float PID_T_Ki;
float PID_T_Kd;
uint16_t PID_T_ref;
float PID_T_meas;
uint16_t PID_T_out;

void FloatToUint8s(float* src, uint8_t* array, int position)
{
	memcpy(array+position, src, sizeof(float));
}

void FloatFromUint8s(uint8_t* array, int position, float* dest)
{
  memcpy(dest, array+position, sizeof(float));
}

void Uint16FromUint8s(uint8_t* array, int position, uint16_t* dest)
{
  memcpy(dest, array+position, sizeof(uint16_t));
}

void Int16FromUint8s(uint8_t* array, int position, int16_t* dest)
{
  memcpy(dest, array+position, sizeof(int16_t));
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
  slave.setDataMode(SPI_MODE3);
  slave.begin(HSPI, CLK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  // clear buffers
  memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

  // Bluetooth Init
  SerialBT.begin("Drone-ESP32"); //Bluetooth device name
  forward_bluetooth_data = false;
  Serial.println("The device started, now you can pair it with bluetooth!");

  // LED Init
  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  // Receive data on Bluetooth Serial
  int i = 0;
  while (SerialBT.available())
  {
    // Signal that it can be forwarded
    forward_bluetooth_data = true;

    BluetoothDataBuffer = SerialBT.read();
    Serial.write((uint8_t)BluetoothDataBuffer);

    // Store the data
    BluetoothData[i] = (uint8_t)BluetoothDataBuffer;
    if (i < 16-1)
      i++;
  }



  // If there is Bluetooth data to be forwarded
  if (forward_bluetooth_data)
  {
    String float_string = String((char*)BluetoothData+9);
    float float_value = float_string.toFloat();

    if (BluetoothData[3] == '1') // PID_Ro
      spi_slave_tx_buf[1] = 1;
    else if (BluetoothData[3] == '2') // PID_Ri
      spi_slave_tx_buf[1] = 2;
    else if (BluetoothData[3] == '3') // PID_Po
      spi_slave_tx_buf[1] = 3;
    else if (BluetoothData[3] == '4') // PID_Pi
      spi_slave_tx_buf[1] = 4;
    else if (BluetoothData[3] == '5') // PID_Y
      spi_slave_tx_buf[1] = 5;
    else if (BluetoothData[3] == '6') // PID_T
      spi_slave_tx_buf[1] = 6;

    if (BluetoothData[6] == 'p') // Kp
      spi_slave_tx_buf[2] = 'p';
    else if (BluetoothData[6] == 'i') // Ki
      spi_slave_tx_buf[2] = 'i';
    else if (BluetoothData[6] == 'd') // Kd
      spi_slave_tx_buf[2] = 'd';

    FloatToUint8s(&float_value, spi_slave_tx_buf, 3); // Float value

    spi_slave_tx_buf[7] = '\r';
    spi_slave_tx_buf[8] = '\n';

    forward_bluetooth_data = false;
  }



  // block until the transaction comes from master
  if (slave.remained() == 0)
    slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

  while (slave.available())
  {
    if (spi_slave_rx_buf[0] == 'i')
    {
      Uint16FromUint8s(spi_slave_rx_buf, 1, &(Throttle[0]));
      Uint16FromUint8s(spi_slave_rx_buf, 3, &(Throttle[1]));
      Uint16FromUint8s(spi_slave_rx_buf, 5, &(Throttle[2]));
      Uint16FromUint8s(spi_slave_rx_buf, 7, &(Throttle[3]));
      Uint16FromUint8s(spi_slave_rx_buf, 9, &(Throttle[4]));
      
      Int16FromUint8s(spi_slave_rx_buf, 11, &(Pitch));
      Int16FromUint8s(spi_slave_rx_buf, 13, &(Roll));
      Int16FromUint8s(spi_slave_rx_buf, 15, &(Yaw));
      Uint16FromUint8s(spi_slave_rx_buf, 17, &(SWA));
      Uint16FromUint8s(spi_slave_rx_buf, 19, &(SWB));
      Uint16FromUint8s(spi_slave_rx_buf, 21, &(SWC));
      Uint16FromUint8s(spi_slave_rx_buf, 23, &(SWD));
      Uint16FromUint8s(spi_slave_rx_buf, 25, &(VRA));
      Uint16FromUint8s(spi_slave_rx_buf, 27, &(VRB));

      ns = (char)spi_slave_rx_buf[28];
      ew = (char)spi_slave_rx_buf[29];
      fix = (uint8_t)spi_slave_rx_buf[30];
      num_of_satelites = (uint8_t)spi_slave_rx_buf[31];
      altitude_unit = (char)spi_slave_rx_buf[32];

      Int16FromUint8s(spi_slave_rx_buf, 33, &(PID_Ri_out));

      ReceiveDone[0] = true;
    }
    else if (spi_slave_rx_buf[0] == 'f' && ReceiveDone[0])
    {
      FloatFromUint8s(spi_slave_rx_buf, 1, &TempData);
      FloatFromUint8s(spi_slave_rx_buf, 5, AccData);
      FloatFromUint8s(spi_slave_rx_buf, 9, AccData+1);
      FloatFromUint8s(spi_slave_rx_buf, 13, AccData+2);
      FloatFromUint8s(spi_slave_rx_buf, 17, GyroData);
      FloatFromUint8s(spi_slave_rx_buf, 21, GyroData+1);
      FloatFromUint8s(spi_slave_rx_buf, 25, GyroData+2);

      FloatFromUint8s(spi_slave_rx_buf, 29, &BMP_Temp);
      FloatFromUint8s(spi_slave_rx_buf, 33, &BMP_Pres);
      FloatFromUint8s(spi_slave_rx_buf, 37, &BMP_Alt);

      FloatFromUint8s(spi_slave_rx_buf, 41, &MAG_X_RAW);
      FloatFromUint8s(spi_slave_rx_buf, 45, &MAG_Y_RAW);
      FloatFromUint8s(spi_slave_rx_buf, 49, &MAG_Z_RAW);
      FloatFromUint8s(spi_slave_rx_buf, 53, &MAG_dir);

      FloatFromUint8s(spi_slave_rx_buf, 57, &Distance);

      ReceiveDone[1] = true;
    }
    else if (spi_slave_rx_buf[0] == 'g' && ReceiveDone[0] && ReceiveDone[1])
    {
      FloatFromUint8s(spi_slave_rx_buf, 1, &dec_latitude);
      FloatFromUint8s(spi_slave_rx_buf, 5, &dec_longitude);
      FloatFromUint8s(spi_slave_rx_buf, 9, &horizontal_dilution_of_precision);
      FloatFromUint8s(spi_slave_rx_buf, 13, &mean_sea_level_altitude);

      ReceiveDone[2] = true;
    }
    else if (spi_slave_rx_buf[0] == 't')
    {
      FloatFromUint8s(spi_slave_rx_buf, 1, &PID_Ro_Kp);
      FloatFromUint8s(spi_slave_rx_buf, 5, &PID_Ro_Ki);
      FloatFromUint8s(spi_slave_rx_buf, 9, &PID_Ro_Kd);
      Int16FromUint8s(spi_slave_rx_buf, 13, &PID_Ro_ref);
      FloatFromUint8s(spi_slave_rx_buf, 15, &PID_Ro_meas);
      FloatFromUint8s(spi_slave_rx_buf, 19, &PID_Ro_out);

      FloatFromUint8s(spi_slave_rx_buf, 23, &PID_Ri_Kp);
      //FloatFromUint8s(spi_slave_rx_buf, 27, &PID_Ri_Ki);
      FloatFromUint8s(spi_slave_rx_buf, 27, &PID_Ri_Kd);
      FloatFromUint8s(spi_slave_rx_buf, 31, &PID_Ri_ref);
      FloatFromUint8s(spi_slave_rx_buf, 35, &PID_Ri_meas);
      Int16FromUint8s(spi_slave_rx_buf, 39, &PID_Ri_out);

      FloatFromUint8s(spi_slave_rx_buf, 41, &PID_Y_Kp);
      FloatFromUint8s(spi_slave_rx_buf, 45, &PID_Y_Ki);
      FloatFromUint8s(spi_slave_rx_buf, 49, &PID_Y_Kd);

      Uint16FromUint8s(spi_slave_rx_buf, 53, &PID_T_ref);
      FloatFromUint8s(spi_slave_rx_buf, 55, &PID_T_meas);
      Uint16FromUint8s(spi_slave_rx_buf, 59, &PID_T_out);

      ReceiveDone[3] = true;
    }
    else if (spi_slave_rx_buf[0] == 'u')
    {
      FloatFromUint8s(spi_slave_rx_buf, 1, &PID_Po_Kp);
      FloatFromUint8s(spi_slave_rx_buf, 5, &PID_Po_Ki);
      FloatFromUint8s(spi_slave_rx_buf, 9, &PID_Po_Kd);
      Int16FromUint8s(spi_slave_rx_buf, 13, &PID_Po_ref);
      FloatFromUint8s(spi_slave_rx_buf, 15, &PID_Po_meas);
      FloatFromUint8s(spi_slave_rx_buf, 19, &PID_Po_out);

      FloatFromUint8s(spi_slave_rx_buf, 23, &PID_Pi_Kp);
      //FloatFromUint8s(spi_slave_rx_buf, 27, &PID_Pi_Ki);
      FloatFromUint8s(spi_slave_rx_buf, 27, &PID_Pi_Kd);
      FloatFromUint8s(spi_slave_rx_buf, 31, &PID_Pi_ref);
      FloatFromUint8s(spi_slave_rx_buf, 35, &PID_Pi_meas);
      Int16FromUint8s(spi_slave_rx_buf, 39, &PID_Pi_out);
      
      Int16FromUint8s(spi_slave_rx_buf, 41, &PID_Y_ref);
      FloatFromUint8s(spi_slave_rx_buf, 43, &PID_Y_meas);
      Int16FromUint8s(spi_slave_rx_buf, 47, &PID_Y_out);

      FloatFromUint8s(spi_slave_rx_buf, 49, &PID_T_Kp);
      FloatFromUint8s(spi_slave_rx_buf, 53, &PID_T_Ki);
      FloatFromUint8s(spi_slave_rx_buf, 57, &PID_T_Kd);

      ReceiveDone[4] = true;
    }

    slave.pop();  // pop the oldest transaction result
  }

  if (Diag)
  {
    if (ReceiveDone[0] && ReceiveDone[1] && ReceiveDone[2])
    {
      ReceiveDone[0] = false;
      ReceiveDone[1] = false;
      ReceiveDone[2] = false;
      
      sprintf(str, "");

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

  else if (Tune)
  {
    if (ReceiveDone[0])
    {
      ReceiveDone[0] = false;
      
      sprintf(str, "");

      sprintf(str, "Throttle: (%d) %d %d %d %d\r\n",
            Throttle[0], Throttle[1], Throttle[2], Throttle[3], Throttle[4]);
      sprintf(str, "%s\r\n\r\n", str);

      Serial.print(str);
      SerialBT.write((uint8_t*)str, strlen(str));
    }

    if (ReceiveDone[3])
    {
      ReceiveDone[3] = false;

      sprintf(str, "");

      sprintf(str, "%sPID_Ro_Kp: %.3f\r\n", str, PID_Ro_Kp);
      sprintf(str, "%sPID_Ro_Ki: %.3f\r\n", str, PID_Ro_Ki);
      sprintf(str, "%sPID_Ro_Kd: %.3f\r\n", str, PID_Ro_Kd);
      sprintf(str, "%sPID_Ro_ref: %d\r\n", str, PID_Ro_ref);
      sprintf(str, "%sPID_Ro_meas: %.3f\r\n", str, PID_Ro_meas);
      sprintf(str, "%sPID_Ro_out: %.3f\r\n", str, PID_Ro_out);

      sprintf(str, "%sPID_Ri_Kp: %.3f\r\n", str, PID_Ri_Kp);
      sprintf(str, "%sPID_Ri_Ki: %.3f\r\n", str, PID_Ri_Ki);
      sprintf(str, "%sPID_Ri_Kd: %.3f\r\n", str, PID_Ri_Kd);
      sprintf(str, "%sPID_Ri_ref: %.3f\r\n", str, PID_Ri_ref);
      sprintf(str, "%sPID_Ri_meas: %.3f\r\n", str, PID_Ri_meas);
      sprintf(str, "%sPID_Ri_out: %d\r\n", str, PID_Ri_out);

      sprintf(str, "%sPID_Y_Kp: %.3f\r\n", str, PID_Y_Kp);
      sprintf(str, "%sPID_Y_Ki: %.3f\r\n", str, PID_Y_Ki);
      sprintf(str, "%sPID_Y_Kd: %.3f\r\n", str, PID_Y_Kd);

      sprintf(str, "%sPID_T_ref: %d\r\n", str, PID_T_ref);
      sprintf(str, "%sPID_T_meas: %.3f\r\n", str, PID_T_meas);
      sprintf(str, "%sPID_T_out: %d\r\n", str, PID_T_out);

      sprintf(str, "%s\r\n\r\n", str);

      SerialBT.write((uint8_t*)str, strlen(str));
    }
    else if (ReceiveDone[4])
    {
      ReceiveDone[4] = false;

      sprintf(str, "");

      sprintf(str, "%sPID_Po_Kp: %.3f\r\n", str, PID_Po_Kp);
      sprintf(str, "%sPID_Po_Ki: %.3f\r\n", str, PID_Po_Ki);
      sprintf(str, "%sPID_Po_Kd: %.3f\r\n", str, PID_Po_Kd);
      sprintf(str, "%sPID_Po_ref: %d\r\n", str, PID_Po_ref);
      sprintf(str, "%sPID_Po_meas: %.3f\r\n", str, PID_Po_meas);
      sprintf(str, "%sPID_Po_out: %.3f\r\n", str, PID_Po_out);

      sprintf(str, "%sPID_Pi_Kp: %.3f\r\n", str, PID_Pi_Kp);
      sprintf(str, "%sPID_Pi_Ki: %.3f\r\n", str, PID_Pi_Ki);
      sprintf(str, "%sPID_Pi_Kd: %.3f\r\n", str, PID_Pi_Kd);
      sprintf(str, "%sPID_Pi_ref: %.3f\r\n", str, PID_Pi_ref);
      sprintf(str, "%sPID_Pi_meas: %.3f\r\n", str, PID_Pi_meas);
      sprintf(str, "%sPID_Pi_out: %d\r\n", str, PID_Pi_out);
      
      sprintf(str, "%sPID_Y_ref: %d\r\n", str, PID_Y_ref);
      sprintf(str, "%sPID_Y_meas: %.3f\r\n", str, PID_Y_meas);
      sprintf(str, "%sPID_Y_out: %d\r\n", str, PID_Y_out);

      sprintf(str, "%sPID_T_Kp: %.3f\r\n", str, PID_T_Kp);
      sprintf(str, "%sPID_T_Ki: %.3f\r\n", str, PID_T_Ki);
      sprintf(str, "%sPID_T_Kd: %.3f\r\n", str, PID_T_Kd);

      sprintf(str, "%s\r\n\r\n", str);

      SerialBT.write((uint8_t*)str, strlen(str));
    }
  }
}
