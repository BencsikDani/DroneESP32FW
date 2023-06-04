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

BluetoothSerial SerialBT;
char str[512];


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

  Serial.printf("Waiting for SPI data...\n");
  // block until the transaction comes from master
  slave.wait(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);
  Serial.printf("SPI data has arrived!\n");

  while (slave.available())
  {
    sprintf(str, "");

    // show received data
    for (int i = 0; i < BUFFER_SIZE; ++i)
    {
        sprintf(str, "%s%d ", str, spi_slave_rx_buf[i]);
    }

    sprintf(str, "Throttle: (%d) %d %d %d %d\r\n",
            spi_slave_rx_buf[0],
            spi_slave_rx_buf[1], spi_slave_rx_buf[2], spi_slave_rx_buf[3], spi_slave_rx_buf[4]);
    sprintf(str, "%sYaw: %d\r\n", str, spi_slave_rx_buf[5]);
    sprintf(str, "%sPitch: %d\r\n", str, spi_slave_rx_buf[6]);
    sprintf(str, "%sRoll: %d\r\n", str, spi_slave_rx_buf[7]);
    sprintf(str, "%sSWA: %d\r\n", str, spi_slave_rx_buf[8]);
    sprintf(str, "%sSWB: %d\r\n", str, spi_slave_rx_buf[9]);
    sprintf(str, "%sSWC: %d\r\n", str, spi_slave_rx_buf[10]);
    sprintf(str, "%sSWD: %d\r\n", str, spi_slave_rx_buf[11]);
    sprintf(str, "%sVRA: %d\r\n", str, spi_slave_rx_buf[12]);
    sprintf(str, "%sVRB: %d\r\n", str, spi_slave_rx_buf[13]);

    sprintf(str, "%sTemp: %.4f\r\nAcc:  %1.4f ; %1.4f ; %1.4f\r\nGyro: %1.4f ; %1.4f ; %1.4f\r\n",
            str,
            spi_slave_rx_buf[14],
            spi_slave_rx_buf[15], spi_slave_rx_buf[16], spi_slave_rx_buf[17],
            spi_slave_rx_buf[18], spi_slave_rx_buf[19], spi_slave_rx_buf[20]);
    
    sprintf(str, "%sBMP_Temp: %.4f\r\nBMP_Pres: %.4f\r\nBMP_Alt: %.4f\r\n",
            str,
            spi_slave_rx_buf[21], spi_slave_rx_buf[22], spi_slave_rx_buf[23]);

    sprintf(str, "%sMAG_X_RAW: %.4f\r\nMAG_Y_RAW: %.4f\r\nMAG_Z_RAW: %.4f\r\ndir: %.4f\r\n",
            str,
            spi_slave_rx_buf[24], spi_slave_rx_buf[25], spi_slave_rx_buf[26], spi_slave_rx_buf[27]);

    sprintf(str, "%sDistance: %.0f mm\r\n", str, spi_slave_rx_buf[28]);

    sprintf(str, "%sGPS:\tLat -> %.4f %c\r\n\tLong -> %.4f %c\r\n\tFix -> %d\r\n\tNOS -> %d\r\n\tHDOP -> %.4f\r\n\tAlt -> %.4f %c\r\n",
            str,
            spi_slave_rx_buf[29], spi_slave_rx_buf[30], spi_slave_rx_buf[31], spi_slave_rx_buf[32], spi_slave_rx_buf[33], spi_slave_rx_buf[34], spi_slave_rx_buf[35], spi_slave_rx_buf[36], spi_slave_rx_buf[37]);

    sprintf(str, "%s\r\n\r\n", str);

    Serial.print(str);
    SerialBT.write((uint8_t*)str, strlen(str));

    slave.pop();  // pop the oldest transaction result
  }
}