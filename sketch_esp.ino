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
static constexpr uint32_t BUFFER_SIZE {8};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

BluetoothSerial SerialBT;
char str[30];


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
  slave.begin(VSPI, CLK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
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
    sprintf(str, "%s\r\n", str);

    Serial.print(str);
    SerialBT.write((uint8_t*)str, strlen(str));

    slave.pop();  // pop the oldest transaction result
  }
}