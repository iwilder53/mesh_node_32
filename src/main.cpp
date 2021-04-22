
#include "painlessMesh.h"
#include "FS.h"
#include <Arduino.h>
#include "esp_wifi.h"
#include <SPIFFS.h>
#include <ModbusRTU.h>
#include <Arduino_JSON.h>
#define ESP8266
#include "LiquidCrystal_I2C.h"
#define relayPin 5
#define MAX485_DE_RE 5
#define sendLed 26
#define connLed 26
#define BUTTON_PIN 0
#define ROLE "LMH_1"
#define VERSION "SpyderEye v1.0.2"
//#include "loraMesh.h"
//#include <AsyncTCP.h>
#define MESH_PORT 5555 // Mesh Port should be same for all  nodes in Mesh Network

#include <SPI.h> // include libraries
#include <LoRa.h>

const long frequency = 433E6; // LoRa Frequency

const int csPin = 5;   // LoRa radio chip select
const int resetPin = 14; // LoRa radio reset
const int irqPin = 27;   // change for your board; must be a hardware interrupt pin

uint32_t mfdVals[25];
//objects declaraation
ModbusRTU mb;

Modbus::ResultCode err;
LiquidCrystal_I2C lcd(0x27, 20, 4);
Scheduler userScheduler; // to control your personal task
//loraMesh loramesh(csPin,resetPin,irqPin);
//variables
uint8_t mfd_read_pos = 0, sdQueue;
uint16_t hregs2[96];
float mfdValues[25];
String ssid, password;
int relay_pin_0_min, relay_pin_0_max, relay_pin_1_min, relay_pin_1_max, relay_pin_2_min, relay_pin_2_max, relay_pin_3_min, relay_pin_3_max, relay_pin_04_min, relay_pin_04_max, relay_pin_05_min, relay_pin_05_max, relay_pin_06_min, relay_pin_06_max, relay_pin_07_min, relay_pin_07_max; // Mesh Port should be same for all  nodes in Mesh Network
int bmeRelay_t_Min, bmeRelay_t_Max, bmeRelay_p_Max, bmeRelay_h_Max, bmeRelay_h_Min, bmeRelay_p_Min;
int device_count;
uint16_t mfd_dev_id[10];
uint8_t sendDelay = 1;
unsigned long period = 0;
String id;
int wdt = 0;
int ts_epoch;
int timeIndex;
int rebootTime;
uint8_t dropCounter = 0;
int pos;
uint32_t root;
long previousMillis = 0;
int mcp = 0, mfd = 0, pins = 0, smb = 0;
int first_Reg, second_Reg;
int time_to_print;
bool MCP_Sent = false;
boolean led_active;
int16_t rssi;
uint16_t led_refresh;
uint32_t connectedNode;
String msgMfd_payload[5];
boolean meshAlive;
String msgSd;
bool ackStatus;
uint16_t meshTimer;
boolean rootStorage = true;
TaskHandle_t meshTaskHandle_t;
TaskHandle_t mfdTaskHandle_t = NULL;
uint32_t meshNodes[4] = {2137584946, 2989895757, 3520592757};
xSemaphoreHandle xMutex;

String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xFF;     // address of this device
byte destination = 0xBB;      // destination to send to
long lastSendTime = 0;        // last send time

// User stub
void mfdConfig();
void onTxDone();
void LoRa_sendMessage(String message);

void blinkLed(void *random);
void updateTime();
void sendMFD();
//void lcdShiet( void *random);
//void meshUpdate(void *random);
void vApplicationIdleHook(void);
void schedulerUpdate(void *random);
void writeToCard();
void writeTimeToCard();
void blink_con_led();
String readMfd(uint16_t devId, uint16_t address, uint8_t iteration);
void taskToggle();
bool dataStream(uint16_t address, uint16_t deviceId);
boolean read_Mfd_Task();
void lcdUpdate();
void updateTime();
void ledUpdate();
void sendPayload(String &payload);
void saveToCard(String &payload);
void updateRssi();
void multi_mfd_read();
void meshUpdate(void *random);
void setConnectedNode();
void convertMfdFloats();
void writePosToCard();
 void onReceiveCb(int recieved_packet_size);

void updateTime()
{ //will update time from root && also watchdog
    //digitalWrite(sendLed, LOW);
    wdt++;
    ts_epoch++;
    rebootTime++;
    //  //lcd.blink_on();
    //Serial.println(WiFi.RSSI());
}

//Declarations for tasks scheduling
Task taskUpdateTime(TASK_SECOND * 1, TASK_FOREVER, &updateTime);        // Set task second to send msg in a time interval (Here interval is 4 second)
Task taskReadMfd(TASK_MINUTE * 2, TASK_FOREVER, &read_Mfd_Task);        // Set task second to send msg in a time interval (Here interval is 4 second)
Task taskMultiMfdRead(TASK_SECOND * 10, TASK_FOREVER, &multi_mfd_read); // Set task second to send msg in a time interval (Here interval is 4 second)
Task updateLcd(TASK_SECOND * 3, TASK_FOREVER, &lcdUpdate);              // Set task second to send msg in a time interval (Here interval is 4 second)

// for internal timekeeping of the mesh
void onTxDone()
{
    Serial.println("TxDone"); 
} 
void onReceiveCb(int recieved_packet_size) {
  if (recieved_packet_size == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";                 // payload of packet

  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
    ts_epoch = incoming.toInt();
}

void setup()
{
  // loramesh.init(433E6);
    lcd.init(); // initialize the //lcd
    lcd.backlight();
    lcd.blink_off();
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8E1);
    mb.begin(&Serial2);
    mb.master();  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(frequency))
    {
        Serial.println("LoRa init failed. Check your connections.");
        while (true)
            ; // if failed, do nothing
    }
  

    LoRa.onReceive(onReceiveCb);
    LoRa.setSpreadingFactor(12);
    LoRa.setTxPower(20);
    LoRa.setOCP(140);
    LoRa.onTxDone(onTxDone); LoRa.setCodingRate4(4); //LoRa.setSignalBandwidth(31.25E3);
      LoRa.setSyncWord(0xA5);

    // parsing the config
    /*  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B);*/
    esp_wifi_set_max_tx_power(82);
    SPIFFS.begin();

    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile)
    {
        Serial.println("Failed to open config file");
        //lcd.println("Failed to open config file");
    }
    size_t size = configFile.size();
    if (size > 1024)
    {
        Serial.println("Config file size is too large");
    }
    std::unique_ptr<char[]> buf(new char[size]);

    configFile.readBytes(buf.get(), size);

    StaticJsonDocument<1024> doc;
    auto error = deserializeJson(doc, buf.get());
    if (error)
    {
        Serial.println("Failed to parse config file");
    }
    const char *MESH_PREFIX = doc["ssid"];
    ssid = MESH_PREFIX;
    const char *MESH_PASSWORD = doc["password"];
    password = MESH_PASSWORD;
    const char *ID = doc["id"];

    const char *ROOT = doc["root"];
    const char *MCP = doc["mcp"];
    //const char *MFD = doc["mfd"];
    int MFD = doc["mfd"];
    const char *PINS = doc["pins"];
    const char *DELAY = doc["delay"];

    sendDelay = atoi(DELAY);
    Serial.print("Loaded id: ");

    id = ID;
    lcd.print("Loaded id: ");
    lcd.print(id);

    Serial.println(ID);
    // Serial.print("Loaded root id: ");
    root = 2137584946;
    Serial.println(ROOT);
    mcp = atoi(MCP);
    Serial.println(mcp);

    // mfd = atoi(MFD);
    Serial.println(mfd);
    configFile.close();

    pins = atoi(PINS);
    Serial.print("Loaded MCP pins: ");
    Serial.print(pins);
    Serial.print(" loaded Send Delay: ");
    Serial.print(sendDelay);
    // maintain time in case of wdt reset
    File timeFile = SPIFFS.open("/time.txt", "r");
    if (timeFile)
    {
        String timeTemp = timeFile.readStringUntil('\n');
        ts_epoch = timeTemp.toInt();
        timeFile.close();
    }
    if (SPIFFS.exists("/pos.txt"))
    {
        File posFile = SPIFFS.open("/pos.txt", "r");
        String timeTemp = timeFile.readStringUntil('\n');
        pos = timeTemp.toInt();

        posFile.close();
    }
    Serial.println("position");
    Serial.print(pos);
    mfdConfig();
    //start the mesh
    //declarations for scheduler                                                                                                                                                                                                                                                                                          UwU
    userScheduler.addTask(taskUpdateTime);
    userScheduler.addTask(taskMultiMfdRead);
    userScheduler.addTask(updateLcd);
    updateLcd.enable();
    userScheduler.addTask(taskReadMfd);
    taskReadMfd.enable();

    //  mesh.initOTAReceive(ROLE);

    SPIFFS.end();
    taskUpdateTime.enable();
    pinMode(connLed, OUTPUT);
    xTaskCreatePinnedToCore(schedulerUpdate, "schedulerUpdate", 32000, NULL, 2, NULL, 1);
    
    lcd.clear();
    lcd.print("Hetadatain");

    delay(100);
    lcd.setCursor(3, 1);
    lcd.print("Spyder Eye");
    for (uint8_t i = 3; i < 14; i++)
    {

        lcd.setCursor(i, 1);

        lcd.print(".");
        delay(200);
    }

    lcd.setCursor(13, 1);
    lcd.print(" ");
    lcd.setCursor(14, 1);
    lcd.print(" ");

    lcd.setCursor(3, 1);
    lcd.print("SPYDER EYE");
    lcd.blink_on();
    delay(300);
    //button.begin();
    // Add the callback function to be called when the button is pressed.
    // button.onPressed();*/
}
void schedulerUpdate(void *random)
{

    for (;;)
    {
        userScheduler.execute();

        vTaskDelay(10 / portTICK_RATE_MS); //  delay(10);
    }
}
void loop()
{
    // it will run the  scheduler as well
    //updateRssi(); //maintains the led flash frequency
    //watchdog
    if (wdt == 180 || dropCounter > 5)
    {
        writeTimeToCard();
        while (1)
            ;
    }
    if (rebootTime > 84600)
    {
        writeTimeToCard();
        ESP.restart();
    }
}
void writeTimeToCard()
{
    SPIFFS.begin();
    SPIFFS.remove("/time.txt");
    File dataFile = SPIFFS.open("/time.txt", "w");
    if (dataFile)
    {
        dataFile.println(ts_epoch);
        dataFile.close();
    }
    else
    {
        Serial.println("error opening  time.txt");
    }
    SPIFFS.end();
}
void writePosToCard()
{
    SPIFFS.begin();
    SPIFFS.remove("/pos.txt");
    File dataFile = SPIFFS.open("/pos.txt", "w");
    if (dataFile)
    {
        dataFile.println(pos);
        dataFile.close();
    }
    else
    {
        Serial.println("error opening  time.txt");
    }
    SPIFFS.end();
}
boolean read_Mfd_Task()
{
    MCP_Sent = false;
    //  userScheduler.disableAll();
    taskUpdateTime.enableIfNot();
    // vTaskSuspend(meshTaskHandle_t);
    time_to_print = ts_epoch;
    taskMultiMfdRead.enable();
    // xTaskCreate(multi_mfd_read,"readsMFD",20000,NULL,2,&mfdTaskHandle_t);
    return true;
}

bool resCallback(Modbus::ResultCode event, uint16_t, void *)
{
    err = event;
}

Modbus::ResultCode readSync(uint16_t Address, uint16_t start, uint16_t num, uint16_t *buf)
{
    // xSemaphoreTake(xMutex, portMAX_DELAY);
    if (mb.slave())
    {
        //    xSemaphoreGive(xMutex);
        return Modbus::EX_GENERAL_FAILURE;
    }
    Serial.printf("SlaveID: %d Hreg %d\n", Address, start);
    mb.readHreg(Address, start, buf, num, resCallback);
    while (mb.slave())
    {
        mb.task();
    }
    Modbus::ResultCode res = err;
    //  xSemaphoreGive(xMutex);
    return res;
}
bool dataStream(uint16_t address, uint16_t deviceId)
{

    if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
    {
        Serial.println("OK 2");
        meshTimer = 0;
        convertMfdFloats();
        return true;
    }
    else
    {
        meshTimer++;
        Serial.print("Error trying again ! ");
        if (meshTimer >= 3)
        {
            return false;
        }
        else
        { // dataStream(address,deviceId);

            if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
            {
                Serial.println("OK 2");
                meshTimer = 0;
                convertMfdFloats();
                return true;
            }
            else
            {
                if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
                {
                    Serial.println("OK 2");
                    meshTimer = 0;
                    convertMfdFloats();
                    return true;
                }
                else
                {
                    if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
                    {
                        Serial.println("OK 2");
                        meshTimer = 0;
                        convertMfdFloats();
                        return true;
                    }
                    else
                    {
                        if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
                        {
                            Serial.println("OK 2");
                            meshTimer = 0;
                            convertMfdFloats();
                            return true;
                        }
                        else
                        {
                            if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
                            {
                                Serial.println("OK 2");
                                meshTimer = 0;
                                convertMfdFloats();
                                return true;
                            }
                            else
                            {
                                if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
                                {
                                    Serial.println("OK 2");
                                    meshTimer = 0;
                                    convertMfdFloats();
                                    return true;
                                }
                                else
                                {
                                    if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
                                    {
                                        Serial.println("OK 2");
                                        meshTimer = 0;
                                        convertMfdFloats();
                                        return true;
                                    }
                                    else
                                    {
                                        uint8_t j = 0;
                                        // String msgMfd;
                                        for (uint8_t i = 0; i <= 48; i++)
                                        {
                                            mfdValues[i] = 0;
                                        }

                                        return false;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void convertMfdFloats()
{

    uint8_t j = 0;
    // String msgMfd;
    for (uint8_t i = 0; i <= 48; i++)
    {

        uint16_t temp1[2] = {hregs2[i], hregs2[i + 1]};
        memcpy(&mfdValues[j], temp1, 32); i++;j++;
    }
}
String readMfd(uint16_t devId, uint16_t address, uint8_t iteration)
{
    String msgMfd;
    //   updateLcd.disable();
    lcd.clear();
    time_to_print = ts_epoch;

    lcd.print("Reading device");
    lcd.setCursor(0, 2);
    lcd.print(String(devId));
    if (dataStream(address, devId))

    {
        msgMfd += time_to_print;
        msgMfd.concat(",");
        msgMfd.concat(id);
        msgMfd.concat(",");
        msgMfd.concat(devId);
        msgMfd.concat(",");
        msgMfd.concat("12");
        msgMfd.concat(",");
        msgMfd.concat(iteration);
        msgMfd.concat(",");

        for (uint8_t i = 0; i <= 23; i++)
        {
            msgMfd.concat(mfdValues[i]);
            if (i <= 22)
            {
                msgMfd.concat(",");
            }
        }
        lcd.clear();
        lcd.setCursor(0, 2);
        lcd.print("Success");
        delay(500);
        String msgToSend = msgMfd;

        return msgToSend;
    }
    else
    {
        lcd.clear();
        lcd.setCursor(0, 2);
        lcd.print("Failed");
        delay(500);
        mfd_read_pos++;
        taskMultiMfdRead.restart();
        msgMfd += time_to_print;
        msgMfd.concat(",");
        msgMfd.concat(id);
        msgMfd.concat(",");
        msgMfd.concat(devId);
        msgMfd.concat(",");
        msgMfd.concat("13");
        msgMfd.concat(",");
        msgMfd.concat(iteration);
        msgMfd.concat(",");

        for (uint8_t i = 0; i <= 23; i++)
        {
            msgMfd.concat(mfdValues[i]);
            if (i <= 22)
            {
                msgMfd.concat(",");
            }
        }
        return msgMfd;
    }
}
void multi_mfd_read()
{

    time_to_print++; //set the time for mfd data to be in sync
    Serial2.end();
    Serial2.begin(9600, SERIAL_8E1);

    msgMfd_payload[0] = readMfd(mfd_dev_id[mfd_read_pos], 100, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
    msgMfd_payload[1] = readMfd(mfd_dev_id[mfd_read_pos], 148, 2);
    msgMfd_payload[2] = readMfd(mfd_dev_id[mfd_read_pos], 242, 3);
    //  msgMfd_payload[3] = readMfd(mfd_dev_id[mfd_read_pos],290,2);

    delay(10);
    sendMFD();
    Serial2.flush();
    mfd_read_pos++;
    if (mfd_read_pos >= device_count)
    {
        Serial2.end();
        mfd_read_pos = 0;
        taskMultiMfdRead.disable();
    
    }
}

void sendMFD()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        sendPayload(msgMfd_payload[i]);
        vTaskDelay(3000 / portTICK_RATE_MS);
    }
}
//writing data to card
void saveToCard(String &payload, uint16_t wdtOld)
{
    SPIFFS.begin();
    wdt = wdtOld;
    File dataFile = SPIFFS.open("/offlinelog.txt", "a");
    Serial.println("current data size : ");
    Serial.println(dataFile.size());
    dataFile.println(payload);
    dataFile.close();
    SPIFFS.end();
}
//sending data to root
void sendPayload(String &payload)
{
    digitalWrite(connLed, HIGH);
    uint16_t wdtOld = wdt;

    wdt = 0;

    Serial.println(payload);
    LoRa_sendMessage(payload);
        // digitalWrite(sendLed, HIGH);

    digitalWrite(connLed, LOW);
}

boolean infoNow = true;
//uint8_t lcdState; //periodic restart to avoid memory fragmentation

void lcdUpdate()
{

    if (rebootTime == 3600)
    {
        writeTimeToCard();
        ESP.restart();
    }
    //lcd.init();

    if (infoNow == true)
    {
        infoNow = false;
        if (meshAlive || rssi != 0)
        {

            lcd.blink_off();
            lcd.clear();

            lcd.setCursor(0, 0);
            lcd.print("Connected To : ");
            lcd.setCursor(3, 1);
            if (connectedNode == 0)
            {
                lcd.print("ROOT NODE");
            }
            else
            {
                lcd.print(String(connectedNode));
            }
        }
        else
        {
            lcd.clear();
            lcd.print("Searching");
        }
    }
    else
    {
        infoNow = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Spyder EYE");
        lcd.setCursor(0, 1);
        lcd.print("Signal :");
        lcd.setCursor(9, 1);
        lcd.print(String(WiFi.RSSI()) + "db");
        /*  lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Free Ram :");
        lcd.setCursor(3, 1);
        lcd.print(String(ESP.getFreeHeap()));*/

        // vTaskDelay(120000/portTICK_RATE_MS);
    }
}
void mfdConfig()
{

    SPIFFS.begin();

    File configFile = SPIFFS.open("/mfdConf.json", "r");
    if (!configFile)
    {
        Serial.println("Failed to open config file");
        //lcd.println("Failed to open config file");
    }
    size_t size = configFile.size();
    if (size > 1024)
    {
        Serial.println("Config file size is too large");
    }
    std::unique_ptr<char[]> buf(new char[size]);

    configFile.readBytes(buf.get(), size);

    StaticJsonDocument<1024> doc;
    auto error = deserializeJson(doc, buf.get());
    if (error)
    {
        Serial.println("Failed to parse config file");
    }
    const char *DEV_COUNT = doc["device_count"];
    device_count = atoi(DEV_COUNT);
    Serial.println(device_count);

    const char *MFD_SERIAL_ID_1 = doc["mfd_dev_id_1"];
    mfd_dev_id[0] = atoi(MFD_SERIAL_ID_1);
    Serial.println(mfd_dev_id[0]);

    const char *MFD_SERIAL_ID_2 = doc["mfd_dev_id_2"];
    mfd_dev_id[1] = atoi(MFD_SERIAL_ID_2);
    Serial.println(mfd_dev_id[1]);

    const char *MFD_SERIAL_ID_3 = doc["mfd_dev_id_3"];
    mfd_dev_id[2] = atoi(MFD_SERIAL_ID_3);
    Serial.println(mfd_dev_id[2]);

    const char *MFD_SERIAL_ID_4 = doc["mfd_dev_id_4"];
    mfd_dev_id[3] = atoi(MFD_SERIAL_ID_4);
    Serial.println(mfd_dev_id[3]);

    const char *MFD_SERIAL_ID_5 = doc["mfd_dev_id_5"];
    mfd_dev_id[4] = atoi(MFD_SERIAL_ID_5);
    Serial.println(mfd_dev_id[4]);

    const char *MFD_SERIAL_ID_6 = doc["mfd_dev_id_6"];
    mfd_dev_id[5] = atoi(MFD_SERIAL_ID_6);
    Serial.println(mfd_dev_id[5]);

    const char *MFD_SERIAL_ID_7 = doc["mfd_dev_id_7"];
    mfd_dev_id[6] = atoi(MFD_SERIAL_ID_7);
    Serial.println(mfd_dev_id[6]);

    const char *MFD_SERIAL_ID_8 = doc["mfd_dev_id_8"];
    mfd_dev_id[7] = atoi(MFD_SERIAL_ID_8);
    Serial.println(mfd_dev_id[7]);
}

void LoRa_sendMessage(String message)
{
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(message.length());        // add payload length
  LoRa.print(message);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                   // go back into receive mode
 // finish packet and send it  
  LoRa.receive();  
}