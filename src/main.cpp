//Libraries

#include "painlessMesh.h"
#include "FS.h"
#include <Arduino.h>
#include "esp_wifi.h"
#include <SPIFFS.h>
#include <ModbusRTU.h>
#include <Arduino_JSON.h>

#include "LiquidCrystal_I2C.h"
#define relayPin 5
#define MAX485_DE_RE 5
#define sendLed 34
#define connLed 32
#define BUTTON_PIN 0

#define CS_PIN 5
#define CLOCK_PIN 18
#define MOSI_PIN 23
#define MISO_PIN 19
#include <AsyncTCP.h>
#define MESH_PORT 5555 // Mesh Port should be same for all  nodes in Mesh Network

uint32_t mfdVals[25];
//objects declaraation

Modbus::ResultCode err;
LiquidCrystal_I2C lcd(0x27, 20, 4);

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;
ModbusRTU mb;
//variables
uint8_t mfd_read_pos = 0,sdQueue;
uint16_t hregs2[96];
float mfdValues[25];
String ssid,password;
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
uint32_t meshNodes[4] = {2137584946,3520592757,2989895757};
xSemaphoreHandle xMutex;

// User stub
  void  mfdConfig();

void blinkLed(void* random);
void updateTime();
void sendMFD();
//void lcdShiet( void *random);
//void meshUpdate(void *random);
void vApplicationIdleHook( void );
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
void sendMsgSd();
void ledUpdate();
void sendPayload(String &payload);
void saveToCard(String &payload);
void updateRssi();
void multi_mfd_read();
 void meshUpdate(void * random);
 void setConnectedNode();
void convertMfdFloats();void writePosToCard();
 void setConnectedNode(){

    if(connectedNode == meshNodes[0]){
        connectedNode = 0;
    }
    else 
    if(connectedNode == meshNodes[1]){
        connectedNode = 1;
    }else 
    if(connectedNode == meshNodes[1]){
        connectedNode = 2;
    }


 }
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
Task taskUpdateTime(TASK_SECOND * 1, TASK_FOREVER, &updateTime); // Set task second to send msg in a time interval (Here interval is 4 second)
Task taskSendMsgSd(TASK_MILLISECOND * 200, TASK_FOREVER, &sendMsgSd); // Set task second to send msg in a time interval
Task taskReadMfd(TASK_MINUTE * 2, TASK_FOREVER, &read_Mfd_Task); // Set task second to send msg in a time interval (Here interval is 4 second)
Task taskMultiMfdRead(TASK_SECOND * 5 , TASK_FOREVER, &multi_mfd_read); // Set task second to send msg in a time interval (Here interval is 4 second)
Task updateLcd(TASK_SECOND * 3 , TASK_FOREVER, &lcdUpdate); // Set task second to send msg in a time interval (Here interval is 4 second)
Task taaskUpdateLed(TASK_SECOND * 3 , TASK_FOREVER, &ledUpdate); // Set task second to send msg in a time interval (Here interval is 4 second)
Task task_blink_con_led(TASK_SECOND * 3 , TASK_FOREVER, &blink_con_led); // Set task second to send msg in a time interval (Here interval is 4 second)


//runs when node recieves something
void receivedCallback(uint32_t from, String &msg)
{  task_blink_con_led.enable();
taaskUpdateLed.enable();        taskSendMsgSd.enableIfNot();

    JSONVar msgConfig = JSON.parse(msg);
    if(msgConfig.hasOwnProperty("meshNodes")  ){
            Serial.println("got Config");
     const char* ID =  msgConfig["id"] ;
     File configFile;
     const char* configType = msgConfig["meshNodes"];
     Serial.print(ID);
       if( String(ID) == id){
           SPIFFS.begin();
if(String(configType) == "config")
           {
           SPIFFS.remove("/config.json");
           configFile = SPIFFS.open("/config.json","w");
          }
          else{

           SPIFFS.remove("/mfdConf.json");
           configFile = SPIFFS.open("/mfdConf.json","w");

          }
          configFile.print(msg);
          configFile.close();
          SPIFFS.end();
       }
    }
 if (msg == "rootFull"){
        rootStorage = false;
    }

    else
    {
        String strMsg = String(msg);
        ts_epoch = msg.toInt();
    }
 Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
}
// runs when a new connection is established
void newConnectionCallback(uint32_t nodeId)
{    //rssi = WiFi.RSSI();

    taaskUpdateLed.enable();
    task_blink_con_led.enable();
    connectedNode = nodeId;
     for ( uint8_t i = 0;i < 4; i++){

     if(connectedNode == meshNodes[i]){
         connectedNode = i ;
        
         i = 4;
     } 

 }
    meshAlive = true;
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
    //String nMap = mesh.asNodeTree().toString();
    //mesh.sendSingle(root, nMap);
    if (mesh.startDelayMeas(root))
    {
        taskSendMsgSd.enable();
     //  String configFile = String(String(id) + "," + String(root) + "," + String(mcp) + "," + String(mfd) + "," + String(pins) + "," + String(sendDelay));
     //   mesh.sendSingle(root, configFile);
    }
  //  taskConnLed.enable();
}
//runs when the topology changes
void changedConnectionCallback()
{
    //rssi = WiFi.RSSI();
    if (rootStorage == true && mesh.isConnected(root))
    {
        taskSendMsgSd.enable();
    }  
    meshAlive = true;
    Serial.printf("Changed connections\n");
    String nMap = mesh.subConnectionJson(true);
    mesh.sendSingle(root, nMap);
}
// for internal timekeeping of the mesh

void droppedConnection(uint32_t node_id)
{   task_blink_con_led.disable();
    taaskUpdateLed.disable();
    meshAlive = false;
    //lcd.setCursor(0, 1);
    //lcd.clear();
   // lcd.println("Lost connection To : " + node_id);
    digitalWrite(connLed, LOW);
    delay(3000);
        rssi = 0;
        dropCounter++;

}
void setup()
{

    lcd.init(); // initialize the //lcd
    lcd.backlight();
    lcd.blink_off();
    Serial.begin(115200);

    Serial2.begin(9600, SERIAL_8E1);
    mb.begin(&Serial2);
    mb.master();
    xMutex = xSemaphoreCreateMutex();

    // parsing the config
   // esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR);

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

     id = ID;    lcd.print("Loaded id: ");
    lcd.print(id);

    Serial.println(ID);
    // Serial.print("Loaded root id: ");
    root =  2137584946; 
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
    mesh.setDebugMsgTypes(ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE); // all types on
    //int channel = 13;
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, 13);
    mesh.setContainsRoot(true);

    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onDroppedConnection(&droppedConnection);
    //declarations for scheduler                                                                                                                                                                                                                                                                                          UwU
    userScheduler.addTask(taskUpdateTime);
    userScheduler.addTask(taskSendMsgSd);
    userScheduler.addTask(taskMultiMfdRead);
    userScheduler.addTask(updateLcd);
  //  userScheduler.addTask(taaskUpdateLed);
    updateLcd.enable();
  //  userScheduler.addTask(task_blink_con_led);
    userScheduler.addTask(taskReadMfd);
    taskReadMfd.enableDelayed(15000);
     
     SPIFFS.end();
     taskUpdateTime.enable();
     //set IO pins
     pinMode(A0, INPUT);           // Define A0 pin as INPUT
     //pinMode(LED_BUILTIN, OUTPUT); // Define LED_BUILTIN as OUTPUT
     //digitalWrite(LED_BUILTIN, HIGH);
     //pinMode(sendLed, OUTPUT);
     pinMode(connLed, OUTPUT);
     pinMode(relayPin, OUTPUT);
     digitalWrite(relayPin, LOW); // Initially the LED will be off
                                   xTaskCreatePinnedToCore(meshUpdate, "meshTask", 40000, meshTaskHandle_t, 3, NULL, 0);
                                  //  xTaskCreatePinnedToCore(lcdShiet, "lcdTask", 10000, NULL, 3, NULL, 0);

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
     xTaskCreatePinnedToCore(schedulerUpdate, "show led shiet", 8000, NULL, 2, NULL,1);
}
void meshUpdate(void *random)
{

    for (;;)
    {
       
        vTaskDelay(10 / portTICK_RATE_MS); //  delay(10);
    }
}void schedulerUpdate(void *random)
{

    for (;;)
    {
        
        vTaskDelay(10 / portTICK_RATE_MS); //  delay(10);
    }
}
void loop()
{   
    // it will run the  scheduler as well
    mesh.update();
    userScheduler.execute();
    //updateRssi(); //maintains the led flash frequency
    //watchdog
    if (wdt == 180 || dropCounter > 5)
    {
        writeTimeToCard();
        while (1)
            ;
    }
if(rebootTime > 10000){ writeTimeToCard(); ESP.restart();}
}
//to dump data from internal storage
void sendMsgSd()
{   digitalWrite(connLed,HIGH);
    if (mesh.isConnected(root) && (rootStorage == true))
    {
        SPIFFS.begin();
        File file = SPIFFS.open("/offlinelog.txt", "r"); // FILE_READ is default so not realy needed but if you like to use this technique for e.g. write you need FILE_WRITE
                                                         //#endif
        if (!SPIFFS.exists("/offlinelog.txt"))
        {   
            Serial.println("Failed to open file for reading");
            taskSendMsgSd.disable();
            pos = 0;
            timeIndex = 0;  
            return;
        }
        String buffer;
        for (int i = 0; i < 1; i++);

        {
            file.seek(pos);
            buffer = file.readStringUntil('\n');
            msgSd = buffer;
            if (buffer != "")
            {
                if (mesh.isConnected(root))
                {   sdQueue++;
                    mesh.sendSingle(root, msgSd);
                }
                else
                {
                    taskSendMsgSd.disable();
                }
                // mesh.sendSingle(root, msgSd);
                Serial.println(msgSd);
                pos = file.position();
            }
            file.close();
            Serial.println(F("DONE Reading"));
        }
        if (buffer == "")
        {   pos = 0;
            String ackMsg = id;
            ackMsg += "sent from sd card";
            //mesh.sendSingle(root, ackMsg);
            SPIFFS.remove("/offlinelog.txt");
            taskSendMsgSd.disable();//ESP.restart();
        }
        if (sdQueue > 50)
        {   sdQueue = 0;
            taskSendMsgSd.disable();
        }
        SPIFFS.end();
    } digitalWrite(connLed,LOW);
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
//to blink the LED
void blink_con_led()
{

    if (led_active == true)
    {
        digitalWrite(connLed, HIGH);
        delay(led_refresh);
        digitalWrite(connLed, LOW);

        led_active = false;
    }

    else
    {
        digitalWrite(connLed, LOW);
        delay(led_refresh);
        digitalWrite(connLed, HIGH);
        led_active = true;
    }
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

void convertMfdFloats(){

    uint8_t j = 0;
    // String msgMfd;
    for (uint8_t i = 0; i <= 48; i++)
    {

        uint16_t temp1[2] = {hregs2[i], hregs2[i + 1]};
        memcpy(&mfdValues[j], temp1, 32);
        Serial.println(mfdValues[j]);
        j++;
        i++;
    }


}
String readMfd(uint16_t devId, uint16_t address, uint8_t iteration){
     String msgMfd;
    if( dataStream(address, devId)){
     msgMfd += time_to_print;
     msgMfd.concat(",");
     msgMfd.concat(id);
     msgMfd.concat(",");
     msgMfd.concat("7");
     msgMfd.concat(",");
     msgMfd.concat(devId); 
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
     String msgToSend = msgMfd;

     return msgToSend;}
     else {
         return String(__null);
         mfd_read_pos++;
         taskMultiMfdRead.restart();
     }
}
void multi_mfd_read()
{   
   
        time_to_print++; //set the time for mfd data to be in sync
        Serial2.end();
        Serial2.begin(9600, SERIAL_8E1);

    msgMfd_payload[0] = readMfd(mfd_dev_id[mfd_read_pos],100,1);
    vTaskDelay(100 / portTICK_RATE_MS);
    msgMfd_payload[1] = readMfd(mfd_dev_id[mfd_read_pos],148,2);
    msgMfd_payload[2] = readMfd(mfd_dev_id[mfd_read_pos],242,3);
  //  msgMfd_payload[3] = readMfd(mfd_dev_id[mfd_read_pos],290,2);


    delay(10);
    sendMFD();
    Serial2.flush();
    mfd_read_pos++;
    if (mfd_read_pos >= device_count)
    {  
        Serial2.end(); /* writeTimeToCard(); void writePosToCard();
ESP.restart();*/
        mfd_read_pos = 0;
        taskMultiMfdRead.disable();

    if (rootStorage == true)
    {
        taskSendMsgSd.enable();
    }    }
}

void sendMFD()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        sendPayload(msgMfd_payload[i]);
    }        


}
//writing data to card
void saveToCard(String &payload, uint16_t wdtOld)
{
    SPIFFS.begin();
    wdt  = wdtOld;
    File dataFile = SPIFFS.open("/offlinelog.txt", "a");
    Serial.println("current data size : ");
    Serial.println(dataFile.size());
    dataFile.println(payload);
    dataFile.close();
    SPIFFS.end();
}
//sending data to root
void sendPayload(String &payload)
{    digitalWrite(connLed,HIGH); 
    uint16_t wdtOld = wdt;

    wdt = 0;

    Serial.println(payload);
    if (mesh.isConnected(root))
    {
        // digitalWrite(sendLed, HIGH);
        taskSendMsgSd.enable();
        mesh.sendSingle(root, String(payload));
        if (rssi <= (-90))
        {
            mesh.sendSingle(root, (String(id) + ("signal Low ")));

            delay(10);
        }
    }
    else
    {
        saveToCard(payload, wdtOld);
    }digitalWrite(connLed,LOW); 
}

boolean infoNow = true;
//uint8_t lcdState; //periodic restart to avoid memory fragmentation


void lcdUpdate()
{
  
  if (rebootTime == 86400)
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
}}
void mfdConfig(){

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
void ledUpdate()
{
    uint8_t signalStr = rssi;
    signalStr = signalStr * (-1);
    if (signalStr > 90)
    {
        task_blink_con_led.setInterval(TASK_SECOND * 2);
    }

  if (signalStr < 90 && signalStr > 80)
    {
        task_blink_con_led.setInterval(TASK_SECOND * 1);
    }
  if (signalStr < 80 && signalStr > 70)
    {
        task_blink_con_led.setInterval(TASK_MILLISECOND * 700);
    }
  if (signalStr < 70 )
    { 
        task_blink_con_led.setInterval(TASK_MILLISECOND * 50 );
    }









}  



/*
void blinkLed(void* random){



for{;;}{


blink_con_led();

}*/