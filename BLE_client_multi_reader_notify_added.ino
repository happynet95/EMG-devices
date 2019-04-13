                                                                                                        /**
 * A BLE client example that is rich in capabilities.
 * There is a lot new capabilities implemented.
 * author unknown
 * updated by chegewara
 */

#include "BLEDevice.h"
#include "BLEScan.h"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// The remote service we wish to connect to.
static BLEUUID serviceUUID_a("4fafc201-1fb5-459e-8fcc-c5c9c331914a");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID_a("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static boolean doConnect_a = false;
static boolean connected_a = false;
static boolean doScan_a = false;
static BLERemoteCharacteristic* pRemoteCharacteristic_a;
static BLEAdvertisedDevice* myDevice_a;

// The remote service we wish to connect to.
static BLEUUID serviceUUID_b("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID_b("beb5483e-36e1-4688-b7f5-ea07361b26b8");

static boolean doConnect_b = false;
static boolean connected_b = false;
static boolean doScan_b = false;
static BLERemoteCharacteristic* pRemoteCharacteristic_b;
static BLEAdvertisedDevice* myDevice_b;

uint32_t synchro_count[2] = {0,0};

uint32_t PACKET_a[13];
uint32_t PACKET_b[13];

int index_a = 0;
int *pindex_a = &index_a;
int index_b = 0;
int *pindex_b = &index_b;

static int isfull_a = 0;
static int isfull_b = 0;

static int fixcount = 0;

typedef struct{
   uint32_t header;
   uint32_t count;
   uint32_t muscle[10];
   uint32_t add;
}EMG_RES_PACKET;

int fullPACKET_a = 0;
int fullPACKET_b = 0;

static void notifyCallback_a(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  uint32_t myInt = pData[0] + (pData[1] << 8) + (pData[2] << 16) + (pData[3] << 24);
  isfull_a =isfull_a+puttoPACKET(myInt,PACKET_a,synchro_count[0],pindex_a,&(fullPACKET_a));
}
static boolean isheader(uint32_t myInt){
  if(myInt==4294967295){
    return true;
  }
  else{
    return false;
  }
}

static void readPACKETdata(uint32_t res[13],uint32_t synchcount){
  Serial.print("\n");
  Serial.print("count : ");
  Serial.println(res[1]-synchcount);
  for(int packetnum=2;packetnum<12;packetnum++){
    Serial.print(packetnum-1);
    Serial.print("번째 data is : ");
    Serial.println(res[packetnum]);
    }
  Serial.print("add : ");
  Serial.println(res[12]);
  
  Serial.print("count of packet A : ");
  Serial.println(synchro_count[0]);
  Serial.print("count of packet B : ");
  Serial.println(synchro_count[1]);
}

static void synchronize(uint32_t synchro_count[2]){
    synchro_count[0]=PACKET_a[1];
    synchro_count[1]=PACKET_b[1];
}

static void initializePACKET(uint32_t PACKET[13], int *pindex){
  for(int j = 0;j<13;j++){
      PACKET[j]=0;
      }
     *pindex = 0;
}

static int getsumPACKET(uint32_t PACKET[13]){
  uint32_t sum=0;
  for(int k=2;k<12;k++){
    sum += PACKET[k];
    }
  return sum;
}

static int puttoPACKET(uint32_t myInt,uint32_t PACKET[13], uint32_t synchcount, int *pindex, int *pfullPACKET){
  int value_return=0;
  if(isheader(myInt)){
    initializePACKET(PACKET,pindex);
    PACKET[0]=myInt;
    }
   else{
    PACKET[*pindex]=myInt;
    if(*pindex==13){
      initializePACKET(PACKET,pindex);
      }
    else if(*pindex==12){
      if(PACKET[12]>70000){
        initializePACKET(PACKET,pindex);
        }
      else if(getsumPACKET(PACKET)!=PACKET[12]){
        initializePACKET(PACKET,pindex);
      }
      else{
        readPACKETdata(PACKET,synchcount);
        *pindex = 100;
        value_return=1;
         }
      }
   }
   
   (*pindex)++;
   return value_return;
   *pfullPACKET=value_return;
}


static void notifyCallback_b(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  uint32_t myInt = pData[0] + (pData[1] << 8) + (pData[2] << 16) + (pData[3] << 24);
  isfull_b =isfull_b+puttoPACKET(myInt,PACKET_b,synchro_count[1],pindex_b,&(fullPACKET_b));
}

class MyClientCallback_a : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient_a) {
  }

  void onDisconnect(BLEClient* pclient_a) {
    connected_a = false;
    Serial.println("onDisconnect");
  }
};


class MyClientCallback_b : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient_b) {
  }

  void onDisconnect(BLEClient* pclient_b) {
    connected_b = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer_a() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice_a->getAddress().toString().c_str());
    
    BLEClient*  pClient_a  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient_a->setClientCallbacks(new MyClientCallback_a());

    // Connect to the remove BLE Server.
    pClient_a->connect(myDevice_a);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient_a->getService(serviceUUID_a);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID_a.toString().c_str());
      pClient_a->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic_a = pRemoteService->getCharacteristic(charUUID_a);
    if (pRemoteCharacteristic_a == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID_a.toString().c_str());
      pClient_a->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    
    if(pRemoteCharacteristic_a->canNotify())
      pRemoteCharacteristic_a->registerForNotify(notifyCallback_a);

    connected_a = true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks_a: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID_a)) {

      BLEDevice::getScan()->stop();
      myDevice_a = new BLEAdvertisedDevice(advertisedDevice);
      doConnect_a = true;
      doScan_a = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

bool connectToServer_b() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice_b->getAddress().toString().c_str());
    
    BLEClient*  pClient_b  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient_b->setClientCallbacks(new MyClientCallback_b());

    // Connect to the remove BLE Server.
    pClient_b->connect(myDevice_b);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient_b->getService(serviceUUID_b);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID_b.toString().c_str());
      pClient_b->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic_b = pRemoteService->getCharacteristic(charUUID_b);
    if (pRemoteCharacteristic_b == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID_b.toString().c_str());
      pClient_b->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    
    if(pRemoteCharacteristic_b->canNotify())
      pRemoteCharacteristic_b->registerForNotify(notifyCallback_b);

    connected_b = true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks_b: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID_b)) {

      BLEDevice::getScan()->stop(); 
      myDevice_b = new BLEAdvertisedDevice(advertisedDevice);
      doConnect_b = true;
      doScan_b = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void BTwrite(uint32_t Int32){
  uint8_t* myInt = (uint8_t*)&(Int32);
  SerialBT.write(myInt,4);
}

void writeBTPACKET(uint32_t PACKET[13],uint32_t synch_count){
  BTwrite(PACKET[1]-synch_count);
  for(int i = 2;i<12;i++){
    BTwrite(PACKET[i]);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan_a = BLEDevice::getScan();
  pBLEScan_a->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks_a());
  pBLEScan_a->setInterval(1349);
  pBLEScan_a->setWindow(449);
  pBLEScan_a->setActiveScan(true);
  pBLEScan_a->start(5, false);

  BLEScan* pBLEScan_b = BLEDevice::getScan();
  pBLEScan_b->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks_b());
  pBLEScan_b->setInterval(1349);
  pBLEScan_b->setWindow(449);
  pBLEScan_b->setActiveScan(true);
  pBLEScan_b->start(5, false);

  SerialBT.begin("FITBA_home_BT"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

} 


// This is the Arduino main loop function.

void loop() {
    if (doConnect_a == true) {
      if (connectToServer_a()) {
        Serial.println("We are now connected to the BLE Server.");
      } else {
        Serial.println("We have failed to connect to the server; there is nothin more we will do.");
      }
      doConnect_a = false;
    }
  
  //
        //
    // If we are connected to a peer BLE Server, update the characteristic each time we are reached
    // with the current time since boot.
    if (connected_a) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
      digitalWrite(LED_BUILTIN, HIGH); 
    }else if(doScan_a){
      BLEDevice::getScan()->start(0);// this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
    }
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect_b == true) {
    if (connectToServer_b()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect_b = false;
  }

//
      //
  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected_b) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
    digitalWrite(LED_BUILTIN, HIGH);
  }else if(doScan_b){
    BLEDevice::getScan()->start(0);// this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }
  if((isfull_a==1&&isfull_b>1)||(isfull_a>1&&isfull_b==1)){
      synchronize(synchro_count);
      Serial.println("packet counts are successfully synchronized!");
      isfull_a++;
      isfull_b++;
    }
  if(index_a==101&&index_b==101){
    
    Serial.println("I'm Sending Data!!");
    writeBTPACKET(PACKET_a,synchro_count[0]);
    writeBTPACKET(PACKET_b,synchro_count[1]);
    index_a=0;
    index_b=0;
 
    if((PACKET_a[1]-synchro_count[0])>(PACKET_b[1]-synchro_count[1])){
      synchro_count[0]+=((PACKET_a[1]-synchro_count[0])-(PACKET_b[1]-synchro_count[1]));
      fixcount++;
      Serial.print("Fixcount : ");
      Serial.println(fixcount);
    }
    else if((PACKET_a[1]-synchro_count[0])<(PACKET_b[1]-synchro_count[1])){
      synchro_count[1]+=((PACKET_b[1]-synchro_count[1])-(PACKET_a[1]-synchro_count[0]));
      fixcount++;
      Serial.print("Fixcount : ");
      Serial.println(fixcount);
      }
    
  }
// Delay a second between loops.
} // End of loop 
