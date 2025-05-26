#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include <WiFiClient.h>


#define LED_STATUS 25

/* Wi-Fi info */
char ssid[] = "YourSSID";  // your network SSID (name)
char pass[] = "YourPass";   // your network password
WiFiMulti multi;


int status = WL_IDLE_STATUS;  // the Wifi radio's status

char server[] = "api.thingspeak.com";  // server address
String apiKey = "APIkey";    // apki key

// Thingspeak

unsigned long lastThingSpeakUpdate = 0;  // last time you connected to the server, in milliseconds
unsigned long lastConnectionTime = 0;    // last time you connected to the server, in milliseconds
unsigned long NRF24_connect_interval = 0;
WiFiClient client;

#define CE_PIN 5
#define CSN_PIN 8

// RF24 configuration
RF24 radio(CE_PIN, CSN_PIN);


uint8_t address[][6] = { "1Node", "2Node" };
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = false;     // true = TX role, false = RX role
float payload = 0.0;


struct TempData {
  float temp1;
  float temp2;
  float temp3;
};

TempData data;
unsigned long prevMillis = 0;
int LED_delay = 1000;

void setup() {
//  Serial.begin(9600);
  //while (!Serial)
    ;  // <-- Add this line!

pinMode(LED_BUILTIN, OUTPUT);

  SPI.setRX(4);
  SPI.setTX(7);
  SPI.setSCK(6);
  SPI.setCS(5);

  // We start by connecting to a WiFi network
  
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  multi.addAP(ssid, pass);

  if (multi.run() != WL_CONNECTED) {
    Serial.println("Unable to connect to network");
    LED_delay = 300;
    //rp2040.reboot();
  }
  else{
    LED_delay = 2000;
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
  }
  radio.setPALevel(RF24_PA_MAX);                    // RF24_PA_MAX is default.
  radio.setPayloadSize(sizeof(data));            // float datatype occupies 4 bytes
  radio.stopListening(address[radioNumber]);        // put radio in TX mode
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
  if (!role) {
    radio.startListening();  // put radio in RX mode
  }
}

void loop() {

   if (millis() - prevMillis > LED_delay) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    prevMillis = millis();
  }

   if (millis() - NRF24_connect_interval > 30000) {

  if (multi.run() != WL_CONNECTED) {
    LED_delay = 300;
  }
  else{
    LED_delay = 2000;
  }

      NRF24_connect_interval = millis();
   }
  if (role) {
    // This device is a TX node

    unsigned long start_timer = micros();                // start the timer
    bool report = radio.write(&payload, sizeof(float));  // transmit & save the report
    unsigned long end_timer = micros();                  // end the timer

    if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);  // print the timer result
      Serial.print(F(" us. Sent: "));
      Serial.println(payload);  // print payload sent
      payload += 0.01;          // increment float payload
    } else {
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    }

    // to make this example readable in the serial monitor
   delay(1000); 

  } else {
    // This device is a RX node

    uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that received it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&data, bytes);                // fetch payload from FIFO
      Serial.print(F("Received "));
      Serial.print(bytes);  // print the size of the payload
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);  // print the pipe number
      Serial.print(F(": "));
      Serial.print("Data1:");
      Serial.println(data.temp1);
      Serial.print("Data2:");
      Serial.println(data.temp2);
      Serial.print("Data3:");
      Serial.println(data.temp3);
    }
  }  // role


  // Log Temperature to ThingSpeak every 30 min
  if (millis() - lastThingSpeakUpdate >= 600000) { // 30 minutes600000

    SendThingSpeak();
    lastThingSpeakUpdate = millis();
  }

}


void SendThingSpeak() {


  if (multi.run() != WL_CONNECTED) {
    multi.addAP(ssid, pass);
    delay(5000);
  }
  printWifiStatus();
  // close any connection before send a new request
  // this will free the socket on the WiFi shield
  client.stop();

  // if there's a successful connection
  if (client.connect(server, 80)) {
    Serial.println("Connecting...");

    // send the Get request
    client.print(F("GET /update?api_key="));
    client.print(apiKey);
    client.print(F("&field3="));
    client.print(String(data.temp1));
    client.print(F("&field4="));
    client.print(String(data.temp2));
    client.print(F("&field5="));
    client.print(String(data.temp3));
    client.println();
    // note the time that the connection was made
    lastConnectionTime = millis();
   
  } else {
    // if you couldn't make a connection
   
    Serial.println("Connection failed");
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
