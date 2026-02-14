#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "MYTESTNETWORK";
const char* password = "QueenMary12356!";

WiFiUDP UDP;
unsigned int localUdpPort = 4210;  // Local port to listen on
char incomingPacket[255];  // Buffer for incoming packets

void setup() {
  Serial.begin(115200);
  
  // Create WiFi network
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  // Start UDP
  UDP.begin(localUdpPort);
  
  // Print connection info
  Serial.println("");
  Serial.printf("Created network: %s\n", ssid);
  Serial.printf("Network IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("UDP Port: %d\n", localUdpPort);
}

void loop() {
  int packetSize = UDP.parsePacket();
  if (packetSize) {
    // Receive incoming UDP packet
    int len = UDP.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;  // Null-terminate string
    }
    
    Serial.printf("Received %d bytes from %s, port %d: %s\n", 
                  packetSize, 
                  UDP.remoteIP().toString().c_str(), 
                  UDP.remotePort(), 
                  incomingPacket);
                  
    // Send response back
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.write("Message received: ");
    UDP.write(incomingPacket);
    UDP.endPacket();
  }
  delay(10);
}