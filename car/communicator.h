#pragma once

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <string.h>

#include "utils/constants.h"

class Communicator {
public:

    bool setup() {
        wifiConnected = connectWifi();
    
        // Only proceed if wifi connection successful
        if(wifiConnected) {
            udpConnected = connectUDP();
        }    

        return wifiConnected && udpConnected;
    }

    MSG receive() {
        int packetSize = UDP.parsePacket();
        
        if(packetSize) {
            // Serial.print("Received packet of size " + (String) packetSize + " From ");
            
            // IPAddress remote = UDP.remoteIP();
            // for (int i = 0; i < 4; i++)
            // {
            //     Serial.print(remote[i], DEC);
            //     if (i < 3)
            //     {
            //         Serial.print(".");
            //     }
            // }

            // Read the packet into packetBufffer
            UDP.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
            
            if (packetBuffer[0] == STOP) {
                return STOP;
            } else if (packetBuffer[0] == FORWARD) {
                return FORWARD;
            } else if (packetBuffer[0] == BACKWARD) {
                return BACKWARD;      
            } else if (packetBuffer[0] == LEFT) {
                return LEFT;
            } else if (packetBuffer[0] == RIGHT) {
                return RIGHT;
            } else if (packetBuffer[0] == UP) {
                return UP;
            } else if (packetBuffer[0] == DOWN) {
                return DOWN;
            }
        }

        return NONE;
    }

    void sendACK() {
        send(MSG_ACK);
    }

    inline void send(String str) {
        UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
        UDP.write(str.c_str());
        UDP.endPacket();
    }
    
private: 
    bool wifiConnected = false;
 
    // UDP variables
    WiFiUDP UDP;
    bool udpConnected = false;
    char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; // Buffer to hold incoming packet.
        
    bool connectUDP() {
        bool state = false;
    
        // Serial.println("");
        // Serial.println("Connecting to UDP");
    
        if(UDP.begin(PORT) == 1) {
            // Serial.println("Connection successful");
            state = true;
        } else {
            // Serial.println("Connection failed");
        }
    
        return state;
    }

    bool connectWifi() {
        WiFi.mode(WIFI_STA);
        WiFi.begin(NET_NAME, NET_PASS);
        
        // Serial.println("");
        // Serial.print("Connecting to WiFi ");
    
        bool state = true;
        int i = 0;
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            // Serial.print(".");
            
            if (i > 10) {
                state = false;
                break;
            }
            
            i++;
        }

        // Serial.println("");
        // if (state) {
        //     Serial.print("Connected to " + (String) NET_NAME + "IP address: ");
        //     Serial.println(WiFi.localIP());
        // } else {
        //     Serial.println("Connection failed.");
        // }
        
        return state;
    }
};
