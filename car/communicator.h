#pragma once

#include <ESP8266WiFi.h>
#include <WebSocketClient.h>
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
            tcpConnected = true || connectTCP(); // ToDo
        }

        return wifiConnected && udpConnected && tcpConnected;
    }

    MSG receive(bool tcp = false) {
        if (tcp) {
            String data;
            webSocketClient.getData(data);

            if (data.length() > 0) {

            }

            return MSG::NONE;
        }

        int packetSize = UDP.parsePacket();

        if (packetSize) {
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

            if (packetBuffer[0] == (int) MSG::STOP) {
                return MSG::STOP;
            } else if (packetBuffer[0] == (int) MSG::MOVE) {
                return MSG::MOVE;
            } else if (packetBuffer[0] == (int) MSG::RETREAT) {
                return MSG::RETREAT;
            } else if (packetBuffer[0] == (int) MSG::ROTATE_LEFT) {
                return MSG::ROTATE_LEFT;
            } else if (packetBuffer[0] == (int) MSG::ROTATE_RIGHT) {
                return MSG::ROTATE_RIGHT;
            }
        }

        return MSG::NONE;
    }

    void sendACK() {
        send(MSG_ACK);
    }

    inline void send(String str, bool tcp = false) {
        if (tcp) {
            webSocketClient.sendData(str);
        }

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

    // TCP variables
    WebSocketClient webSocketClient;
    WiFiClient client;
    bool tcpConnected = false;

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

    bool connectTCP() {
        bool state = client.connect(SERVER, TCP_PORT);

        strcpy(webSocketClient.path, "/");
        strcpy(webSocketClient.host, SERVER);

        state &= webSocketClient.handshake(client);

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
