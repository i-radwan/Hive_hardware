#pragma once

#include <ESP8266WiFi.h>
#include <WebSocketClient.h>
#include <WebSocketsClient.h>
#include <string.h>

#include "utils/constants.h"

class Communicator {
public:

    bool setup(void (*receive)(MSG)) {
        receiveCallback = receive;

        wifiConnected = connectWifi();

        // Only proceed if wifi connection successful
        if(wifiConnected) {
            WSConnected = connectWS();
        }

        return wifiConnected && WSConnected;
    }

    void sendACK() {
        send(MSG_ACK);
    }

    inline void send(String str) {
        webSocket.sendTXT(str);
    }

    void loop() {
        webSocket.loop();
    }

private:
    bool wifiConnected = false;

    // Websocket
    bool WSConnected = false;
    static WebSocketsClient webSocket;
    static void (*receiveCallback)(MSG);

    static void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
        switch(type) {
            case WStype_DISCONNECTED:
                break;
            case WStype_CONNECTED:
                webSocket.sendTXT("Connected");
                break;
            case WStype_TEXT:
            {
                *payload -= '0';

                MSG msg = MSG::NONE;

                if (*payload == (int) MSG::STOP) {
                    msg = MSG::STOP;
                } else if (*payload == (int) MSG::MOVE) {
                    msg = MSG::MOVE;
                } else if (*payload == (int) MSG::ROTATE_LEFT) {
                    msg = MSG::ROTATE_LEFT;
                } else if (*payload == (int) MSG::ROTATE_RIGHT) {
                    msg = MSG::ROTATE_RIGHT;
                } else if (*payload == (int) MSG::RETREAT) {
                    msg = MSG::RETREAT;
                }

                receiveCallback(msg);
            }
                break;
            case WStype_BIN:
                break;
            case WStype_PING:
                break;
            case WStype_PONG:
                break;
        }
    }

    bool connectWS() {
        // Server address, port and URL
        webSocket.begin(SERVER, WS_PORT, "/");

        // Event handler
        webSocket.onEvent(webSocketEvent);

        // Try every RECONNECT_INTERVAL again if the connection has failed.
        webSocket.setReconnectInterval(RECONNECT_INTERVAL);

        // Start heartbeat (optional)
        // ping server every 15000 ms
        // expect pong from server within 3000 ms
        // consider connection disconnected if pong is not received 2 times

        // webSocket.enableHeartbeat(15000, 3000, 2);

        return true;
    }

    bool connectWifi() {
        WiFi.mode(WIFI_STA);
        WiFi.begin(NET_NAME, NET_PASS);

        bool state = true;
        int i = 0;
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);

            if (i > 10) {
                state = false;
                break;
            }

            i++;
        }

        return state;
    }
};

WebSocketsClient Communicator::webSocket;
void (*Communicator::receiveCallback)(MSG) = NULL;