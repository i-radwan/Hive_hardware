#pragma once

#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>
#include <string.h>

#include "utils/constants.h"

class Communicator {
public:

    bool setup(void (*receive)(SERVER_TASKS)) {
        receiveCallback = receive;

        wifiConnected = connectWifi();

        // Only proceed if wifi connection successful
        if(wifiConnected) {
            WSConnected = connectWS();
        }

        return wifiConnected && WSConnected;
    }

    void sendDone() {
        uint8_t msg[1] = {(uint8_t) MSG_TO_SERVER::DONE};
        send(msg);
    }

    void sendBatteryLevel(uint8_t level) {
        uint8_t msg[2] = {(uint8_t) MSG_TO_SERVER::BATTERY, level};
        send(msg);
    }

    void sendBlockingState(BLOCKING_MODE mode) {
        uint8_t msg[2] = {(uint8_t) MSG_TO_SERVER::BLOCKING, (uint8_t) mode};
        send(msg);
    }

    inline void sendStr(String str) { // For debugging!
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
    static void (*receiveCallback)(SERVER_TASKS);

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

    static void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
        switch(type) {
            case WStype_DISCONNECTED:
                break;
            case WStype_CONNECTED:
                webSocket.sendTXT("Connected");
                break;
            case WStype_TEXT:
                break;
            case WStype_BIN:
                {
                    SERVER_TASKS task = SERVER_TASKS::OTHER;

                    if (payload[0] == (int) MSG_FROM_SERVER::CONFIG) {
                        task = SERVER_TASKS::CONFIG;
                    } else if (payload[0] == (int) MSG_FROM_SERVER::ACTION) {
                        if (payload[1] == (int) ACTIONS::STOP) {
                            task = SERVER_TASKS::STOP;
                        } else if (payload[1] == (int) ACTIONS::MOVE) {
                            task = SERVER_TASKS::MOVE;
                        } else if (payload[1] == (int) ACTIONS::RETREAT) {
                            task = SERVER_TASKS::RETREAT;
                        } else if (payload[1] == (int) ACTIONS::ROTATE_LEFT) {
                            task = SERVER_TASKS::ROTATE_LEFT;
                        } else if (payload[1] == (int) ACTIONS::ROTATE_RIGHT) {
                            task = SERVER_TASKS::ROTATE_RIGHT;
                        } else if (payload[1] == (int) ACTIONS::LOAD) {
                            task = SERVER_TASKS::LOAD;
                        } else if (payload[1] == (int) ACTIONS::OFFLOAD) {
                            task = SERVER_TASKS::OFFLOAD;
                        }
                    } else if (payload[0] == (int) MSG_FROM_SERVER::LIGHTS) {
                        if (payload[1] == (int) LIGHTS::RED) {
                            if (payload[2] == (int) LIGHT_MODE::OFF) {
                                task = SERVER_TASKS::RED_LED_OFF;
                            } else if (payload[2] == (int) LIGHT_MODE::ON) {
                                task = SERVER_TASKS::RED_LED_ON;
                            } else if (payload[2] == (int) LIGHT_MODE::FLASH) {
                                task = SERVER_TASKS::RED_LED_FLASH;
                            }
                        } else if (payload[1] == (int) LIGHTS::BLUE) {
                            if (payload[2] == (int) LIGHT_MODE::OFF) {
                                task = SERVER_TASKS::BLUE_LED_OFF;
                            } else if (payload[2] == (int) LIGHT_MODE::ON) {
                                task = SERVER_TASKS::BLUE_LED_ON;
                            } else if (payload[2] == (int) LIGHT_MODE::FLASH) {
                                task = SERVER_TASKS::BLUE_LED_FLASH;
                            }
                        }
                    }

                    receiveCallback(task);
                }
                break;
            case WStype_PING:
                break;
            case WStype_PONG:
                break;
        }
    }

    inline void send(uint8_t msg[]) {
        webSocket.sendBIN(msg, sizeof msg);
    }
};

WebSocketsClient Communicator::webSocket;
void (*Communicator::receiveCallback)(SERVER_TASKS) = NULL;