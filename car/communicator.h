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

    void issueDone() {
        doneIssued = true;
        doneIssueTime = millis();
    }

    void sendDone() {
        uint8_t msg[1] = {(uint8_t) MSG_TO_SERVER::DONE};
        send(msg, 1);
    }

    void sendBatteryLevel(uint8_t level) {
        uint8_t msg[2] = {(uint8_t) MSG_TO_SERVER::BATTERY, level};
        send(msg, 2);
    }

    void sendBlockingState(BLOCKING_MODE mode) {
        uint8_t msg[2] = {(uint8_t) MSG_TO_SERVER::BLOCKING, (uint8_t) mode};
        send(msg, 2);
    }

    inline void sendStr(String str) { // For debugging!
        webSocket.sendTXT(str);
    }

    void loop() {
        webSocket.loop();

        if (doneIssued && millis() - doneIssueTime > DONE_DELAY) {
            sendDone();

            doneIssued = false;
        }
    }

private:

    bool doneIssued = false;
    double doneIssueTime = 0;

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
        webSocket.begin(SERVER, PORT, "/");

        // Event handler
        webSocket.onEvent(webSocketEvent);

        // Try every RECONNECT_INTERVAL again if the connection has failed.
        webSocket.setReconnectInterval(RECONNECT_INTERVAL);

        // Start heartbeat (optional)
        // ping server every PING_INTERVAL ms
        // expect pong from server within PONG_TIMEOUT ms
        // consider connection disconnected if pong is not received RETRIES_COUNT times

        webSocket.enableHeartbeat(PING_INTERVAL, PONG_TIMEOUT, RETRIES_COUNT);

        return true;
    }

    static void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
        switch(type) {
            case WStype_DISCONNECTED:
                break;
            case WStype_CONNECTED:
                break;
            case WStype_TEXT:
                break;
            case WStype_BIN:
                {
                    SERVER_TASKS task = SERVER_TASKS::OTHER;

                    if (payload[0] == (int) MSG_FROM_SERVER::CONFIG) {
                        task = SERVER_TASKS::CONFIG;
                    } else if (payload[0] == (int) MSG_FROM_SERVER::ACTION) {
                        if (payload[1] == (int) ACTION::STOP) {
                            task = SERVER_TASKS::STOP;
                        } else if (payload[1] == (int) ACTION::MOVE) {
                            task = SERVER_TASKS::MOVE;
                        } else if (payload[1] == (int) ACTION::RETREAT) {
                            task = SERVER_TASKS::RETREAT;
                        } else if (payload[1] == (int) ACTION::ROTATE_LEFT) {
                            task = SERVER_TASKS::ROTATE_LEFT;
                        } else if (payload[1] == (int) ACTION::ROTATE_RIGHT) {
                            task = SERVER_TASKS::ROTATE_RIGHT;
                        } else if (payload[1] == (int) ACTION::LOAD) {
                            task = SERVER_TASKS::LOAD;
                        } else if (payload[1] == (int) ACTION::OFFLOAD) {
                            task = SERVER_TASKS::OFFLOAD;
                        }
                    } else if (payload[0] == (int) MSG_FROM_SERVER::LIGHT) {
                        if (payload[1] == (int) LIGHT::RED) {
                            if (payload[2] == (int) LIGHT_MODE::OFF) {
                                task = SERVER_TASKS::RED_LED_OFF;
                            } else if (payload[2] == (int) LIGHT_MODE::ON) {
                                task = SERVER_TASKS::RED_LED_ON;
                            } else if (payload[2] == (int) LIGHT_MODE::FLASH) {
                                task = SERVER_TASKS::RED_LED_FLASH;
                            }
                        } else if (payload[1] == (int) LIGHT::BLUE) {
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

    inline void send(uint8_t msg[], int length) {
        webSocket.sendBIN(msg, length);
    }
};

WebSocketsClient Communicator::webSocket;
void (*Communicator::receiveCallback)(SERVER_TASKS) = NULL;