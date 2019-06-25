// =================
// Program Args:
//  - Robot ID.
//  - Automatic Done Period ms. If left blank, you will have to manullay send the Done via command [1].

// Config: At first you will receive this buffer [3, id], the id is the program argument. This is a
//         quick hack to idenify the robot without the need for knowing the port beforehand.
//         You can run multiple instances of this program with different IDs as args.

// =================
// These commands and actions have to follow the agreed server/HW protocol.

// Input: Agent# Command
//  - Command:
//      • 1: Done
//      • 2: Blocked
//      • 3: Unblocked
//      • 4: Battery level (followed by the battery level, e.g. (1 4 6):: Agnet#1 battery level is now 6)

// =================
// Possible server actions:
//  - Stop
//  - Move
//  - Rotate right
//  - Rotate left
//  - Retreat

// =================
// Constants
const PORT = 12344;

// =================
// Args
let id = parseInt(process.argv[2]);
let donePeriod = process.argv.length == 4 ? parseInt(process.argv[3]) : -1;

// =================
// Connection vars
let WebSocketClient = require('websocket').client;
let client = new WebSocketClient();

// =================
// Operation vars
let moving = false;

client.on('connect', function(connection) {
    connection.on('error', function(error) {
        console.log("Connection Error: " + error.toString());
    });

    connection.on('close', function() {
        clearInterval(heartbeat);
        console.log('echo-protocol Connection Closed');
    });

    connection.on('message', function(message) {
        let buffer = new Uint8Array(message.binaryData);
        if (buffer[0] == 0) { // Config
            console.log("Received Config...");
        } else if (buffer[0] == 1) { // Action
            console.log("Received Stop...");

            if (buffer[1] == 0) { // Stop
                moving = false;
            } else if (buffer[1] == 1) { // Move
                console.log("Received Move...");

                moving = true;

                automaticDone();
            } else if (buffer[1] == 2) { // Rotate Right
                console.log("Received Rotate Right...");

                moving = true;

                automaticDone();
            } else if (buffer[1] == 3) { // Rotate Left
                console.log("Received Rotate Left...");

                moving = true;

                automaticDone();
            } else if (buffer[1] == 4) { // Retreat
                console.log("Received Retreat...");

                moving = true;

                automaticDone();
            } else if (buffer[1] == 5) { // Load
                console.log("Received Load...");

                sendDone();
            } else if (buffer[1] == 6) { // Offload
                console.log("Received Offload...");

                sendDone();
            }
        } else if (buffer[0] == 2) { // Lights
            console.log("Received Lights Configs...");
        }
    });

    console.log('WebSocket Client Connected');

    // =================
    // Initial Config
    let msg = Buffer.alloc(2);
    msg[0] = 3;
    msg[1] = id;

    connection.sendBytes(msg);

    // =================
    // Handle user input
    let stdin = process.openStdin();
    stdin.addListener("data", function (d) {
        let i = d.toString().trim().split(" ");

        if (parseInt(i[0]) == 1) { // Done
            moving = false;

            sendDone();
        } else if (parseInt(i[0]) == 2) { // Blocked
            moving = false;

            sendBlocked();
        } else if (parseInt(i[0]) == 3) { // Unblocked
            sendUnblocked();
        } else if (parseInt(i[0]) == 4) { // Battery [level]
            sendBatteryLevel(parseInt(i[1]));
        }
    });

    // =================
    // Ping-pong
    let heartbeat = setInterval(function() {
        connection.ping(function() {});
    }, 100);

    // =================
    // Sending functions
    let sendDone = function() {
        console.log("Sending Done...");

        let msg = Buffer.alloc(1);
        msg[0] = 0;
        connection.sendBytes(msg);
    };

    let sendBlocked = function() {
        console.log("Sending Blocked...");

        let msg = Buffer.alloc(2);
        msg[0] = 2;
        msg[1] = 1;
        connection.sendBytes(msg);
    };

    let sendUnblocked = function() {
        console.log("Sending Unblocked...");

        let msg = Buffer.alloc(2);
        msg[0] = 2;
        msg[1] = 0;
        connection.sendBytes(msg);
    };

    let sendBatteryLevel = function(l) {
        console.log("Sending Battery Level...");

        let msg = Buffer.alloc(2);
        msg[0] = 2;
        msg[1] = l;
        connection.sendBytes(msg);
    };

    // =================
    // Helper functions
    let automaticDone = function() {
        if (donePeriod == -1)
            return;

        setTimeout(function() {
            if (moving) {
                moving = false;

                sendDone();
            }
        }, donePeriod);
    };
});

client.on('connectFailed', function(error) {
    console.log('Connect Error: ' + error.toString());
});

client.connect('ws://127.0.0.1:' + PORT);