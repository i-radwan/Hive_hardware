// =================
// Program Args:
//  - Robot ID.
//  - Automatic Done Period ms. If left blank, you will have to manullay send the Done via command [1].

// Config: At first you will receive this buffer [4, id], the id is the program argument. This is a
//         quick hack to idenify the robot without the need for knowing the port beforehand.
//         You can run multiple instances of this program with different IDs as args.

// =================
// These commands and actions have to follow the agreed server/HW protocol.

// Input Commands:
//  - 1: Done
//  - 2: Blocked
//  - 3: Unblocked
//  - 4: Battery level (followed by the battery level, e.g. (1 4 6):: Agent#1 battery level is now 6)
//  - 5: Error message (followed by the error code)

// =================
// Possible server actions:
//  - Stop
//  - Move
//  - Rotate right
//  - Rotate left
//  - Retreat
//  - Load
//  - Offload

// =================
// Constants
const PORT = 8080;

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

            if (buffer[1] == 0) { // Stop
                console.log("Received Stop...");
                
                moving = false;
            } else if (buffer[1] == 1) { // Move
                if (buffer[2] == 0) {
                    console.log("Received Move...");
                } else {
                    console.log("Received Recover Move...");
                }

                moving = true;

                automaticDone();
            } else if (buffer[1] == 2) { // Rotate Right
                if (buffer[2] == 0) {
                    console.log("Received Rotate Right...");
                } else {
                    console.log("Received Recover Rotate Right...");
                }

                moving = true;

                automaticDone();
            } else if (buffer[1] == 3) { // Rotate Left
                if (buffer[2] == 0) {
                    console.log("Received Rotate Left...");
                } else {
                    console.log("Received Recover Rotate Left...");
                }

                moving = true;

                automaticDone();
            } else if (buffer[1] == 4) { // Retreat
                if (buffer[2] == 0) {
                    console.log("Received Retreat...");
                } else {
                    console.log("Received Recover Retreat...");
                }

                moving = true;

                automaticDone();
            } else if (buffer[1] == 5) { // Load
                if (buffer[2] == 0) {
                    console.log("Received Load...");
                } else {
                    console.log("Received Recover Load...");
                }

                sendDone();
            } else if (buffer[1] == 6) { // Offload
                if (buffer[2] == 0) {
                    console.log("Received Offload...");
                } else {
                    console.log("Received Recover Offload...");
                }

                sendDone();
            }
        } else if (buffer[0] == 2) { // Lights
            console.log("Received Lights Configs...");
        }
    });

    console.log('WebSocket Client Connected');

    // =================
    // Initial Config
    connection.sendBytes(Buffer.from([4, id]));

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
        } else if (parseInt(i[0]) == 5) { // Error [error code]
            moving = false;

            sendError(parseInt(i[1]));
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

        connection.sendBytes(Buffer.from([0]));
    };

    let sendBatteryLevel = function(l) {
        console.log("Sending Battery Level...");

        connection.sendBytes(Buffer.from([1, l]));
    };

    let sendError = function(l) {
        console.log("Sending Error...");

        connection.sendBytes(Buffer.from([3, l]));
    };

    let sendBlocked = function() {
        console.log("Sending Blocked...");

        connection.sendBytes(Buffer.from([2, 1]));
    };

    let sendUnblocked = function() {
        console.log("Sending Unblocked...");

        connection.sendBytes(Buffer.from([2, 0]));
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
