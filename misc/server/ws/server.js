const readline = require('readline');

let WebSocketServer = require('websocket').server;
let http = require('http');

let server = http.createServer(function (request, response) {
});

server.listen(12344, function () {
});

// create the server
wsServer = new WebSocketServer({
    httpServer: server
});

// WebSocket server
wsServer.on('request', function (request) {
    let s;
    let con = request.accept(null, request.origin);

    console.log("Connected!");

    con.on('message', function (message) {
        let m = message.utf8Data;

        console.log("Received::\n" + m + "\nRoundtrip time: " + (new Date() - s));
    });

    con.on('close', function (con) {
    });

    console.log("0: STOP\n1: Move\n2: Retreat\n3: Rotate left\n4: Rotate right")
    // Take my input as orders
    let stdin = process.openStdin();
    stdin.addListener("data", function (d) {
        let i = d.toString().trim().split(" ");

        s = new Date();

        con.sendUTF(i[0]);

    });
});