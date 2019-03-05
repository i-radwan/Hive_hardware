// Processing UDP example to send and receive binary data
// press f to toggel between LED flickering twice per second or not
// the LED will illuminate when any other key is held down and
// turn back off when the key is released.

import hypermedia.net.*;

UDP udp; // define the UDP object

String ip = "192.168.1.10"; // the remote IP address
int port = 12345; // the destination port

long previousMillis = 0;
int light = 0;
long interval = 500;
int flicker = 0;
int held = 0;

void setup() {
    udp = new UDP(this, 12345); // create a new datagram connection on port 8888
    //udp.log( true ); // <– printout the connection activity
    udp.listen(true); // and wait for incoming message
}

void draw() {
      
}

void keyPressed() {
    if (key == CODED) {
        if (keyCode == UP) {
            byte[] message = new byte[1];
            message[0] = 1;
            udp.send(message, ip, port);
        } else if (keyCode == DOWN) {
            byte[] message = new byte[1];
            message[0] = 2;
            udp.send(message, ip, port);          
        } else if (keyCode == LEFT) {
            byte[] message = new byte[1];
            message[0] = 3;
            udp.send(message, ip, port);          
        } else if (keyCode == RIGHT) {
            byte[] message = new byte[1];
            message[0] = 4;
            udp.send(message, ip, port);          
        }
    } else if (key == ' ') {
        byte[] message = new byte[1];
        message[0] = 0;
        udp.send(message, ip, port);
        System.out.println("Stop");    
    }      
}

void receive(byte[] data) { // <– default handler
    //void receive( byte[] data, String ip, int port ) { // <– extended handler

    for (int i = 0; i < data.length; i++)
        print(char(data[i]));
    println();
}
