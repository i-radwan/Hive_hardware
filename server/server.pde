import hypermedia.net.*;

UDP udp; // Define the UDP object

String ip = "192.168.1.12"; // The remote IP address
int port = 12345; // The destination port
double time;

void setup() {
    udp = new UDP(this, 12345);
    udp.listen(true);
    time = millis();
}

void draw() {
    //if (millis() - time > 100){
    //      byte[] message = new byte[1];
    //      message[0] = 18;
    //      udp.send(message, ip, port);
          
    //      time = millis();
    //}    
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
    } else if (key == 'u' || key == 'U') {
        byte[] message = new byte[1];
        message[0] = 5;
        udp.send(message, ip, port);
    } else if (key == 'd' || key == 'D') {
        byte[] message = new byte[1];
        message[0] = 6;
        udp.send(message, ip, port);
    }
}

void receive(byte[] data) {
    if (data.length > 1) {
        for (int i = 0; i < data.length; i++)
            print(char(data[i]));
        
        println();
        println();
    }
    
    // Send ACK. TODO: for debugging only
    byte[] message = new byte[1];
    message[0] = (byte) 255;
    udp.send(message, ip, port);
}
