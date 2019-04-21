import hypermedia.net.*;
import java.util.concurrent.ThreadLocalRandom;

UDP udp; // Define the UDP object

final String MSG_SET = "0";
final String MSG_ACK = "1";
final String MSG_ERR = "2";

String ip = "192.168.1.6"; // The remote IP address
int port = 12345; // The destination port
double time;
boolean ack = false;
double ackTime;

final int dir[][] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}}; // Right, Up, Left, Down

boolean moving = false;

int x = 0, y = 0;
int dirIdx = 0;
final int xLimit = 3;
final int yLimit = 1;

int i = 0;
final int iLimit = 1000;

boolean started = false;

void setup() {
    udp = new UDP(this, 12345);
    udp.listen(true);
    time = millis();
}

void draw() {
    if (millis() - time > 100 && moving){
          byte[] message = new byte[1];
          message[0] = (byte)255;
          udp.send(message, ip, port);
          
          time = millis();
    }    
    
    if (started && millis() - ackTime > 10000) {
        ack = true;
        ackTime = millis();
    }
    
    if (ack && i < iLimit) {
        int randomNum = ThreadLocalRandom.current().nextInt(0, 3);
        
        if (randomNum == 0 && x + dir[dirIdx][0] >= 0 && x + dir[dirIdx][0] <= xLimit && y + dir[dirIdx][1] >= 0 && y + dir[dirIdx][1] <= yLimit) {
            println("New pos @ " + i + " : " + x + ", " + y + " - " + dirIdx);
            
            byte[] message = new byte[1];
            message[0] = 1;
            udp.send(message, ip, port);
            
            ack = false;
            moving = true;
            
            x += dir[dirIdx][0];
            y += dir[dirIdx][1];
            
            i++;
        } else if (randomNum == 1) {
            //byte[] message = new byte[1];
            //message[0] = 2;
            //udp.send(message, ip, port);  
            
            //ack = false;
            //moving = true; 
        }  else if (randomNum == 2) {
            println("New pos @ " + i + " : " + x + ", " + y + " - " + dirIdx);

            byte[] message = new byte[1];
            message[0] = 3;
            udp.send(message, ip, port);    
            
            ack = false;
            moving = true;
            
            dirIdx = (dirIdx + 1) % 4;
            
            i++;
        }  else if (randomNum == 3) {
            println("New pos @ " + i + " : " + x + ", " + y + " - " + dirIdx);

            byte[] message = new byte[1];
            message[0] = 4;
            udp.send(message, ip, port);   
            
            ack = false;
            moving = true;
            
            dirIdx = (dirIdx - 1 + 4) % 4;
            
            i++;
        } 
    }
}

void keyPressed() {
    if (key == CODED) {
        if (keyCode == UP/* && x + dir[dirIdx][0] >= 0 && x + dir[dirIdx][0] <= xLimit && y + dir[dirIdx][1] >= 0 && y + dir[dirIdx][1] <= yLimit*/) {            
            byte[] message = new byte[1];
            message[0] = 1;
            udp.send(message, ip, port);
            
            moving = true;
            ack = false;
            
            x += dir[dirIdx][0];
            y += dir[dirIdx][1];
        } else if (keyCode == DOWN) {
            byte[] message = new byte[1];
            message[0] = 2;
            udp.send(message, ip, port);    
            
            moving = true;
            ack = false;
        } else if (keyCode == LEFT) {
            byte[] message = new byte[1];
            message[0] = 3;
            udp.send(message, ip, port);     
            
            moving = true;
            ack = false;    
        } else if (keyCode == RIGHT) {
            byte[] message = new byte[1];
            message[0] = 4;
            udp.send(message, ip, port);    
            
            moving = true;
            ack = false;      
        }
    } else if (key == ' ') {
        byte[] message = new byte[1];
        message[0] = 0;
        udp.send(message, ip, port);
        
        moving = false;             
        ack = false;
        started = true;
    } else if (key == 'u' || key == 'U') {
        byte[] message = new byte[1];
        message[0] = 5;
        udp.send(message, ip, port);
                   
        ack = false;
    } else if (key == 'd' || key == 'D') {
        byte[] message = new byte[1];
        message[0] = 6;
        udp.send(message, ip, port);
                   
        ack = false;
    }
}

void receive(byte[] data) {
    String msg = "";
    
    for (int i = 0; i < data.length; i++)
        msg += char(data[i]);
    
    if (msg.equals("1")) { 
        ack = true;
        ackTime = millis();
    }
      
    if (msg.equals("0")) { 
        i = 0; 
        ack = true;
        ackTime = millis();
    }
      
    println(msg);
    println();
    
    // Send ACK. TODO: for debugging only
    byte[] message = new byte[1];
    message[0] = (byte) 255;
    udp.send(message, ip, port);
}
