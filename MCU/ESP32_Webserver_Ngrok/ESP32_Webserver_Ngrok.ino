//
// Access ESP32 Web server from anywhere in the world
//
// ESP32 run this webserver example
// PC run ngrok 
// ngrok tcp 192.168.168.25:80 --authtoken 28kTBnGwdaeYEjbPvz8HiixtXcO_55yGVE4Jhdy459nEK1dE5
#include <WiFi.h>

const char* ssid = "HITRON-BD50";  
const char* password = "#####";

WiFiServer server(80);
String header;


void setup() {
  Serial.begin(115200);
  
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients
  
    if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) 
    {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            client.println("<!DOCTYPE html>\n");
            client.println("<html>\n");
            client.println("<body>\n");
            client.println("<center>\n");
            client.println("<h1 style=\"color:blue;\">ESP32 webserver</h1>\n");
            client.println("<h2 style=\"color:green;\">Hello World Web Sever</h2>\n");
            client.println("<h2 style=\"color:blue;\">by 00953150 2022/5/9</h2>\n");
            client.println("</center>\n");
            client.println("</body>\n");
            client.println("</html>");
            break;                       
          } 
          else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
