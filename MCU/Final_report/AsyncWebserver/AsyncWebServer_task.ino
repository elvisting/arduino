/*****************************************************
 *
 * AsycWebServer_func
 *
 * task นี้จะทำหน้าที่ สร้าง WebServer แบบ Asynchronous
 * ทาง WebServer ไม่จำเป็นต้องรอให้ client นึงๆที่ติดต่อเข้ามา
 * เรียกทำงานจนจบกระบวนการ ค่อยให้ client ถัดไปใช้งาน
 * ทำให้การทำงานแบบ Multitask มีความไหลลื่นกว่าปกติ
 *
 * หมายเหตุ
 * ภายใน task ให้ใช้ DELAY() ตัวพิมพ์ใหญ่ทั้งหมด
 * ไลบรารี่ สร้าง WebServer แบบ Asynchronous ให้ติดตั้งไลบรารี่ทั้ง 2 นี้
 *    https://github.com/me-no-dev/AsyncTCP
 *    https://github.com/me-no-dev/ESPAsyncWebServer
 *
 ****************************************************/

#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

void AsycWebServer_func(void*) {
  //----พื้นที่สำหรับประกาศตัวแปรที่ใช้ภายใน task นี้เท่านั้น----

  //-----------------------------------------------
  VOID SETUP() {                       // VOID SETUP() ใน task ใช้พิมพ์ใหญ่
    server.on("/heap", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", String(ESP.getFreeHeap()));
    });
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      char buf[50];
      sprintf(buf, "<H2>Temp  : %.2f<br>\nHumid : %.2f<br></H2>\n", xHTU_temp, xHTU_humid);      
      request->send(200, "text/html", String(buf));
    });
    
    server.onNotFound([](AsyncWebServerRequest * request) {
      Serial.println("[AsyncWebServer] Page NOT FOUND ");
      request->send(404, "text/plain", "Page not found");
    });

    Serial.println("[AsyncWebServer] start!");
    Serial.printf ("[AsyncWebServer] Open http://%s on the browser.\n", WiFi.localIP().toString().c_str());
    server.begin();
  }

  VOID LOOP() {                        // VOID LOOP() ใน task ใช้พิมพ์ใหญ่
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
            client.println("<h1 style=\"color:black;\">ESP32 webserver</h1>\n");
            client.println("<h2 style=\"color:Teal;\">Humidity:%.2f</h2>\n", xHTU_humid);
            client.println("<h3 style=\"color:Fuchsia;\">Temperature:%.2f</h3>\n", xHTU_temp);
            client.println("<h4 style=\"color:red;\">Window state:</h4>\n");
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
      DELAY(1);                          // DELAY(..) ใน task ใช้พิมพ์ใหญ่
  }
}
