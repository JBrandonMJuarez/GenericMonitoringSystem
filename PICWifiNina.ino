#include <WiFiNINA.h>

char ssid[] = "";             //  your network SSID (name) between the " "
char pass[] = "";      // your network password between the " "
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;      //connection status   
WiFiServer server(80);
WiFiClient client;

int ledPin = LED_BUILTIN;

String mensajeUART = "";  // Último mensaje UART recibido completo

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  enable_WiFi();
  connect_WiFi();
  server.begin();
  printWifiStatus();
}

void loop() {
  // Leer todo lo disponible por UART (si hay)
  if (Serial.available()) {
    mensajeUART = Serial.readString();  // Lee todo el buffer UART
    mensajeUART.trim();                  // Elimina espacios y saltos al inicio y final
  }

  WiFiClient client = server.available();
  if (client) {
    String currentLine = "";
    String request = "";
    bool mensajeHTMLProcesado = false;

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        request += c;

        if (c == '\n' && currentLine.length() == 0) {
          // Procesar petición GET /?mensaje=...
          if (request.indexOf("GET /?mensaje=") >= 0 && !mensajeHTMLProcesado) {
            int start = request.indexOf("GET /?mensaje=") + strlen("GET /?mensaje=");
            int end = request.indexOf(' ', start);
            String mensajeRecibidoHTML = request.substring(start, end);
            mensajeRecibidoHTML.replace("+", " ");
            mensajeRecibidoHTML.trim();
            Serial.println(mensajeRecibidoHTML);  // Enviar mensaje recibido por HTML al UART
            mensajeUART = Serial.readString();  // Lee todo el buffer UART
            mensajeHTMLProcesado = true;
          }

          if (request.indexOf("GET /H") >= 0) digitalWrite(ledPin, HIGH);
          if (request.indexOf("GET /L") >= 0) digitalWrite(ledPin, LOW);

          // Enviar respuesta HTTP y página HTML
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.println("<!DOCTYPE html><html>");
          client.println("<head><title>Servidor WIFI NINA - UART</title></head><body>");
          client.println("<h3>Control LED Modulo WIFI</h3>");
          client.println("<a href=\"/H\">Encender</a><br>");
          client.println("<a href=\"/L\">Apagar</a><br><br>");

          client.println("<h3>Enviar comando UART</h3>");
          client.println("<form action=\"/\" method=\"GET\">");
          client.println("Ingresar comando: <input type=\"text\" name=\"mensaje\">");
          client.println("<input type=\"submit\" value=\"Enviar\">");
          client.println("</form><br>");

          // Mostrar mensaje UART con saltos de línea como <br>
          client.println("<h3>Respuesta UART PIC18F4550:</h3>");
          String mensajeHTML = mensajeUART;
          mensajeHTML.replace("\n", "<br>");
          mensajeHTML.replace("\r", ""); // Quitar retornos de carro
          client.println(mensajeHTML);

          client.println("</body></html>");
          client.println();
          break;
        }

        if (c != '\r') {
          currentLine += c;
        } else {
          currentLine = "";
        }
      }
    }
    delay(5);
    client.stop();
  }
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.print("Para ver la página, abre el navegador en: http://");
  Serial.println(ip);
}

void enable_WiFi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Fallo comunicación con módulo WiFi");
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Por favor actualiza el firmware");
  }
}

void connect_WiFi() {
  while (status != WL_CONNECTED) {
    Serial.print("Conectando a SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
}


