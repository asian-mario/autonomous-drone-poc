#include <WiFi.h>
#include <WebServer.h>

// WiFi Credentials
const char* ssid = "DroneControlNetwork";
const char* password = "password123";

// Web server setup
WebServer server(80);

// Serial Communication Setup (ESP32 <-> Arduino Mega)
#define RXD2 16
#define TXD2 17

String serialOutput = ""; // Stores logs from Arduino Mega

void handleRoot();
void handleHoverToggle();
void handleKillSwitch();
void handleUpdatePID();
void handleUpdateRollTrim();
void handleGetLogs();

void setup() {
  Serial.begin(57600);
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);  // Use Serial2 for Mega communication

  WiFi.softAP(ssid, password);
  Serial.println("WiFi Access Point started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Define Web Server routes
  server.on("/", handleRoot);
  server.on("/hover", handleHoverToggle);
  server.on("/kill", handleKillSwitch);
  server.on("/updatePID", handleUpdatePID);
  server.on("/updateRollTrim", handleUpdateRollTrim);  // New route for roll trim update
  server.on("/getLogs", handleGetLogs);

  server.begin();
}

void loop() {
  server.handleClient();

  // Read Serial data from Arduino Mega and store it for the web app
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {  // Full message received
      Serial.println("Received from Mega: " + serialOutput);  // Debugging output
      serialOutput += "\n";  // Ensure new line for web display
    } else {
      serialOutput += c;
    }
    // Prevent buffer overflow (keep last 2000 characters)
    if (serialOutput.length() > 2000) {
      serialOutput = serialOutput.substring(serialOutput.length() - 2000);
    }
    delay(2);  // Small delay to allow processing
  }
}

// HTML Web Interface
void handleRoot() {
  String html = "<html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<script>";
  html += "function updateLogs() {";
  html += "fetch('/getLogs').then(response => response.text()).then(data => {";
  html += "document.getElementById('logs').innerHTML = data.replace(/\\n/g, '<br>');";
  html += "});";
  html += "}";
  html += "setInterval(updateLogs, 1000);"; // Refresh logs every second
  html += "</script>";
  html += "</head><body>";
  html += "<h1>Drone Control Panel</h1>";
  html += "<button onclick=\"fetch('/hover')\">Toggle Hover Mode</button>";
  html += "<button onclick=\"fetch('/kill')\">Kill Switch</button>";

  html += "<h2>PID Tuning</h2>";
  html += "<form action='/updatePID' method='GET'>";
  html += "Kp: <input type='number' step='0.01' name='kp' value='1.2'><br>";
  html += "Ki: <input type='number' step='0.01' name='ki' value='0.07'><br>";
  html += "Kd: <input type='number' step='0.01' name='kd' value='0.5'><br>";
  html += "<input type='submit' value='Update PID'>";
  html += "</form>";

  html += "<h2>Roll Trim Adjustment</h2>";
  html += "<form action='/updateRollTrim' method='GET'>";
  html += "Roll Trim: <input type='number' step='0.1' name='rolltrim' value='55'><br>";
  html += "<input type='submit' value='Update Roll Trim'>";
  html += "</form>";

  html += "<h2>Arduino Mega Logs</h2>";
  html += "<pre id='logs'>Waiting for logs...</pre>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

bool hoverState = false;  // Tracks current hover state

void handleHoverToggle() {
  hoverState = !hoverState;  // Toggle the state
  if (hoverState) {
    Serial2.println("HOVER_ON");
    server.send(200, "text/plain", "Hover mode activated");
  } else {
    Serial2.println("HOVER_OFF");
    server.send(200, "text/plain", "Hover mode deactivated");
  }
}

// Handles Kill Switch
void handleKillSwitch() {
  Serial2.println("KILL");
  server.send(200, "text/plain", "Kill switch activated");
}

// Handles PID Updates
void handleUpdatePID() {
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
    String command = "PID," + server.arg("kp") + "," + server.arg("ki") + "," + server.arg("kd");
    Serial2.println(command);
    server.send(200, "text/plain", "PID updated");
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

// Handles Roll Trim Updates
void handleUpdateRollTrim() {
  if (server.hasArg("rolltrim")) {
    String command = "ROLLTRIM," + server.arg("rolltrim");
    Serial2.println(command);
    server.send(200, "text/plain", "Roll trim updated: " + server.arg("rolltrim"));
  } else {
    server.send(400, "text/plain", "Missing parameter: rolltrim");
  }
}

// Handles Log Retrieval
void handleGetLogs() {
  server.send(200, "text/plain; charset=utf-8", serialOutput);
}
