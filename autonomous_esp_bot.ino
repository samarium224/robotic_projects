#include <Arduino.h>
#include <Servo.h>
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>

enum Mode
{
  REMOTE,
  AUTO
};
Mode currentMode = REMOTE;

#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define UP_LEFT 5
#define UP_RIGHT 6
#define DOWN_LEFT 7
#define DOWN_RIGHT 8
#define TURN_LEFT 9
#define TURN_RIGHT 10
#define AUTO_MODE 11
#define REMOTE_MODE 12
#define STOP 0

#define FRONT_RIGHT_MOTOR 0
#define BACK_RIGHT_MOTOR 1
#define FRONT_LEFT_MOTOR 2
#define BACK_LEFT_MOTOR 3
#define FRONT_MOTOR_SPEED_PIN 32
#define BACK_MOTOR_SPEED_PIN 34

#define FORWARD 1
#define BACKWARD -1

// for ultrasonic sensor
#define TRIGGER_PIN 12
#define ECHO_PIN 13
float duration, distance;

// for servo
#define SERVO_PIN 14

struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;
};

struct ObstacleScan
{
  int distancemap[3]; // 0 = front, 1 = left, 2 = right
  int ObstacleMap[3]; // 1 = obstacle, 0 = free
};

Servo Robotservo;

std::vector<MOTOR_PINS> motorPins =
    {
        {25, 33}, // FRONT_RIGHT_MOTOR
        {27, 26}, // FRONT_LEFT_MOTOR
        {18, 19}, // BACK_RIGHT_MOTOR
        {16, 17}, // BACK_LEFT_MOTOR
};

const char *ssid = "MyWiFiCar";
const char *password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char *htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>
  <head>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
    <style>
    .arrows {
      font-size:70px;
      color:red;
    }
    .circularArrows {
      font-size:80px;
      color:blue;
    }
    td {
      background-color:black;
      border-radius:25%;
      box-shadow: 5px 5px #888888;
    }
    td:active {
      transform: translate(5px,5px);
      box-shadow: none; 
    }

    .noselect {
      -webkit-touch-callout: none; /* iOS Safari */
        -webkit-user-select: none; /* Safari */
         -khtml-user-select: none; /* Konqueror HTML */
           -moz-user-select: none; /* Firefox */
            -ms-user-select: none; /* Internet Explorer/Edge */
                user-select: none; /* Non-prefixed version, currently
                                      supported by Chrome and Opera */
    }
    </style>
  </head>
  <body class="noselect" align="center" style="background-color:white">
     
    <h1 style="color: teal;text-align:center;">Hash Include Electronics</h1>
    <h2 style="color: teal;text-align:center;">Wi-Fi &#128663; Control</h2>
    
    <table id="mainTable" style="width:400px;margin:auto;table-layout:fixed" CELLSPACING=10>
      <tr>
        <td ontouchstart='onTouchStartAndEnd("5")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#11017;</span></td>
        <td ontouchstart='onTouchStartAndEnd("1")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#8679;</span></td>
        <td ontouchstart='onTouchStartAndEnd("6")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#11016;</span></td>
      </tr>
      
      <tr>
        <td ontouchstart='onTouchStartAndEnd("3")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#8678;</span></td>
        <td></td>    
        <td ontouchstart='onTouchStartAndEnd("4")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#8680;</span></td>
      </tr>
      
      <tr>
        <td ontouchstart='onTouchStartAndEnd("7")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#11019;</span></td>
        <td ontouchstart='onTouchStartAndEnd("2")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#8681;</span></td>
        <td ontouchstart='onTouchStartAndEnd("8")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#11018;</span></td>
      </tr>
    
      <tr>
        <td ontouchstart='onTouchStartAndEnd("9")' ontouchend='onTouchStartAndEnd("0")'><span class="circularArrows" >&#8634;</span></td>
        <td style="background-color:white;box-shadow:none"></td>
        <td ontouchstart='onTouchStartAndEnd("10")' ontouchend='onTouchStartAndEnd("0")'><span class="circularArrows" >&#8635;</span></td>
      </tr>
    </table>

    <script>
      var webSocketUrl = "ws:\/\/" + window.location.hostname + "/ws";
      var websocket;
      
      function initWebSocket() 
      {
        websocket = new WebSocket(webSocketUrl);
        websocket.onopen    = function(event){};
        websocket.onclose   = function(event){setTimeout(initWebSocket, 2000);};
        websocket.onmessage = function(event){};
      }

      function onTouchStartAndEnd(value) 
      {
        websocket.send(value);
      }
          
      window.onload = initWebSocket;
      document.getElementById("mainTable").addEventListener("touchend", function(event){
        event.preventDefault()
      });      
    </script>
    
  </body>
</html> 

)HTMLHOMEPAGE";

void rotateMotor(int motorNumber, int motorDirection)
{
  if (motorDirection == FORWARD)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
    analogWrite(FRONT_MOTOR_SPEED_PIN, 100);
    analogWrite(BACK_MOTOR_SPEED_PIN, 100);
  }
  else if (motorDirection == BACKWARD)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);
    analogWrite(FRONT_MOTOR_SPEED_PIN, 100);
    analogWrite(BACK_MOTOR_SPEED_PIN, 100);
  }
  else
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
    analogWrite(FRONT_MOTOR_SPEED_PIN, 0);
    analogWrite(BACK_MOTOR_SPEED_PIN, 0);
  }
}

void processCarMovement(String inputValue)
{
  Serial.printf("Got value as %s %d\n", inputValue.c_str(), inputValue.toInt());
  switch (inputValue.toInt())
  {

  case UP:
    rotateMotor(FRONT_RIGHT_MOTOR, FORWARD);
    rotateMotor(BACK_RIGHT_MOTOR, FORWARD);
    rotateMotor(FRONT_LEFT_MOTOR, FORWARD);
    rotateMotor(BACK_LEFT_MOTOR, FORWARD);
    break;

  case DOWN:
    rotateMotor(FRONT_RIGHT_MOTOR, BACKWARD);
    rotateMotor(BACK_RIGHT_MOTOR, BACKWARD);
    rotateMotor(FRONT_LEFT_MOTOR, BACKWARD);
    rotateMotor(BACK_LEFT_MOTOR, BACKWARD);
    break;

  case LEFT:
    rotateMotor(FRONT_RIGHT_MOTOR, FORWARD);
    rotateMotor(BACK_RIGHT_MOTOR, BACKWARD);
    rotateMotor(FRONT_LEFT_MOTOR, BACKWARD);
    rotateMotor(BACK_LEFT_MOTOR, FORWARD);
    break;

  case RIGHT:
    rotateMotor(FRONT_RIGHT_MOTOR, BACKWARD);
    rotateMotor(BACK_RIGHT_MOTOR, FORWARD);
    rotateMotor(FRONT_LEFT_MOTOR, FORWARD);
    rotateMotor(BACK_LEFT_MOTOR, BACKWARD);
    break;

  case UP_LEFT:
    rotateMotor(FRONT_RIGHT_MOTOR, FORWARD);
    rotateMotor(BACK_RIGHT_MOTOR, STOP);
    rotateMotor(FRONT_LEFT_MOTOR, STOP);
    rotateMotor(BACK_LEFT_MOTOR, FORWARD);
    break;

  case UP_RIGHT:
    rotateMotor(FRONT_RIGHT_MOTOR, STOP);
    rotateMotor(BACK_RIGHT_MOTOR, FORWARD);
    rotateMotor(FRONT_LEFT_MOTOR, FORWARD);
    rotateMotor(BACK_LEFT_MOTOR, STOP);
    break;

  case DOWN_LEFT:
    rotateMotor(FRONT_RIGHT_MOTOR, STOP);
    rotateMotor(BACK_RIGHT_MOTOR, BACKWARD);
    rotateMotor(FRONT_LEFT_MOTOR, BACKWARD);
    rotateMotor(BACK_LEFT_MOTOR, STOP);
    break;

  case DOWN_RIGHT:
    rotateMotor(FRONT_RIGHT_MOTOR, BACKWARD);
    rotateMotor(BACK_RIGHT_MOTOR, STOP);
    rotateMotor(FRONT_LEFT_MOTOR, STOP);
    rotateMotor(BACK_LEFT_MOTOR, BACKWARD);
    break;

  case TURN_LEFT:
    rotateMotor(FRONT_RIGHT_MOTOR, FORWARD);
    rotateMotor(BACK_RIGHT_MOTOR, FORWARD);
    rotateMotor(FRONT_LEFT_MOTOR, BACKWARD);
    rotateMotor(BACK_LEFT_MOTOR, BACKWARD);
    break;

  case TURN_RIGHT:
    rotateMotor(FRONT_RIGHT_MOTOR, BACKWARD);
    rotateMotor(BACK_RIGHT_MOTOR, BACKWARD);
    rotateMotor(FRONT_LEFT_MOTOR, FORWARD);
    rotateMotor(BACK_LEFT_MOTOR, FORWARD);
    break;

  case STOP:
    rotateMotor(FRONT_RIGHT_MOTOR, STOP);
    rotateMotor(BACK_RIGHT_MOTOR, STOP);
    rotateMotor(FRONT_LEFT_MOTOR, STOP);
    rotateMotor(BACK_LEFT_MOTOR, STOP);
    break;

  case AUTO_MODE:
    currentMode = AUTO;
    Serial.println("Switched to AUTO mode");
    rotateMotor(FRONT_RIGHT_MOTOR, STOP);
    rotateMotor(BACK_RIGHT_MOTOR, STOP);
    rotateMotor(FRONT_LEFT_MOTOR, STOP);
    rotateMotor(BACK_LEFT_MOTOR, STOP);
    break;

  case REMOTE_MODE:
    currentMode = REMOTE;
    Serial.println("Switched to REMOTE mode");
    rotateMotor(FRONT_RIGHT_MOTOR, STOP);
    rotateMotor(BACK_RIGHT_MOTOR, STOP);
    rotateMotor(FRONT_LEFT_MOTOR, STOP);
    rotateMotor(BACK_LEFT_MOTOR, STOP);
    break;

  default:
    rotateMotor(FRONT_RIGHT_MOTOR, STOP);
    rotateMotor(BACK_RIGHT_MOTOR, STOP);
    rotateMotor(FRONT_LEFT_MOTOR, STOP);
    rotateMotor(BACK_LEFT_MOTOR, STOP);
    break;
  }
}

void handleRoot(AsyncWebServerRequest *request)
{
  request->send_P(200, "text/html", htmlHomePage);
}

void handleNotFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "File Not Found");
}

void onWebSocketEvent(AsyncWebSocket *server,
                      AsyncWebSocketClient *client,
                      AwsEventType type,
                      void *arg,
                      uint8_t *data,
                      size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    // client->text(getRelayPinsStatusJson(ALL_RELAY_PINS_INDEX));
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    processCarMovement("0");
    break;
  case WS_EVT_DATA:
    if (currentMode == REMOTE)
    {
      AwsFrameInfo *info;
      info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
      {
        std::string myData = "";
        myData.assign((char *)data, len);
        processCarMovement(myData.c_str());
      }
    }
    break;

  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  default:
    break;
  }
}

void setUpPinModes()
{
  for (int i = 0; i < motorPins.size(); i++)
  {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);
    rotateMotor(i, STOP);
  }

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Robotservo.attach(SERVO_PIN);
  Robotservo.write(90); // Set servo to neutral position
}

long measureDistance()
{
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 20000); // 20ms timeout
  long distance = duration * 0.034 / 2;           // cm
  return distance;
}

ObstacleScan isObstacleAhead()
{
  ObstacleScan scanResult;

  // ---- Check Left ----
  Robotservo.write(45);
  delay(100);
  scanResult.distancemap[1] = measureDistance();
  scanResult.ObstacleMap[1] = (scanResult.distancemap[1] < 20) ? 1 : 0;

  // ---- Check Front ----
  Robotservo.write(90);
  delay(100);
  scanResult.distancemap[0] = measureDistance();
  scanResult.ObstacleMap[0] = (scanResult.distancemap[0] < 20) ? 1 : 0;

  // ---- Check Right ----
  Robotservo.write(135);
  delay(100);
  scanResult.distancemap[2] = measureDistance();
  scanResult.ObstacleMap[2] = (scanResult.distancemap[2] < 20) ? 1 : 0;

  return scanResult;
}

void runAutonomous()
{
  ObstacleScan scan = isObstacleAhead();

  // Example: print results
  Serial.printf("Front: %dcm | Left: %dcm | Right: %dcm\n",
                scan.distancemap[0],
                scan.distancemap[1],
                scan.distancemap[2]);

  if (scan.ObstacleMap[0] == 1)
  {
    // Obstacle ahead → move back + turn
    processCarMovement("2"); // backward
    delay(500);
    processCarMovement("0");

    if (scan.distancemap[1] > scan.distancemap[2])
    {
      processCarMovement("9"); // turn left
      delay(400);
    }
    else
    {
      processCarMovement("10"); // turn right
      delay(400);
    }
    processCarMovement("0");
  }
  else
  {
    // No obstacle ahead → go forward
    processCarMovement("1");
  }
}

void setup(void)
{
  setUpPinModes();
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  server.begin();
  Serial.println("HTTP server started");
}

void loop()
{
  ws.cleanupClients();

  if (currentMode == AUTO)
  {
    runAutonomous();
  }
}