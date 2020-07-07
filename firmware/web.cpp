/*---------------------------------------------------------------------------------
 OTA code uploading (over the air/wifi update)
 
 Credit: The original code is from the link below:
 https://lastminuteengineers.com/esp32-ota-updates-arduino-ide/
---------------------------------------------------------------------------------*/
#include <ArduinoOTA.h>
#include <SPIFFS.h>                 // SPI File system
#include <Update.h>
//#include <WiFi.h>
//#include <WiFiClient.h>
//#include <WebServer.h>
//#include <ESPmDNS.h>              //This would cause power consumption increase 

#include "firmware.h"

extern String   loginIndex;
extern String   serverIndex;
/*---------------------------------------------------------------------------------
  WiFi router configuration

  wifi_id.h only contain the lines blow, and is not tracked by Git.
  const char* wifi2_4G_ssid = " ";    // ESP32 only support 2.4GHz wifi, not 5G Hz
  const char* wifi_password = "";
---------------------------------------------------------------------------------*/
#include "wifi_id.h"
#define HOST_NAME  "homeicu"
/*---------------------------------------------------------------------------------
  Basic OTA (on-the-air) function

  This function enables ArduinoIDE update ESP32 board through WiFi network.
  Three ways to upload code by this feature:
  1. Arduino IDE, change port to localIP
  2. VSCode, change port
  3. ota.py
  Please reference to README.md at "# Upload Binary to board"
---------------------------------------------------------------------------------*/
void initBasicOTA(void) 
{
  WiFi.mode(WIFI_STA);

  WiFi.begin(wifi2_4G_ssid, wifi_password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(4000);
    ESP.restart();
  }

  //initialize SPI file system
  if(!SPIFFS.begin(true)) 
  {
    Serial.println("SPIFFS Error (it is ok for the brand new board's the first time bootup)");
    system_init_error++;
  }
  else
    Serial.println("SPIFFS OK");

  // Hostname 
  ArduinoOTA.setHostname(HOST_NAME);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() 
  {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
    {
      type = "filesystem";
      SPIFFS.end();
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  Serial.print("OTA IP: ");
  Serial.println(WiFi.localIP());
}
/*---------------------------------------------------------------------------------
  Web Update function

  This function enables ESP32 board update by web browser through WiFi network.
---------------------------------------------------------------------------------*/
void stopWifi(void)
{
  WiFi.disconnect();
  while(WiFi.status() != WL_CONNECTED)
    delay(100);
  Serial.println("WiFi is disconnected.");
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi is OFF.");
}
/*---------------------------------------------------------------------------------
 web uploading

 https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/

 User ID: admin   Password: password

 it is hard coded in the string like below:
 "if(form.userid.value=='admin' && form.pwd.value=='password')" 

 This feature is disabled in the real product, no security risk.
---------------------------------------------------------------------------------*/
#if WEB_UPDATE

const char *host = HOST_NAME;       // host name of web server, this name will be used by ota.py and usb.py
WebServer server(80);               // 80 is the port number
 
void handleWebClient(void) 
{
  server.handleClient();
  delay(1);
}
void setupWebServer(void) 
{
  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) {  //FIXME this would cause increase power consumption
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
}
/*---------------------------------------------------------------------------------
 webpage html data
---------------------------------------------------------------------------------*/

/* Style */
String style =
"<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
"input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
"#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
"#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
"form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
".btn{background:#3498db;color:#fff;cursor:pointer}</style>";
 
/* Login page */
String loginIndex = 
"<form name=loginForm>"
"<h1>HomeICU Upload</h1>"
"<input name=userid placeholder='User ID'> "
"<input name=pwd placeholder=Password type=Password> "
"<input type=submit onclick=check(this.form) class=btn value=Login></form>"
"<script>"
"function check(form) {"
"if(form.userid.value=='admin' && form.pwd.value=='password')"  //default username and passoword
"{window.open('/serverIndex')}"
"else"
"{alert('Error Password or Username')}"
"}"
"</script>" + style;
 
/* Server Index Page */
String serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
"<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
"<label id='file-input' for='file'>   Choose file...</label>"
"<input type='submit' class=btn value='Update'>"
"<br><br>"
"<div id='prg'></div>"
"<br><div id='prgbar'><div id='bar'></div></div><br></form>"
"<script>"
"function sub(obj){"
"var fileName = obj.value.split('\\\\');"
"document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
"};"
"$('form').submit(function(e){"
"e.preventDefault();"
"var form = $('#upload_form')[0];"
"var data = new FormData(form);"
"$.ajax({"
"url: '/update',"
"type: 'POST',"
"data: data,"
"contentType: false,"
"processData:false,"
"xhr: function() {"
"var xhr = new window.XMLHttpRequest();"
"xhr.upload.addEventListener('progress', function(evt) {"
"if (evt.lengthComputable) {"
"var per = evt.loaded / evt.total;"
"$('#prg').html('progress: ' + Math.round(per*100) + '%');"
"$('#bar').css('width',Math.round(per*100) + '%');"
"}"
"}, false);"
"return xhr;"
"},"
"success:function(d, s) {"
"console.log('success!') "
"},"
"error: function (a, b, c) {"
"}"
"});"
"});"
"</script>" + style;

#endif //WEB_UPDATE

/*
Future feature: 
This feature enable ESP32 receive wifi passoword from phone before initialization.
It turn WiFi to the AP mode, and wait phone use an app to connect it.
The problem is that ESP32 only use 2.4G Hz, might confuse consumer.
It is better not to turn this feature on.

void setupSmartConfig() {
  //init WiFi as Station, start SmartConfig
  WiFi.mode(WIFI_AP_STA);
  WiFi.beginSmartConfig();

  //Wait for SmartConfig packet from mobile
  while (!WiFi.smartConfigDone()) {
    delay(500);
    Serial.print(".");
  }

  //Wait for WiFi to connect to AP
  Serial.println("Waiting for WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}
*/