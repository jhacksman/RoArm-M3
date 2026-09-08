#include "m3_web_page.h"
#include <ESPAsyncWebServer.h>
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void handleWsEvent(
    AsyncWebSocket *server,
    AsyncWebSocketClient *client,
    AwsEventType type,
    void *arg,
    uint8_t *data,
    size_t len)
{

}

void handleRoot(AsyncWebServerRequest *request){
  request->send_P(200, "text/html; charset=utf-8", index_html); //Send web page
}

void handleHorizontalDrag(AsyncWebServerRequest *request){
   request->send_P(200, "text/html", horizontal_drag_html); //Send web page
}

void handleVerticalDrag(AsyncWebServerRequest *request){
   request->send_P(200, "text/html", vertical_drag_html); //Send web page
}

void webCtrlServer(){
  server.on("/", HTTP_GET,handleRoot);
  server.on("/horiDrag",HTTP_GET, handleHorizontalDrag);
  server.on("/vertDrag",HTTP_GET, handleVerticalDrag);


    server.on("/js", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("json")) {
      String jsonCmdWebString = request->getParam("json")->value();
        // 停止命令：直接清空队列
    if (jsonCmdWebString.indexOf("\"T\":0") != -1) {
        StopFlag=true;
        char tmp[CMD_MAX_LEN];
        while (xQueueReceive(cmdQueue, tmp, 0) == pdPASS) {}
    }else if(jsonCmdWebString.indexOf("\"T\":999") != -1)
    { StopFlag=false;
    }
    else {
        char cmdBuffer[CMD_MAX_LEN];
        jsonCmdWebString.toCharArray(cmdBuffer, CMD_MAX_LEN);

        if (xQueueSend(cmdQueue, cmdBuffer, 0) != pdPASS) {
            request->send(200, "application/json",
                          "{\"error\":\"Queue full\"}");
            return;
        }
    }
}
  // 序列化 JSON 并返回
    //serializeJson(jsonInfoHttp, jsonFeedbackWeb);
    //request->send(200, "application/json", jsonFeedbackWeb);
    request->send(200, "application/json","{\"ok\":1}");
    // 清理
    //jsonFeedbackWeb = "";
  });
// WebSocket
  ws.onEvent(handleWsEvent);
  server.addHandler(&ws);
  // Start server
  server.begin();
  Serial.println("Server Starts.");
    
}

void initHttpWebServer(){
  webCtrlServer();
}

void pushTelemetry() {
  if (millis() - lastTick >= FB_INTERVAL_MS) {
    lastTick = millis();
    if (ws.count() > 0) {
      jsonFeedback.clear();
      jsonFeedback["T"]   = 50;
      jsonFeedback["sta"]   =" IP:"+ localIP.toString();

      if (WiFi.status() == WL_CONNECTED) {
        jsonFeedback["ssid"] = WiFi.SSID();
      } else {
        jsonFeedback["ssid"] = "Disconnected";
        jsonFeedback["sta"]   ="";
      }
     // jsonFeedback["uptime"] = (uint32_t)(millis()/1000);
      serializeJson(jsonFeedback, outputString);
      ws.textAll(outputString);
    }
  }
    if (millis() - lastTick_pos >= FB_INTERVAL_MS) {
    lastTick_pos = millis();
    if (ws.count() > 0) {
      jsonFeedback.clear();
      jsonFeedback["T"]   = -15;
      jsonFeedback["x"] = lastX;
      jsonFeedback["y"] = lastY;
      jsonFeedback["z"] = lastZ;
      jsonFeedback["tit"] = lastT;

      jsonFeedback["b"] = radB;
      jsonFeedback["s"] = radS;
      jsonFeedback["e"] = radE;
      jsonFeedback["t"] = radT;
      jsonFeedback["r"] = radR;
      jsonFeedback["g"] = radG;
      jsonFeedback["Stalltor"] = Stall_flag;
      jsonFeedback["Stalltep"] = Temp_flag;
      jsonFeedback["Stallvol"] = Voltage_flag;

      serializeJson(jsonFeedback, outputString);
      ws.textAll(outputString);
    }
  }
  
}

