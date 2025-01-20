//@author Ömer & Oscar & Firoz
#include "painlessMesh.h" // Hanterar mesh nätverket mellan enheter
#include <ArduinoJson.h> // Används för att JSON meddelande typ kommer skickas och ta emot
#include <map> //bibliotek för att använda std::map

//konstanter för Mesh nätverket
#define MESH_PREFIX "FireInTheHole" //namn
#define MESH_PASSWORD "password123" //lösenord
#define MESH_PORT 5555 // port som används

Scheduler userScheduler; //schemaläggare för tasks
painlessMesh mesh; // mesh objecktet för att hantera nätverket

// funktioner för att simulera olika händelser som man får från GUI
void simulateFireAtPosition(int x, int y);
void simulatePOIAtPosition(int x, int y);
void simulateFlammableMaterialAtPosition(int x, int y);

// Task to read from Serial (from GUI)
Task readSerialTask(100, TASK_FOREVER, []() {
    while (Serial.available()) {
        // läs hela strängen fram till radbryttning 
        String msg = Serial.readStringUntil('\n');
        msg.trim(); // tar emot eventualla mellanslag i början

        if (msg.length() == 0) continue;

        Serial.println("Received from GUI: " + msg);

        // Parsar meddelandet beroende på prefix
        if (msg.startsWith("FIRE:")) {
            // Message format: "FIRE:x,y"
            String posStr = msg.substring(5); // allt efter FIRE:
            int commaIndex = posStr.indexOf(',');
            if (commaIndex > 0) {
                int x = posStr.substring(0, commaIndex).toInt();
                int y = posStr.substring(commaIndex + 1).toInt();
                simulateFireAtPosition(x, y);
            }
        } else if (msg.startsWith("POI:")) {
            // Message format: "POI:x,y"
            String posStr = msg.substring(4); //Allt efter POI:
            int commaIndex = posStr.indexOf(',');
            if (commaIndex > 0) {
                int x = posStr.substring(0, commaIndex).toInt();
                int y = posStr.substring(commaIndex + 1).toInt();
                simulatePOIAtPosition(x, y);
            }
        } else if (msg.startsWith("FLAMMABLE:")) {
            // Message format: "FLAMMABLE:x,y"
            String posStr = msg.substring(10); //Allt efter Flammable:
            int commaIndex = posStr.indexOf(',');
            if (commaIndex > 0) {
                int x = posStr.substring(0, commaIndex).toInt();
                int y = posStr.substring(commaIndex + 1).toInt();
                simulateFlammableMaterialAtPosition(x, y);
            }
        }
    }
});

// Här hämtar vi en lista på alla noder i nätverket och skickar vidare till GUI 
Task sendDevicesListTask(5000, TASK_FOREVER, []() {
    String deviceList = "Devices:";
    std::list<uint32_t> nodes = mesh.getNodeList(); // hämtar noderna
    bool first = true;
    // Brygger upp en sträng med nodernas ID
    for (auto nodeId : nodes) {
      if (first) {
          deviceList += " " + String(nodeId);
          first = false;
      } else {
          deviceList += "," + String(nodeId);
      }
    }
    // Skriver ut t.ex. "Devices: 1234567,2345678,..." i serie-monitorn
    Serial.println(deviceList);
});

// Callback when a message is received from the mesh network
void receivedCallback(uint32_t from, String &msg) {
    // För debugging skriver vi på en separat rad
    Serial.print("DEBUG: Message received from ");
    Serial.println(from);

    // Här försöker vi avkoda JSON från kommande meddelander

    StaticJsonDocument<1024> doc; // Ömer: Här ökade jag storleken för storleken var inte tillräcklint
    DeserializationError error = deserializeJson(doc, msg);
    if (!error) {

      // Skriver ut endast ren JSON på en egen rad
      Serial.println(msg);

    } else {
      // om avkodningen misslyckas skriv ut felet
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
    }
    // här läser vi ut type för att förstå vilken sorts meddelande det är
    String type = doc["type"];

    /****************************************************
     * Hantering av olika typer av meddelanden
     ****************************************************/

     //1. position_update
    if (type == "position_update") {
        uint32_t nodeId = doc["node_id"];
        int x = doc["position"]["x"];
        int y = doc["position"]["y"];
        // här bygger vi ett enkelt strängformat som POS:nodeid, x, y
        String posMsg = "POS:" + String(nodeId) + "," + String(x) + "," + String(y);
        Serial.println(posMsg);
        
      //2. state_update
    } else if (type == "state_update") {
        uint32_t nodeId = doc["node_id"];
        String state = doc["state"].as<String>();
        String stateMsg = "STATE:" + String(nodeId) + "," + state;
        Serial.println(stateMsg);

        //3. task_complete
    } else if (type == "task_complete") {
        // Hantera task completion
        uint32_t nodeId = doc["node_id"];
        String result = doc["result"];
        int x = doc["location"]["x"];
        int y = doc["location"]["y"];
        // här skriver vi ut att noden slutfört en uppgift
        Serial.printf("Task completed by node %u with result: %s at location (%d, %d)\n", nodeId, result.c_str(), x, y);

        // Skickar vidare informationen om slutförd uppgift till GUI
        String taskCompleteMsg = "TASK_COMPLETE:" + String(nodeId) + "," + result + "," + String(x) + "," + String(y);
        Serial.println(taskCompleteMsg);
    }
    // Ömer: Tidigare else { Serial.println(msg); } är borttaget för att slippa skicka okänt data till GUI
}


/****************************************************
 * Simuleringsfunktioner
 * Dessa skickar ut händelser i nätverket som JSON
 * till alla noder (mesh.sendBroadcast).
 ****************************************************/

// 1.Simulerar vi en brandhändelse
void simulateFireAtPosition(int x, int y) {
    StaticJsonDocument<1024> doc; // Ökad storlek igen
    doc["type"] = "fire_detected";
    uint32_t eventId = millis(); // Use millis as unique event_id
    doc["event_id"] = eventId;
    doc["location"]["x"] = x;
    doc["location"]["y"] = y;
    doc["priority"] = 1; //sätter en prioritet

    // Konverterar till Json och skickar vidare till noder
    String msg;
    serializeJson(doc, msg);
    mesh.sendBroadcast(msg);
    Serial.println("Simulated fire sent with event_id: " + String(eventId));
}

// 2.Simulerar en POI
void simulatePOIAtPosition(int x, int y) {
    StaticJsonDocument<1024> doc; // Ökad storlek igen
    doc["type"] = "poi_spawned";
    uint32_t eventId = millis(); // Use millis as unique event_id
    doc["event_id"] = eventId;
    doc["location"]["x"] = x;
    doc["location"]["y"] = y;

    String msg;
    serializeJson(doc, msg);
    mesh.sendBroadcast(msg);
    Serial.println("Simulated POI sent with event_id: " + String(eventId));
}

// 1.Simulerar vi brandfarligt material
void simulateFlammableMaterialAtPosition(int x, int y) {
    StaticJsonDocument<1024> doc; // Ökad storlek igen
    doc["type"] = "flammable_material_detected";
    uint32_t eventId = millis(); // Use millis as unique event_id
    doc["event_id"] = eventId;
    doc["location"]["x"] = x;
    doc["location"]["y"] = y;

    String msg;
    serializeJson(doc, msg);
    mesh.sendBroadcast(msg);
    Serial.println("Simulated flammable material sent with event_id: " + String(eventId));
}

void setup() {
    Serial.begin(115200);

    // Startar mesh nätverket
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&receivedCallback);

    //Sätter denna nod som root och anger att den kan innehålla root
    mesh.setRoot(true);
    mesh.setContainsRoot(true);

    // lägger till Tasks i schemat
    userScheduler.addTask(readSerialTask);
    readSerialTask.enable();

    userScheduler.addTask(sendDevicesListTask);
    sendDevicesListTask.enable();

    Serial.println("Master node started and acting as a bridge!");
}

void loop() {
    mesh.update();
}






