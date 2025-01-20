// @author Ömer & Oscar & Firoz

#include "painlessMesh.h"  // hanterar mesh nätverket
#include <ArduinoJson.h> // används för att enkelt skapa och tolka json version
#include <map> // map
#include <vector> // vector
#include <queue> // kö
#include <math.h>  // För sqrt
#include <Adafruit_NeoPixel.h> // för att styra Lysdioder
#include <SPI.h> // för att styra TFT skärmen
#include <PNGdec.h>
#include <TFT_eSPI.h>  // för att styra TFT skärmen
#include "horby.h"

#define MESH_PREFIX "FireInTheHole" // namn till nätverket
#define MESH_PASSWORD "password123" // lösenordet
#define MESH_PORT 5555 // porten för att koppla till nätverket


/****************************************************
 *  GPIO / PIN-inställningar
 ****************************************************/

#define yesButton 34
#define helpButton 32
#define noButton 35
#define LED 23
#define NUM_PIXELS 1

PNG png;  
int16_t xpos = 0;
int16_t ypos = 0;
#define MAX_IMAGE_WIDTH 240  


Adafruit_NeoPixel strip(NUM_PIXELS, LED, NEO_GRB + NEO_KHZ800); // styr lysdioden
TFT_eSPI tft = TFT_eSPI();  // styra tft skärmen

const int REQUIRED_HELPERS = 2; // här vi sätter hur många hjälpare som behövs för POI

Scheduler userScheduler; //schemaläggare för tasks
painlessMesh mesh; // mesh objektet för att hantera nätverket

// -----------------------------------------------------------------------------
// GLOBALA VARIABLER FÖR SKILJA "ASSIGNED" VS. "HELPER" I POI-SCENARIO
bool isAssignedPOI = false;  // true om denna nod officiellt blev assigned "poi_spawned"
bool isHelperPOI = false;    // true om denna nod har accepterat att hjälpa
bool helpRequestSent = false; 
// -----------------------------------------------------------------------------
// Struktur för position x och y
struct Position {
  int x;
  int y;
};

uint32_t helpRequestNodeId = 0;  // ID på noden som begär hjälp
uint32_t GateWayNode = 4238560381; // gateway nod ID
Position helpRequestLocation;    // var behövs hjälpen?
bool waitingForAnswer = false;   // väntar vi på en knapptryckning ? 
bool waitingForHelpButtonPress = false;

bool waitingForResponse = false;  // om en nod har inte svarat än
uint8_t helpAcceptedCount = 0;    // hur många som accepterade en hjälp förfrågan
std::vector<uint32_t> nodesAsked; // håller koll på vilka noder vi redan frågat
std::vector<uint32_t> allNodes;   // alla noder i nätet
uint32_t currentTargetNode = 0;   // Id på noden vi just nu ber om hjälp
std::vector<uint32_t> acceptedNodes;  // noder som tackat ja

bool HelpFound = false;
bool AllReady = false; 

int currentTaskPriority = 0;
unsigned long globalEventCounter = 0;  // om man vill generera unika eventid
int helpersArrivedCount = 0; // antal hjälpare som anlänt till en poi

// Vector med väggar för BFS pathfinding algoritmen
std::vector<Position> walls = {
  { 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 }, { 4, 0 }, { 5, 0 }, { 6, 0 }, { 7, 0 }, { 8, 0 }, { 9, 0 }, { 10, 0 }, { 11, 0 }, { 12, 0 }, { 13, 0 }, { 14, 0 }, { 15, 0 }, { 16, 0 }, { 17, 0 }, { 17, 1 }, { 17, 2 }, { 17, 3 }, { 17, 5 }, { 17, 4 }, { 17, 6 }, { 17, 7 }, { 17, 8 }, { 17, 9 }, { 17, 10 }, { 17, 11 }, { 17, 13 }, { 17, 14 }, { 16, 15 }, { 17, 15 }, { 14, 15 }, { 15, 15 }, { 12, 15 }, { 13, 15 }, { 11, 15 }, { 10, 15 }, { 7, 15 }, { 6, 15 }, { 5, 15 }, { 4, 15 }, { 2, 15 }, { 1, 15 }, { 3, 15 }, { 0, 15 }, { 0, 14 }, { 0, 13 }, { 0, 12 }, { 0, 11 }, { 0, 10 }, { 0, 9 }, { 0, 8 }, { 0, 7 }, { 0, 6 }, { 0, 5 }, { 0, 4 }, { 0, 3 }, { 0, 2 }, { 0, 1 }, { 18, 0 }, { 19, 0 }, { 20, 0 }, { 21, 0 }, { 22, 0 }, { 23, 0 }, { 26, 0 }, { 27, 0 }, { 25, 0 }, { 24, 0 }, { 28, 0 }, { 29, 0 }, { 30, 0 }, { 32, 0 }, { 33, 0 }, { 34, 0 }, { 34, 1 }, { 34, 2 }, { 34, 3 }, { 34, 4 }, { 34, 5 }, { 34, 6 }, { 34, 7 }, { 34, 9 }, { 34, 10 }, { 34, 11 }, { 34, 13 }, { 34, 14 }, { 34, 8 }, { 34, 15 }, { 34, 16 }, { 34, 17 }, { 34, 18 }, { 34, 19 }, { 33, 15 }, { 29, 15 }, { 28, 15 }, { 27, 15 }, { 25, 15 }, { 24, 15 }, { 23, 15 }, { 23, 20 }, { 24, 20 }, { 25, 20 }, { 26, 20 }, { 28, 20 }, { 27, 20 }, { 17, 20 }, { 16, 20 }, { 15, 20 }, { 14, 20 }, { 13, 20 }, { 12, 20 }, { 11, 20 }, { 10, 20 }, { 7, 20 }, { 6, 20 }, { 4, 20 }, { 5, 20 }, { 3, 20 }, { 2, 20 }, { 0, 20 }, { 1, 20 }, { 0, 19 }, { 0, 16 }, { 0, 21 }, { 0, 22 }, { 0, 24 }, { 0, 25 }, { 0, 26 }, { 0, 23 }, { 0, 27 }, { 0, 28 }, { 0, 29 }, { 0, 30 }, { 0, 31 }, { 0, 33 }, { 0, 35 }, { 0, 34 }, { 0, 32 }, { 0, 36 }, { 0, 37 }, { 0, 38 }, { 2, 39 }, { 1, 39 }, { 0, 39 }, { 3, 39 }, { 4, 39 }, { 5, 39 }, { 6, 39 }, { 7, 39 }, { 8, 39 }, { 9, 39 }, { 11, 39 }, { 10, 39 }, { 12, 39 }, { 13, 39 }, { 14, 39 }, { 16, 39 }, { 17, 39 }, { 15, 39 }, { 18, 39 }, { 19, 39 }, { 20, 39 }, { 21, 39 }, { 22, 39 }, { 23, 39 }, { 24, 39 }, { 25, 39 }, { 26, 39 }, { 27, 39 }, { 29, 39 }, { 30, 39 }, { 28, 39 }, { 31, 39 }, { 33, 39 }, { 34, 39 }, { 32, 39 }, { 34, 38 }, { 34, 37 }, { 34, 36 }, { 34, 35 }, { 34, 34 }, { 33, 34 }, { 30, 34 }, { 29, 34 }, { 32, 34 }, { 31, 34 }, { 28, 34 }, { 27, 34 }, { 24, 34 }, { 23, 34 }, { 23, 35 }, { 23, 36 }, { 23, 37 }, { 23, 38 }, { 22, 34 }, { 18, 34 }, { 17, 34 }, { 16, 34 }, { 15, 34 }, { 14, 34 }, { 13, 34 }, { 12, 34 }, { 12, 35 }, { 12, 36 }, { 12, 37 }, { 12, 38 }, { 11, 34 }, { 10, 34 }, { 9, 34 }, { 8, 34 }, { 7, 34 }, { 6, 34 }, { 5, 34 }, { 1, 34 }, { 33, 29 }, { 34, 29 }, { 34, 30 }, { 34, 31 }, { 34, 33 }, { 34, 32 }, { 32, 29 }, { 31, 29 }, { 30, 29 }, { 29, 29 }, { 34, 28 }, { 34, 25 }, { 34, 26 }, { 34, 24 }, { 33, 24 }, { 32, 24 }, { 31, 24 }, { 30, 24 }, { 28, 23 }, { 28, 24 }, { 29, 24 }, { 34, 23 }, { 34, 22 }, { 34, 21 }, { 34, 20 }, { 33, 19 }, { 30, 19 }, { 32, 19 }, { 31, 19 }, { 29, 19 }, { 28, 19 }, { 28, 18 }, { 28, 17 }, { 28, 16 }, { 28, 21 }, { 33, 5 }, { 32, 5 }, { 31, 5 }, { 30, 5 }, { 29, 5 }, { 28, 5 }, { 28, 6 }, { 28, 7 }, { 28, 8 }, { 28, 9 }, { 28, 10 }, { 27, 10 }, { 26, 10 }, { 25, 10 }, { 24, 10 }, { 23, 10 }, { 22, 10 }, { 18, 10 }, { 22, 15 }, { 18, 15 }, { 26, 15 }, { 22, 20 }, { 18, 20 }
};

// Struktur för eventhantering
struct EventData {
  String type; // type t.ex. fire
  Position location; // location
  unsigned long startTime; 
  std::map<uint32_t, float> nodeDistances;  // node_id -> distance
  bool handled = false;                     // Indikerar om eventet redan har hanterats
  bool assigned = false;                    // om eventet är tilldelat en nod
};

// Aktiva händelser
std::map<uint32_t, EventData> activeEvents;

// Huvud tillstånd
enum State {
  IDLE,
  PATROLLING,
  FIRE,
  POI,
  FLAMMABLE_MATERIAL
};

State currentState = IDLE;

// Inre tillstånd för FIRE
enum FireState {
  MOVE_TO_FIRE,
  EXTINGUISH_FIRE,
  EXTINGUISH_SMOKE
};

FireState fireState = MOVE_TO_FIRE;

// Inre tillstånd för POI
enum POIState {
  MOVE_TO_POI,
  WAIT_FOR_HELPERS,
  ASSIST_PERSON,
  WAIT_TO_MOVE_POI,
  MOVE_POI_TO_FIRETRUCK
};

POIState poiState = MOVE_TO_POI;

// Inre tillstånd för FLAMMABLE_MATERIAL
enum FlammableMaterialState {
  MOVE_TO_FLAMMABLE_MATERIAL,
  CARRY_FLAMMABLE_TO_FIRETRUCK
};

FlammableMaterialState flammableMaterialState = MOVE_TO_FLAMMABLE_MATERIAL;

// Struktur för uppgifter
struct NodeTask {
  String type;
  Position location;
  int priority;
  uint32_t eventId;
};

// Startposition manuellt 
Position myPosition = { 1, 16 };
Position targetPosition = { 1, 16 };  // Initiera till startposition
NodeTask currentTask;

// För pathfinding
std::vector<Position> currentPath;

// För hantering av distance calculations
#define DISTANCE_REPORT_TIMEOUT 5000  // 5 sekunder

// för patrullering
Position patrolTarget = { 1, 16 };  // Nästa patrullermål

// -----------------------------------------------------------------------------
// FUNKTIONSDEKLARATIONER
// -----------------------------------------------------------------------------
float calculateDistance(Position pos1, Position pos2);
void handleFireInnerStates();
void handlePOIInnerStates();
void handleFlammableMaterialInnerStates();
void computePath(Position start, Position goal);
void moveAlongPath();
bool isAtPosition(Position pos1, Position pos2);
void setPatrolTarget();
void handleTaskForwarded(uint32_t fromNodeId, uint32_t eventId, uint32_t targetNodeId, String eventType, JsonObject& eventData);
bool extractPosition(const JsonObject& obj, Position& pos);
void assignTask(uint32_t eventId, const EventData& eventData);
void answerHelpButton();
void startRequestHelpTask();
void sendAllReadyMessage();
void pressHelpButton();
void printActiveEvents();
void setColor(uint8_t red, uint8_t green, uint8_t blue);
void drawSwedenFlag(int x, int y, int width, int height);
void updateStateDisplay();
void updateMessageDisplay(String message); 


// -----------------------------------------------------------------------------
//här skriver vi ner aktiva händelser
// -----------------------------------------------------------------------------
void printActiveEvents() {
  Serial.println("=== ACTIVE EVENTS ===");
  if (activeEvents.empty()) {
    Serial.println("No active events currently.");
  } else {
    for (auto &kv : activeEvents) {
      uint32_t eventId = kv.first;
      const EventData &ev = kv.second;

      Serial.printf("Event ID: %u\n", eventId);
      Serial.printf("  type: %s\n", ev.type.c_str());
      Serial.printf("  location: (%d, %d)\n", ev.location.x, ev.location.y);
      Serial.printf("  assigned: %s\n", ev.assigned ? "true" : "false");
      Serial.printf("  handled:  %s\n", ev.handled ? "true" : "false");
      Serial.printf("  startTime(ms): %lu\n", ev.startTime);

      // Om man vill se distanser (nodeDistances):
      if (!ev.nodeDistances.empty()) {
        Serial.println("  nodeDistances: ");
        for (auto &distPair : ev.nodeDistances) {
          uint32_t nodeId = distPair.first;
          float dist = distPair.second;
          Serial.printf("    nodeId=%u => distance=%.2f\n", nodeId, dist);
        }
      } else {
        Serial.println("  nodeDistances: (none)");
      }
      Serial.println(); // tom rad för läsbarhet
    }
  }
  Serial.println("=== END ACTIVE EVENTS ===\n");
}

// -----------------------------------------------------------------------------
// Letar efter o-assignade events med högst prioritet och tar den
// -----------------------------------------------------------------------------
void checkForNewTasks() {
    bool foundEvent = false;
    int bestPriority = -1;
    uint32_t bestEventId = 0;

    // Gå igenom alla activeEvents
    for (auto &kv : activeEvents) {
      if (!kv.second.handled && !kv.second.assigned) {
        // brand=1, poi=2, flammable=3
        int p = 0;
        if      (kv.second.type == "fire_detected")                p = 1;
        else if (kv.second.type == "poi_spawned")                  p = 2;
        else if (kv.second.type == "flammable_material_detected")  p = 3;

        // Letar efter högsta prioritet
        if (p > bestPriority) {
          bestPriority = p;
          bestEventId = kv.first;
          foundEvent = true;
        }
      }
    }

    // Om vi hittade ett event
    if (foundEvent) {
      Serial.printf("[checkForNewTasks] Tar event %u med prio=%d\n", bestEventId, bestPriority);
      assignTask(bestEventId, activeEvents[bestEventId]);
    }
    else {
      Serial.println("[checkForNewTasks] Inga nya events hittade. Jag kan patrullera eller vara IDLE...");
    }
}

// -----------------------------------------------------------------------------
// Task stateMachineTask - undvik "handled=true" om ingen tog branden
// -----------------------------------------------------------------------------
Task stateMachineTask(2000, TASK_FOREVER, []() {
  unsigned long now = millis();

  // 1) Gå igenom alla aktiva event
  for (auto it = activeEvents.begin(); it != activeEvents.end();) {
    EventData &ev = it->second;

    // a) Hoppa över event som redan är assigned eller handled
    if (ev.assigned || ev.handled) {
      ++it;
      continue;
    }

    // b) Om DISTANCE_REPORT_TIMEOUT har passerat
    if ((now - ev.startTime) >= DISTANCE_REPORT_TIMEOUT) {

      // --- 1) RANDOM BACKOFF --------------------------------------------
      // Lägger till en slumpad fördröjning 0–300 ms 
      // för att minska risken att två noder agerar samtidigt.
      delay(random(300, 500));  // <-- NY RAD

      // --- Beräkna eventets prioritet: brand=1, poi=2, flammable=3 ---
      int eventPriority = 0;
      if      (ev.type == "fire_detected")                eventPriority = 1;
      else if (ev.type == "poi_spawned")                  eventPriority = 2;
      else if (ev.type == "flammable_material_detected")  eventPriority = 3;

      // --- 2) Sortera nodeDistances med tie-break på nodeId ---
      std::vector<std::pair<uint32_t, float>> nodeDistVec;
      nodeDistVec.reserve(ev.nodeDistances.size());
      for (auto &p : ev.nodeDistances) {
        nodeDistVec.push_back({ p.first, p.second });
      }
      std::sort(nodeDistVec.begin(), nodeDistVec.end(),
        [](const std::pair<uint32_t, float>& a,
           const std::pair<uint32_t, float>& b)
        {
          // Om distansen är nästan lika, välj lägst nodeId för det här många gånger
          if (fabs(a.second - b.second) < 0.001) {
            return a.first < b.first; // tie-break
          }
          return a.second < b.second; // annars sortera på distans
        }
      );

      bool eventAssignedNow = false; 
      bool eventForwarded   = false;

      // c) Försök hitta en nod i närhetsordning
      for (auto &pair : nodeDistVec) {
        uint32_t candidateNode = pair.first;
        float dist = pair.second;

        // Skippa "noll" (ifall något gick snett)
        if (candidateNode == 0) {
          continue;
        }

        // Om candidateNode == denna nod
        if (candidateNode == mesh.getNodeId()) {
          // Kolla om jag är ledig ELLER har lägre prioritet uppgift
          if (currentState == IDLE || eventPriority > currentTaskPriority) {
            Serial.printf("[Event %u] Jag %u tar uppgiften (prio=%d)\n",
                          it->first, mesh.getNodeId(), eventPriority);

            // Avbryt ev pågående uppgift om ni vill vara "hårda"
            currentState = IDLE;
            updateStateDisplay();

            // Nu tar jag uppgiften
            // => Sätter ev.assigned = true inuti assignTask
            // => Sänder "task_assigned" broadcast
            assignTask(it->first, ev);

            // Markera att jag tog eventet => handled = true (om du vill)
            ev.handled = true; 
            Serial.printf("[Event %u] set handled=true (taken by me).\n", it->first);

            eventAssignedNow = true;
            break;  // klart
          }
          else {
            // Jag vill/kan inte ta => kolla nästa node
            continue;
          }
        }
        else {
          // d) Försök forwarda eventet
          Serial.printf("[Event %u] Försöker forwarda till nod %u (distance=%.2f)\n",
                        it->first, candidateNode, dist);

          // Bygg JSON
          StaticJsonDocument<512> forwardDoc;
          forwardDoc["type"]           = "task_forwarded";
          forwardDoc["event_id"]       = it->first;
          forwardDoc["from_node_id"]   = mesh.getNodeId();
          forwardDoc["target_node_id"] = candidateNode;
          forwardDoc["event_type"]     = ev.type;
          forwardDoc["location"]["x"]  = ev.location.x;
          forwardDoc["location"]["y"]  = ev.location.y;

          String outMsg;
          serializeJson(forwardDoc, outMsg);
          mesh.sendSingle(candidateNode, outMsg);

          // Sätt *inte* ev.handled=true här
          eventForwarded = true;
          break; // vi slutar leta fler noder
        }
      } // end for (nodeDistVec)

      // e) Om varken assignedNow eller forwarded => ingen nod tog den
      if (!eventAssignedNow && !eventForwarded) {
        ev.handled = true; 
        Serial.printf("[Event %u] Ingen nod tog eventet. set handled=true => avbryts.\n", it->first);
      }
    }

    // f) Rensa bort event som varit handled i t.ex. 10s
    //    (ett exempel, man väljer själv)
    if (ev.handled && ((now - ev.startTime) > (DISTANCE_REPORT_TIMEOUT + 10000))) {
      it = activeEvents.erase(it);
    } else {
      ++it;
    }
  } // end for (activeEvents)

  // 2) Byt state (samma logik som förut)
  switch (currentState) {
    case IDLE:
      // Kolla snabbt om nya events
      checkForNewTasks(); 
      if (currentState == IDLE) {
        // Om jag inte ändrats till nån uppgift -> gå mot startPosition
        if (!isAtPosition(myPosition, targetPosition)) {
          computePath(myPosition, targetPosition);
          Serial.println("Returning to start position...");
        } else {
          Serial.println("State: IDLE. Awaiting tasks.");
        }
      }
      break;

    case PATROLLING:
      checkForNewTasks();
      if (currentState == PATROLLING) {
        if (isAtPosition(myPosition, patrolTarget)) {
          setPatrolTarget();
          computePath(myPosition, patrolTarget);
        }
        moveAlongPath();
        Serial.printf("Patrolling: MyPos=(%d,%d), Target=(%d,%d)\n",
                      myPosition.x, myPosition.y,
                      patrolTarget.x, patrolTarget.y);
      }
      break;

    case FIRE:
      handleFireInnerStates();
      break;

    case POI:
      handlePOIInnerStates();
      break;

    case FLAMMABLE_MATERIAL:
      handleFlammableMaterialInnerStates();
      break;
  }

  // skriver ut debug om aktiva event
  printActiveEvents();
});


// -----------------------------------------------------------------------------
// // Task för att skicka position till master med Json form
// -----------------------------------------------------------------------------
Task sendPositionTask(2000, TASK_FOREVER, []() {
  // Skicka position med json form
  StaticJsonDocument<256> posDoc;
  posDoc["type"] = "position_update";
  posDoc["node_id"] = mesh.getNodeId();
  posDoc["position"]["x"] = myPosition.x;
  posDoc["position"]["y"] = myPosition.y;
  String posMsg;
  serializeJson(posDoc, posMsg);
  mesh.sendBroadcast(posMsg);
  Serial.printf("Position update sent: (%d, %d)\n", myPosition.x, myPosition.y);
});

// -----------------------------------------------------------------------------
// // Task för att skicka nodens tillstånds (IDLE, FIRE, POI...) till master med json form
// -----------------------------------------------------------------------------
Task sendCurrentStateTask(1000, TASK_FOREVER, []() {
  // Skicka tillstånd med json form
  StaticJsonDocument<256> stateDoc;
  stateDoc["type"] = "state_update";
  stateDoc["node_id"] = mesh.getNodeId();
  // Konvertera enum State till sträng
  switch (currentState) {
    case IDLE: stateDoc["state"] = "IDLE"; break;
    case PATROLLING: stateDoc["state"] = "PATROLLING"; break;
    case FIRE: stateDoc["state"] = "FIRE"; break;
    case POI: stateDoc["state"] = "POI"; break;
    case FLAMMABLE_MATERIAL: stateDoc["state"] = "FLAMMABLE_MATERIAL"; break;
  }
  String stateMsg;
  serializeJson(stateDoc, stateMsg);
  mesh.sendBroadcast(stateMsg);
  Serial.printf("State update sent: %s\n", stateDoc["state"].as<String>().c_str());

});

// -----------------------------------------------------------------------------
// // Task för att kolla om knappar tryckts för hjäälpförfrågan
// -----------------------------------------------------------------------------
Task taskCheckButtonPresses(20, TASK_FOREVER, []() {
  answerHelpButton();
});


// -----------------------------------------------------------------------------
// // Task för att kolla om en nod vill fråga andra noder om hjälp
// -----------------------------------------------------------------------------
Task requestHelpTask(2000, TASK_FOREVER, []() {
  // om redan fått hjälp eller tillräckligt många svarat
  if (HelpFound || helpAcceptedCount >= 1) {
    Serial.println("Help request complete (HelpFound or 3 nodes accepted). Stopping requestHelpTask.");
    updateMessageDisplay("Help request complete");
    requestHelpTask.disable();
    return;
  }

  // om väntar på svar från en viss nod vänta tills den svaret
  if (waitingForResponse) {
    Serial.println("Still waiting for current node's response...");
    return;
  }

  // annars fråga nästa nod i listan
  for (uint32_t nodeId : allNodes) {
    if (std::find(nodesAsked.begin(), nodesAsked.end(), nodeId) == nodesAsked.end()) {
      currentTargetNode = nodeId;
      nodesAsked.push_back(nodeId);

      // Bygg json form
      StaticJsonDocument<256> helpRequest;
      helpRequest["type"] = "help_node";
      helpRequest["node_id"] = mesh.getNodeId();
      helpRequest["location"]["x"] = myPosition.x;
      helpRequest["location"]["y"] = myPosition.y;

      String helpMsg;
      serializeJson(helpRequest, helpMsg);
      mesh.sendSingle(currentTargetNode, helpMsg);

      Serial.printf("Sent HELP request to node %u\n", currentTargetNode);
      String message = "Sent HELP request to node " + String(currentTargetNode) + "\n";
      updateMessageDisplay(message);

      waitingForResponse = true;  // den här noden måste svara innan vi frågar till nästa nod
      return;                     
    }
  }

  // om vi kommer hit i koden så betyder det att vi behöver fråga till mer noder
  Serial.println("No more nodes available to ask for help. Disabling requestHelpTask.");
  requestHelpTask.disable();
});

// -----------------------------------------------------------------------------
// Callback när ett meddelande tas emot parsar till json, efteråt agerar type
// -----------------------------------------------------------------------------
// 
void receivedCallback(uint32_t from, String& msg) {
  Serial.printf("Message received from %u: %s\n", from, msg.c_str());

   // Försök tolka meddelandet som JSON
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, msg);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  String type = doc["type"];
// 1) Om walls_data - uppdatera 'walls'
  if (type == "walls_data") {
    walls.clear();
    JsonArray wallsArray = doc["walls"];
    for (JsonObject wallObj : wallsArray) {
      Position wallPos;
      wallPos.x = wallObj["x"];
      wallPos.y = wallObj["y"];
      walls.push_back(wallPos);
    }
    Serial.println("Walls data received and updated.");
  } 
  
  // 2) Om distance_report - uppdatera activeEvents
  else if (type == "distance_report") {
    uint32_t eventId = doc["event_id"];
    uint32_t nodeId = doc["node_id"];
    float distance = doc["distance"];

    if (activeEvents.find(eventId) != activeEvents.end()) {
      activeEvents[eventId].nodeDistances[nodeId] = distance;
      Serial.printf("Received distance report from node %u for event %u: distance = %.2f\n", nodeId, eventId, distance);
    }
  } 
  // 3) Om task_forwarded - försök ta över uppgiften
  else if (type == "task_forwarded") {
    uint32_t eventId = doc["event_id"];
    uint32_t fromNodeId = doc["from_node_id"];
    uint32_t targetNodeId = doc["target_node_id"];
    String eventType = doc["event_type"];
    JsonObject eventData = doc["location"];  // endast "location"

    // Anropa den separata funktionen för att hantera uppgiften
    handleTaskForwarded(fromNodeId, eventId, targetNodeId, eventType, eventData);
  }

  // 4) Om task_assigned - en annan nod tog uppgiften
  else if (type == "task_assigned") {
    uint32_t eventId = doc["event_id"];
    uint32_t assignedNodeId = doc["assigned_node_id"];

    Serial.printf("Received task_assigned for event %u by node %u\n", eventId, assignedNodeId);

    // Ta bort eventet från activeEvents
    activeEvents.erase(eventId);

    // Om vi inte är den tilldelade noden, och är i IDLE, gå över till PATROLLING
    if (assignedNodeId != mesh.getNodeId() && currentState == IDLE) {
      currentState = PATROLLING;
      updateStateDisplay();
      setPatrolTarget();
      computePath(myPosition, patrolTarget);
      Serial.println("Another node assigned to task. Switching to PATROLLING.");
    }
  }

   // 5) Om fire_detected, poi_spawned, flammable_material_detected - nya events
  else if (type == "fire_detected" || type == "poi_spawned" || type == "flammable_material_detected") {
    uint32_t eventId = doc["event_id"];
    Position eventLocation;

    // Manuell extraktion av x och y
    if (doc["location"].is<JsonObject>()) {
      if (!extractPosition(doc["location"], eventLocation)) {
        Serial.println("Error: Invalid 'location' data.");
        return;
      }
    } else {
      Serial.println("Error: 'location' is not a valid object.");
      return;
    }

    Serial.printf("Received event %s with id %u at location (%d, %d)\n",
                  type.c_str(), eventId, eventLocation.x, eventLocation.y);

    // Om eventet inte redan finns i activeEvents, lägg till det i activeEvents
    if (activeEvents.find(eventId) == activeEvents.end()) {
      EventData eventData;
      eventData.type = type;
      eventData.location = eventLocation;
      eventData.startTime = millis();
      activeEvents[eventId] = eventData;

      // Beräkna distansen från mig till eventet
      float distance = calculateDistance(myPosition, eventLocation);
      activeEvents[eventId].nodeDistances[mesh.getNodeId()] = distance;

      // Skicka distance_reporten i nätverket till alla noder
      StaticJsonDocument<256> distDoc;
      distDoc["type"] = "distance_report";
      distDoc["event_id"] = eventId;
      distDoc["node_id"] = mesh.getNodeId();
      distDoc["distance"] = distance;
      String distMsg;
      serializeJson(distDoc, distMsg);
      mesh.sendBroadcast(distMsg);
      Serial.printf("Sent distance report to other firemen for event %u: distance = %.2f\n", eventId, distance);
    }
  }


  // 6) POI-händelser: arrived_for_poi
  else if (type == "arrived_for_poi") {
    // Endast intressant för den nod som officiellt äger POI
    if (isAssignedPOI && currentState == POI && poiState == WAIT_FOR_HELPERS) {
        uint32_t helperId = doc["helper_id"];
        Serial.printf("Helper %u arrived at POI!\n", helperId);
        String message1 = "Helper " + String(helperId) + " arrived at POI!\n";
        updateMessageDisplay(message1);

        // Öka räknaren för anlända hjälpare
        helpersArrivedCount++;

        // Kontrollera om alla hjälpare har anlänt
        if (helpersArrivedCount >= REQUIRED_HELPERS) {
            Serial.println("All helpers have arrived at POI. Sending start_move_to_firetruck.");
            
            // Skicka start_move_to_firetruck till alla accepterade hjälpare
            for (uint32_t helperId : acceptedNodes) {
                StaticJsonDocument<128> startMoveDoc;
                startMoveDoc["type"] = "start_move_to_firetruck";
                startMoveDoc["poi_x"] = helpRequestLocation.x;
                startMoveDoc["poi_y"] = helpRequestLocation.y;
                String msg;
                serializeJson(startMoveDoc, msg);
                mesh.sendSingle(helperId, msg);
                Serial.printf("Sent start_move_to_firetruck to helper %u\n", helperId);
                String message2 = "Sent start_move_to_firetruck to helper " + String(helperId) + "\n";
                updateMessageDisplay(message2);
            }

            // Owner node också går till brandbilen
            Position firetruckPos = {1, 17}; // Brandbilens position
            computePath(myPosition, firetruckPos);
            targetPosition = firetruckPos;
            poiState = MOVE_POI_TO_FIRETRUCK;
            Serial.println("Owner node moving to firetruck.");
            updateMessageDisplay("Owner node moving to firetruck");
        }
    }
}

  // 7) Om global_help_node - alla noder ska vänta på svar
  else if (type == "global_help_node") {
     setColor(0, 0, 255);  // Blå LED
    Serial.println("GLOBAL HELP message received - entire network must answer yes/no!");
    updateMessageDisplay("GLOBAL HELP message received - entire network must answer yes/no!");
    helpRequestNodeId = doc["node_id"];  // Den nod som sände global_help_node
    helpRequestLocation.x = doc["location"]["x"];
    helpRequestLocation.y = doc["location"]["y"];

    // Alla noder sätter waitingForAnswer = true;
    waitingForAnswer = true;
    taskCheckButtonPresses.enable();
    // Om man vill helt stoppa state machine: blockAllMotion = true;
  }

   // 8) Om help_node - en förfrågan om hjälp
  else if (type == "help_node") {
    Serial.println("HELP message received");
    updateMessageDisplay("Help message received");
    // Extrahera nodeID och position där hjälp behövs
    helpRequestNodeId = from;  // Sender's node ID
    helpRequestLocation.x = doc["location"]["x"];
    helpRequestLocation.y = doc["location"]["y"];

    Serial.printf("Help requested by node %u at location (%d, %d)\n",
                  helpRequestNodeId, helpRequestLocation.x, helpRequestLocation.y);

    // vänta till button är nedtryckt
    waitingForAnswer = true;
    taskCheckButtonPresses.enable();
  }

  // 9) Om help_accepted - någon har tackat ja till vår help-förfrågan
  else if (type == "help_accepted") {
    setColor(255, 0, 0);  //röd LED
    uint32_t responderId = doc["node_id"];
    String message = "Node " + String(responderId) + " accepted the help request.\n";
    updateMessageDisplay(message);
    // Lägg till hjälparnoden till listan över accepterade hjälpare
    if (std::find(acceptedNodes.begin(), acceptedNodes.end(), responderId) == acceptedNodes.end()) {
        acceptedNodes.push_back(responderId);
        helpAcceptedCount++;
        Serial.printf("Help accepted count: %d\n", helpAcceptedCount);
    }
     setColor(0, 0, 0); //släck den
    
}

  // 10) Om help_declined
   else if (type == "help_declined") {
    if (from == currentTargetNode) {
      setColor(0, 255, 0);  
      String message = "Node " + String(from) + " declined the help request.\n";
      updateMessageDisplay(message);
      waitingForResponse = false;  // skicka till nästa nod
    }
    setColor(0, 0, 0); //släcka led
  } 
  // 11) om all_ready
  else if (type == "all_ready") {
    AllReady = true;
  }

  // 12) Om start_move_to_firetruck - denna nod är en hjälpare och ska gå till brandbilen
  else if (type == "start_move_to_firetruck") {
    if (isHelperPOI) {
        Position firetruckPos = {1, 17}; // Brandbilens position
        targetPosition = firetruckPos;
        computePath(myPosition, firetruckPos);
        poiState = MOVE_POI_TO_FIRETRUCK;
        Serial.printf("Received start_move_to_firetruck, moving to firetruck at (%d, %d)\n", firetruckPos.x, firetruckPos.y);
    }
}
// 13) Om start_patrolling - börja patrullera
  else if (type == "start_patrolling") {
    if (currentState != PATROLLING) {
      currentState = PATROLLING;
      updateStateDisplay();
      setPatrolTarget();
      computePath(myPosition, patrolTarget);
      Serial.println("Switching to PATROLLING.");
    }
  } 
  // 14) Om task_complete - en nod rapporterar färdig uppgift
  else if (type == "task_complete") {
    uint32_t eventId = doc["event_id"];
    uint32_t nodeId = doc["node_id"];
    String result = doc["result"];

    Position location;
    if (!extractPosition(doc["location"], location)) {
      Serial.println("Error: Invalid 'location' data in task_complete.");
      return;
    }

    Serial.printf("Task completed for event_id %u by node_id %u with result: %s at location (%d, %d)\n",
                  eventId, nodeId, result.c_str(), location.x, location.y);

    // Om detta event finns i activeEvents, ta bort det
    activeEvents.erase(eventId);

    // Uppdatera andra noder eller utför ytterligare logik om nödvändigt
  } 
  // 15) Om vi tar emot "start_poi_carry", då blir denna nod en HELPER
   else if (type == "start_poi_carry") {
    // Markera att jag är hjälpande brandman
    isHelperPOI = true;

    Position poiPos;
    poiPos.x = doc["poi_x"];
    poiPos.y = doc["poi_y"];

    // Gå in i POI-läge och sätt MOVE_TO_POI
    currentState = POI;
    poiState = MOVE_TO_POI;
    updateStateDisplay();
    targetPosition = poiPos;
    computePath(myPosition, targetPosition);

    Serial.println("I am now helping with the POI. Moving to person...");
    updateMessageDisplay("I am now helping with the POI. Moving to person...");
}
  else {
    Serial.printf("Ignoring unknown or invalid message: %s\n", msg.c_str());
  }
}


// -----------------------------------------------------------------------------
//  extractPosition läser ut pos.x, pos.y ur en JSON-struktur
// -----------------------------------------------------------------------------
bool extractPosition(const JsonObject& obj, Position& pos) {  
  if (obj.containsKey("x") && obj.containsKey("y")) {
    pos.x = obj["x"];
    pos.y = obj["y"];
    return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
//  handleTaskForwarded Om denna nod är targetNodeId, försök ta uppgift, annars forsätt forwarda
// -----------------------------------------------------------------------------
void handleTaskForwarded(uint32_t fromNodeId,
                         uint32_t eventId,
                         uint32_t targetNodeId,
                         String eventType,
                         JsonObject& eventData)
{
  // Om denna nod är den “target node” som ska försöka ta uppgiften ...
  if (mesh.getNodeId() == targetNodeId) {
    // 1) Räkna ut prioritet 
    int eventPriority = 0;
    if (eventType == "flammable_material_detected") eventPriority = 3;
    else if (eventType == "poi_spawned") eventPriority = 2;
    else if (eventType == "fire_detected") eventPriority = 1;

    // 2) Hämta eventPosition
    Position eventPosition;
    if (!extractPosition(eventData, eventPosition)) {
      Serial.println("Error: 'location' field missing or invalid in eventData.");
      return;
    }

    // 3) Om jag är IDLE eller har lägre priority uppgift, ta uppgiften
    if (currentState == IDLE || eventPriority > currentTaskPriority) {
      Serial.printf("Node %u has been forwarded task %u from node %u\n",
                    mesh.getNodeId(), eventId, fromNodeId);

      // Markera eventet som assigned
      EventData& evt = activeEvents[eventId];
      evt.assigned = true;
      evt.handled = true;

      // Byt state efter event-typ
      if (eventType == "fire_detected") {
        currentState = FIRE;
        fireState = MOVE_TO_FIRE;
        updateStateDisplay();
        targetPosition = eventPosition;
        computePath(myPosition, targetPosition);
        currentTask.priority = 1;
      } else if (eventType == "poi_spawned") {
        currentState = POI;
        poiState = MOVE_TO_POI;
        updateStateDisplay();
        targetPosition = eventPosition;
        computePath(myPosition, targetPosition);
        currentTask.priority = 2;
        isAssignedPOI = true;  //markera att denna nod officiellt tar POI-uppgiften
      } else if (eventType == "flammable_material_detected") {
        currentState = FLAMMABLE_MATERIAL;
        flammableMaterialState = MOVE_TO_FLAMMABLE_MATERIAL;
        updateStateDisplay();
        targetPosition = eventPosition;
        computePath(myPosition, targetPosition);
        currentTask.priority = 3;
      }

      // Sätt currentTask
      currentTask.type = eventType;
      currentTask.location = eventPosition;
      currentTask.eventId = eventId;
      currentTaskPriority = currentTask.priority;

      // Sänd task_assigned till alla
      StaticJsonDocument<512> taskDoc;
      taskDoc["type"] = "task_assigned";
      taskDoc["event_id"] = eventId;
      taskDoc["assigned_node_id"] = mesh.getNodeId();
      String taskMsg;
      serializeJson(taskDoc, taskMsg);
      mesh.sendBroadcast(taskMsg);
    }
    else {
      // Noden kan inte/vill inte ta uppgiften, vidarebefordra tillbaka
      uint32_t nextNodeId = fromNodeId;

      // Om nextNodeId == fromNodeId, då är vi på väg att skicka samma event tillbaka
      // till den som gav oss eventet nyss. här bryter vi loopen genom att markera eventet “handled”.
      if (nextNodeId == mesh.getNodeId()) {
        // Skulle betyda att fromNodeId == targetNodeId == mesh.getNodeId(), mycket konstigt fall:
        // Avbryt händelsen
        if (activeEvents.find(eventId) != activeEvents.end()) {
          activeEvents[eventId].handled = true;
        }
        Serial.println("Loop avoidance: I'm both source and target. Discarding event.");
        return;
      }
      if (nextNodeId == fromNodeId) {
        // Vi är på väg att skicka eventet tillbaka till precis samma nod
        if (activeEvents.find(eventId) != activeEvents.end()) {
          activeEvents[eventId].handled = true;
        }
        Serial.printf("Loop avoidance: Would forward event %u back to fromNodeId %u. Discarding.\n",
                      eventId, fromNodeId);
        return;
      }

      Serial.printf("Node %u cannot take the task %u. Forwarding to node %u.\n",
                    mesh.getNodeId(), eventId, nextNodeId);

      StaticJsonDocument<512> forwardTaskDoc;
      forwardTaskDoc["type"] = "task_forwarded";
      forwardTaskDoc["event_id"] = eventId;
      forwardTaskDoc["from_node_id"] = mesh.getNodeId();
      forwardTaskDoc["target_node_id"] = nextNodeId;
      forwardTaskDoc["event_type"] = eventType;
      forwardTaskDoc["location"]["x"] = eventPosition.x;
      forwardTaskDoc["location"]["y"] = eventPosition.y;

      String forwardTaskMsg;
      serializeJson(forwardTaskDoc, forwardTaskMsg);
      mesh.sendSingle(nextNodeId, forwardTaskMsg);
    }
  }
  // Om denna nod inte är targetNodeId, så kollar vi också loop-fall
  else {
    // Sannolikt är targetNodeId = en helt annan nod i nätet.
    // Kolla loop undantag:
    if (targetNodeId == fromNodeId) {
      // Samma logik: vi är på väg att skicka tillbaka direkt
      if (activeEvents.find(eventId) != activeEvents.end()) {
        activeEvents[eventId].handled = true;
      }
      Serial.printf("Loop avoidance: Would forward event %u back to fromNodeId %u. Discarding.\n",
                    eventId, fromNodeId);
      return;
    }

    Serial.printf("Node %u cannot take the task %u. Forwarding to node %u.\n",
                  mesh.getNodeId(), eventId, targetNodeId);

    StaticJsonDocument<512> forwardTaskDoc;
    forwardTaskDoc["type"] = "task_forwarded";
    forwardTaskDoc["event_id"] = eventId;
    forwardTaskDoc["from_node_id"] = mesh.getNodeId();
    forwardTaskDoc["target_node_id"] = targetNodeId;
    forwardTaskDoc["event_type"] = eventType;
    forwardTaskDoc["location"]["x"] = eventData["x"];
    forwardTaskDoc["location"]["y"] = eventData["y"];

    String forwardTaskMsg;
    serializeJson(forwardTaskDoc, forwardTaskMsg);
    mesh.sendSingle(targetNodeId, forwardTaskMsg);
  }
}

// -----------------------------------------------------------------------------
//  Räknar ut avståndet här
// -----------------------------------------------------------------------------
float calculateDistance(Position pos1, Position pos2) {
  int dx = pos1.x - pos2.x;
  int dy = pos1.y - pos2.y;
  return sqrt(dx * dx + dy * dy);
}

// -----------------------------------------------------------------------------
//  // Hantera inner states för FIRE
// -----------------------------------------------------------------------------
void handleFireInnerStates() {
  static unsigned long lastActionTime = 0;
  unsigned long currentTime = millis();

  switch (fireState) {
    case MOVE_TO_FIRE:
      if (currentTime - lastActionTime >= 2000) {  // Flytta var 2000 ms
        moveAlongPath();
        lastActionTime = currentTime;
        Serial.printf("Moving towards fire: Current Position (%d, %d)\n", myPosition.x, myPosition.y);

        if (isAtPosition(myPosition, targetPosition)) {
          fireState = EXTINGUISH_FIRE;
          updateStateDisplay();
          lastActionTime = currentTime;
          Serial.println("Arrived at fire location. Transitioning to EXTINGUISH_FIRE.");
          updateMessageDisplay("Arrived at fire location.");
        }
      }
      break;

    case EXTINGUISH_FIRE:
      if (currentTime - lastActionTime >= 5000) {  // Vänta 5 sekunder för att släcka eld
        Serial.println("Extinguishing fire...");
        fireState = EXTINGUISH_SMOKE;
        updateStateDisplay();
        lastActionTime = currentTime;

        // -- NU SKICKAS FÖRSTA TASK_COMPLETE FÖR ELD --
        StaticJsonDocument<256> docFire;
        docFire["type"] = "task_complete";
        docFire["event_id"] = currentTask.eventId;
        docFire["node_id"] = mesh.getNodeId();
        docFire["result"] = "fire_extinguished";      // <--- Eld borta
        docFire["location"]["x"] = targetPosition.x;
        docFire["location"]["y"] = targetPosition.y;
        String msgFire;
        serializeJson(docFire, msgFire);
        mesh.sendBroadcast(msgFire);
        Serial.println("Reported fire_extinguished");
        updateMessageDisplay("Fire extinguished");
      }
      break;

    case EXTINGUISH_SMOKE:
      if (currentTime - lastActionTime >= 5000) {
        Serial.println("Extinguishing smoke...");

       // -- ANDRA TASK_COMPLETE FÖR RÖK --
        StaticJsonDocument<256> docSmoke;
        docSmoke["type"] = "task_complete";
        docSmoke["event_id"] = currentTask.eventId;
        docSmoke["node_id"] = mesh.getNodeId();
        docSmoke["result"] = "smoke_extinguished";  // <--- Nu tas röken bort
        docSmoke["location"]["x"] = targetPosition.x;
        docSmoke["location"]["y"] = targetPosition.y;
        String msgSmoke;
        serializeJson(docSmoke, msgSmoke);
        mesh.sendBroadcast(msgSmoke);
        Serial.println("Reported smoke_extinguished");
        updateMessageDisplay("Smoke extinguished");

        // --- KOLLA OM DET FINNS FLER EVENT ---
        bool foundEvent = false;
        int bestPriority = -1;
        uint32_t bestEventId = 0;

        // Leta i activeEvents
        for (auto &kv : activeEvents) {
          if (!kv.second.handled && !kv.second.assigned) {
            // brand=1, poi=2, flammable=3
            int p = 0;
            if (kv.second.type == "fire_detected") p = 1;
            else if (kv.second.type == "poi_spawned") p = 2;
            else if (kv.second.type == "flammable_material_detected") p = 3;

            if (p > bestPriority) {
              bestPriority = p;
              bestEventId = kv.first;
              foundEvent = true;
            }
          }
        }

        if (foundEvent) {
          // Ta nästa uppgift med högst prioritet
          Serial.printf("New event found (priority=%d). Taking eventId=%u.\n", bestPriority, bestEventId);
          assignTask(bestEventId, activeEvents[bestEventId]);
        } else {

          checkForNewTasks();
          // Annars -> patrullera
          currentState = PATROLLING;
          fireState = MOVE_TO_FIRE;
          updateStateDisplay(); 
          currentTaskPriority = 0;
          setPatrolTarget();
          computePath(myPosition, patrolTarget);
          Serial.println("No new events -> switching to PATROLLING.");
        }
      }
      break;
  }
}
// -----------------------------------------------------------------------------
//  // Hantera inner states för POI
// -----------------------------------------------------------------------------
void handlePOIInnerStates() {
  static unsigned long lastActionTime = 0;
  static unsigned long dropStartTime = 0;
  static bool dropping = false;

  unsigned long currentTime = millis();

  switch (poiState) {

    // 1) MOVE_TO_POI
    case MOVE_TO_POI: {
      if (currentTime - lastActionTime >= 500) {
        moveAlongPath();
        lastActionTime = currentTime;

        // Om jag nått POI:
        if (isAtPosition(myPosition, targetPosition)) {

          // a) Om denna nod är "ägaren" av POI
          if (isAssignedPOI) {
            // Gå till ASSIST_PERSON istället för att automatiskt skicka hjälp
            poiState = ASSIST_PERSON;
            updateStateDisplay();
            Serial.println("Arrived at POI => ASSIST_PERSON.");
            updateMessageDisplay("Arrived at POI");
          }
          // b) Om denna nod är en hjälpnod
          else if (isHelperPOI) {
              Serial.println("Helper arrived at POI => sending arrived_for_poi");
              // Bygg och skicka arrived_for_poi till den nod som äger POI
              updateMessageDisplay("Helper arrived at POI");
              StaticJsonDocument<128> doc;
              doc["type"] = "arrived_for_poi";
              doc["helper_id"] = mesh.getNodeId();
              String outMsg;
              serializeJson(doc, outMsg);
              mesh.sendSingle(helpRequestNodeId, outMsg);

              // Vänta på att alla hjälpare ska anlänt innan man fortsätter
              poiState = WAIT_TO_MOVE_POI;
              updateStateDisplay();
              Serial.println("Helper arrived at POI and waiting for all helpers to arrive.");
          }
        }
      }
    } break;

    // 2) ASSIST_PERSON (för ägaren av POI)
    case ASSIST_PERSON: {
      // Här vill jag att ägaren trycker på en *fysisk knapp* (helpButton)
      // för att skicka hjälpmeddelandet
      if (digitalRead(helpButton) == LOW) {
        delay(50); 
        if (digitalRead(helpButton) == LOW) { 
          // Nu HAR vi tryckt på knappen => skicka hjälpmeddelande
          Serial.println("Help button pressed => sending help request & transition to WAIT_FOR_HELPERS");
          updateMessageDisplay("Help button pressed");

          // Skicka global_help_node för att be alla hjälpare
          StaticJsonDocument<256> helpDoc;
          helpDoc["type"]        = "global_help_node"; 
          helpDoc["node_id"]     = mesh.getNodeId(); 
          helpDoc["location"]["x"] = myPosition.x;
          helpDoc["location"]["y"] = myPosition.y;

          String helpMsg;
          serializeJson(helpDoc, helpMsg);
          mesh.sendBroadcast(helpMsg);

          // Nollställ räknaren för hur många hjälpare som har anlänt
          helpersArrivedCount = 0;

          // Byt state => nu väntar jag i WAIT_FOR_HELPERS
          poiState = WAIT_FOR_HELPERS;
          updateStateDisplay();
        }
      }
    } break;

 
    // 3) WAIT_FOR_HELPERS
    case WAIT_FOR_HELPERS: {
      // Vänta tills alla nödvändiga hjälpare har anlänt
      if (helpersArrivedCount >= REQUIRED_HELPERS) {
        Serial.println("All required helpers have arrived => sending start_move_to_firetruck");
        updateMessageDisplay("All required helpers have arrived");
        // Skicka start_move_to_firetruck till alla accepterade hjälpare
        for (uint32_t helperId : acceptedNodes) {
          StaticJsonDocument<128> startMoveDoc;
          startMoveDoc["type"] = "start_move_to_firetruck";
          startMoveDoc["poi_x"] = helpRequestLocation.x;
          startMoveDoc["poi_y"] = helpRequestLocation.y;
          String msg;
          serializeJson(startMoveDoc, msg);
          mesh.sendSingle(helperId, msg);
          Serial.printf("Sent start_move_to_firetruck to helper %u\n", helperId);
        }

        // Owner node också går till brandbilen
        Position firetruckPos = {1, 17}; // Brandbilens position
        computePath(myPosition, firetruckPos);
        targetPosition = firetruckPos;
        poiState = MOVE_POI_TO_FIRETRUCK;
        updateStateDisplay();
        Serial.println("Owner node moving to firetruck.");
      }
    } break;

    // 4) MOVE_POI_TO_FIRETRUCK
  case MOVE_POI_TO_FIRETRUCK: {
      Position firetruckPos = {1, 17}; // Brandbilens position

      if (currentPath.empty()) {
          computePath(myPosition, firetruckPos);  // Beräkna väg till brandbilen
      }

      if (currentTime - lastActionTime >= 500) {
          moveAlongPath();
          lastActionTime = currentTime;

          // Skicka uppdatering om att brandmannen bär personen
          StaticJsonDocument<256> doc;
          doc["type"] = "poi_carrying_update";
          doc["node_id"] = mesh.getNodeId();
          doc["poi_carrying"] = true;  // Flagga för att signalera att personen bärs
          doc["position"]["x"] = myPosition.x;
          doc["position"]["y"] = myPosition.y;
          String msg;
          serializeJson(doc, msg);
          mesh.sendBroadcast(msg);

          if (isAtPosition(myPosition, firetruckPos)) {
              if (!dropping) {
                  dropStartTime = currentTime;  // Starta avlämningstimer
                  dropping = true;
              }

              if (dropping && currentTime - dropStartTime >= 3000) {
                  // Rapportera att POI har levererats med json form
                  StaticJsonDocument<256> completeDoc;
                  completeDoc["type"] = "task_complete";
                  completeDoc["node_id"] = mesh.getNodeId();
                  completeDoc["result"] = "poi_carried";
                  completeDoc["location"]["x"] = firetruckPos.x;
                  completeDoc["location"]["y"] = firetruckPos.y;
                  String completeMsg;
                  serializeJson(completeDoc, completeMsg);
                  mesh.sendBroadcast(completeMsg);
                  updateMessageDisplay("Poi carried");
                  dropping = false;
                  currentState = PATROLLING;
                  updateStateDisplay();
                  setPatrolTarget();
                  computePath(myPosition, patrolTarget);
              }
          }
      }
    } break;


    // 5) WAIT_TO_MOVE_POI (för hjälpnoder)
    case WAIT_TO_MOVE_POI: {
        // Vänta på start_move_to_firetruck meddelandet
        // Ingen handling behövs här eftersom övergång sker via receivedCallback
        updateStateDisplay();
        break;
    } break;

  } // end switch
}

// -----------------------------------------------------------------------------
// Hantera inner states för FLAMMABLE_MATERIAL
// -----------------------------------------------------------------------------
void handleFlammableMaterialInnerStates() {
  static unsigned long lastActionTime = 0;
  static unsigned long dropStartTime = 0;
  static bool dropping = false;

  unsigned long currentTime = millis();

  switch (flammableMaterialState) {

    // 1) MOVE_TO_FLAMMABLE_MATERIAL
    case MOVE_TO_FLAMMABLE_MATERIAL:
      if (currentTime - lastActionTime >= 500) {
        moveAlongPath();
        lastActionTime = currentTime;
        if (isAtPosition(myPosition, targetPosition)) {
          {//plocka upp materialet
          //skriv meddelandet med json form att flammable picked up
            StaticJsonDocument<256> pickupDoc;
            pickupDoc["type"] = "flammable_picked_up";
            pickupDoc["node_id"] = mesh.getNodeId();
            pickupDoc["location"]["x"] = targetPosition.x;
            pickupDoc["location"]["y"] = targetPosition.y;
            String pickupMsg;
            serializeJson(pickupDoc, pickupMsg);
            mesh.sendBroadcast(pickupMsg);
            updateMessageDisplay("Flammable picked up");

          }
          // gå till brandbilen
          Position firetruckPos = { 1, 17 };
          computePath(myPosition, firetruckPos);
          flammableMaterialState = CARRY_FLAMMABLE_TO_FIRETRUCK;
          updateStateDisplay();

          {
            //skicka state update med json form
            StaticJsonDocument<256> stateDoc;
            stateDoc["type"] = "state_update";
            stateDoc["node_id"] = mesh.getNodeId();
            stateDoc["state"] = "CARRY_FLAMMABLE_TO_FIRETRUCK";
            String stateMsg;
            serializeJson(stateDoc, stateMsg);
            mesh.sendBroadcast(stateMsg);
          }
        }
      }
      break;

    // 1) CARRY_FLAMMABLE_TO_FIRETRUCK 
    case CARRY_FLAMMABLE_TO_FIRETRUCK:
      if (currentTime - lastActionTime >= 500) {
        moveAlongPath();
        lastActionTime = currentTime;
        Position firetruckPos = { 1, 17 };
        if (isAtPosition(myPosition, firetruckPos)) {
          if (!dropping) {
            dropStartTime = currentTime;
            dropping = true;
          }

          if (dropping && currentTime - dropStartTime >= 3000) {
            {
              //rapportera med json form
              StaticJsonDocument<256> doc;
              doc["type"] = "task_complete";
              doc["event_id"] = currentTask.location.x * 1000 + currentTask.location.y;
              doc["node_id"] = mesh.getNodeId();
              doc["result"] = "flammable_material_carried";
              doc["location"]["x"] = myPosition.x;
              doc["location"]["y"] = myPosition.y;
              String msg;
              serializeJson(doc, msg);
              mesh.sendBroadcast(msg);
              updateMessageDisplay("Flammable material carried");
            }

            dropping = false;

            // Ingen ny uppgift hittades återgå till patrullering
            currentState = PATROLLING;
            updateStateDisplay();
            setPatrolTarget();
            computePath(myPosition, patrolTarget);
          }
        }
      }
      break;
  }
}

// -----------------------------------------------------------------------------
// En enkel BFS för att hitta en väg genom kartan genom att undvika väggar
// -----------------------------------------------------------------------------
void computePath(Position start, Position goal) {
    currentPath.clear();
    if (start.x == goal.x && start.y == goal.y) {
        return;
    }

    // BFS-algorithm
    std::queue<std::vector<Position>> q;
    std::vector<Position> initialPath = { start };
    q.push(initialPath);

    std::map<int, std::map<int, bool>> visited;
    visited[start.x][start.y] = true;

    bool pathFound = false;  // Spårar om path hittas

    while (!q.empty()) {
        std::vector<Position> path = q.front();
        q.pop();

        Position last = path.back();

        std::vector<std::pair<int, int>> directions = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
        for (auto& dir : directions) {
            Position next;
            next.x = last.x + dir.first;
            next.y = last.y + dir.second;

            if (next.x < 0 || next.x >= 35 || next.y < 0 || next.y >= 40)
                continue;

            bool isWall = false;
            for (auto& wall : walls) {
                if (wall.x == next.x && wall.y == next.y) {
                    isWall = true;
                    break;
                }
            }
            if (isWall)
                continue;

            if (visited[next.x].find(next.y) != visited[next.x].end())
                continue;

            std::vector<Position> newPath = path;
            newPath.push_back(next);
            q.push(newPath);
            visited[next.x][next.y] = true;

            if (next.x == goal.x && next.y == goal.y) {
                currentPath = newPath;
                Serial.printf("Path found with %lu steps.\n", currentPath.size());
                pathFound = true;
                break;
            }
        }
        if (pathFound) break;
    }

    if (!pathFound) {
        Serial.println("No path found.");
        // Om vi är i PATROLLING-läge och ingen path hittas, välj ett nytt mål men begränsa antalet försök
        if (currentState == PATROLLING) {
            Serial.println("No path found during PATROLLING. Selecting a new patrol target.");
            
            // Inför en räknare för att undvika oändliga försök
            static int patrolRetryCount = 0;
            const int MAX_PATROL_RETRIES = 5;

            if (patrolRetryCount < MAX_PATROL_RETRIES) {
                patrolRetryCount++;
                setPatrolTarget();
                computePath(myPosition, patrolTarget);
            } else {
                Serial.println("Max patrol retries reached. Switching to IDLE.");
                currentState = IDLE;
                updateStateDisplay();
                patrolRetryCount = 0;  // Återställ räknaren
                setPatrolTarget();      // sätter ett säkert mål
            }
        }
    } else {
        // Återställ räknaren om en väg hittas
        if (currentState == PATROLLING) {
            static int patrolRetryCount = 0;
            patrolRetryCount = 0;
        }
    }
}

// -----------------------------------------------------------------------------
// Tar nästa steg i currentPath och anropar checkForUnassignedFireAtMyPosition()
// -----------------------------------------------------------------------------
void moveAlongPath() {
  if (currentPath.empty()) {
    return;
  }

  // Ta nästa steg
  Position nextStep = currentPath.front();
  currentPath.erase(currentPath.begin());
  myPosition = nextStep;

  Serial.printf("Moved to (%d, %d)\n", myPosition.x, myPosition.y);

  // Kolla om här finns en brand (eller annat scenario) oassignad
  checkForUnassignedFireAtMyPosition();
}

// -----------------------------------------------------------------------------
// Funktion för att kolla om två positioner är samma
// -----------------------------------------------------------------------------
bool isAtPosition(Position pos1, Position pos2) {
  return (pos1.x == pos2.x) && (pos1.y == pos2.y);
}

// -----------------------------------------------------------------------------
// Funktion för att sätta ett nytt patrullermål
// -----------------------------------------------------------------------------
void setPatrolTarget() {
    // välj ett nytt slumpmässigt mål inom kartans gränser
    bool targetSet = false;
    int attempt = 0;
    const int MAX_ATTEMPTS = 10;

    while (!targetSet && attempt < MAX_ATTEMPTS) {
        int new_x = random(0, 35);
        int new_y = random(0, 40);
        Position newTarget = { new_x, new_y };

        // Kontrollera att målpositionen inte är en vägg
        bool isWall = false;
        for (auto& wall : walls) {
            if (wall.x == newTarget.x && wall.y == newTarget.y) {
                isWall = true;
                break;
            }
        }

        if (!isWall) {
            patrolTarget = newTarget;
            Serial.printf("New patrol target set to (%d, %d)\n", new_x, new_y);
            targetSet = true;
        }
        attempt++;
    }

    if (!targetSet) {
        Serial.println("Failed to set a new patrol target after multiple attempts.");
        // gå till IDLE
        currentState = IDLE;
        updateStateDisplay();
    } else {
        computePath(myPosition, patrolTarget);
    }
}

// -----------------------------------------------------------------------------
// Funktion för att tilldela uppgift till noden
// -----------------------------------------------------------------------------
void assignTask(uint32_t eventId, const EventData& eventData) {
  // Hämta en referens till eventet i activeEvents
  EventData& evt = activeEvents[eventId];

  // Markera eventet som assigned
  evt.assigned = true;
  Serial.printf("assignTask: Event %u is now assigned to me (node %u)\n", eventId, mesh.getNodeId());

  // Uppdatera tillståndet baserat på eventets typ
  if (evt.type == "fire_detected") {
    currentState = FIRE;
    fireState = MOVE_TO_FIRE;
    updateStateDisplay();
    targetPosition = evt.location;
    computePath(myPosition, targetPosition);
    currentTask.priority = 1;
  } 
  else if (evt.type == "poi_spawned") {
    currentState = POI;
    poiState = MOVE_TO_POI;
    updateStateDisplay();
    targetPosition = evt.location;
    computePath(myPosition, targetPosition);
    currentTask.priority = 2;
    isAssignedPOI = true;
  } 
  else if (evt.type == "flammable_material_detected") {
    currentState = FLAMMABLE_MATERIAL;
    flammableMaterialState = MOVE_TO_FLAMMABLE_MATERIAL;
    updateStateDisplay();
    targetPosition = evt.location;
    computePath(myPosition, targetPosition);
    currentTask.priority = 3;
  }

  // Sätt currentTask
  currentTask.type = evt.type;
  currentTask.location = evt.location;
  currentTask.eventId = eventId;

  // Sänd meddelande om att uppgiften är tilldelad (broadcast)
  StaticJsonDocument<256> taskDoc;
  taskDoc["type"] = "task_assigned";
  taskDoc["event_id"] = eventId;
  taskDoc["assigned_node_id"] = mesh.getNodeId();
  String taskMsg;
  serializeJson(taskDoc, taskMsg);
  mesh.sendBroadcast(taskMsg);

  // Uppdatera prioriteten för aktuell uppgift
  currentTaskPriority = currentTask.priority;

  // => RADERA EVENTET UR activeEvents DIREKT
  activeEvents.erase(eventId);
  Serial.printf("assignTask: Erased event %u from activeEvents.\n", eventId);
}

// -----------------------------------------------------------------------------
// Funktion för att kolla om vi väntar på svar från användaren
// -----------------------------------------------------------------------------
void answerHelpButton() {
  if (!waitingForAnswer) {
    return;
  }

  // Check YES button
if (digitalRead(yesButton) == LOW) {
  delay(50);  
  if (digitalRead(yesButton) == LOW) {
    setColor(255, 0, 0);  
    Serial.println("Yes button pressed");

    // 1) Skicka help_accepted till den brandman som bad om hjälp
    StaticJsonDocument<256> yesResponse;
    yesResponse["type"]   = "help_accepted";
    yesResponse["node_id"] = mesh.getNodeId();
    yesResponse["location"]["x"] = helpRequestLocation.x;
    yesResponse["location"]["y"] = helpRequestLocation.y;
    String msg;
    serializeJson(yesResponse, msg);
    mesh.sendSingle(helpRequestNodeId, msg);

    // 2) Sätt denna nod i POI-läget
    currentState = POI;
    poiState = MOVE_TO_POI;
    updateStateDisplay();
    updateMessageDisplay("Help Accepted!");
    isHelperPOI = true;

    // 3) Se till att vi faktiskt går till personens position
    targetPosition.x = helpRequestLocation.x;
    targetPosition.y = helpRequestLocation.y;
    computePath(myPosition, targetPosition);

    // 4) Vi är nu klara med "vänta på svar"
    waitingForAnswer = false;
    taskCheckButtonPresses.disable();
    setColor(0, 0, 0);
  }
}

  // Check NO button
  if (digitalRead(noButton) == LOW) {
    delay(50);  
    if (digitalRead(noButton) == LOW) {
      setColor(0, 255, 0);  
      Serial.println("No button pressed");

      // skicka help_declined
      StaticJsonDocument<128> noResponse;
      noResponse["type"] = "help_declined";
      noResponse["node_id"] = mesh.getNodeId();
      String msg;
      serializeJson(noResponse, msg);
      mesh.sendSingle(helpRequestNodeId, msg);

      updateMessageDisplay("Help Declined!");

      // nollställa alla dessa variabler
      helpRequestNodeId = 0;
      helpRequestLocation.x = 0;
      helpRequestLocation.y = 0;

      // Vi är nu klara med vänta
      waitingForAnswer = false;
      taskCheckButtonPresses.disable();
      setColor(0, 0, 0);
    }
  }
}

// -----------------------------------------------------------------------------
// Funktion för att kolla om vi sätter igång en process att fråga noder om hjälp
// -----------------------------------------------------------------------------
void startRequestHelpTask() {
  helpAcceptedCount = 0;  // nollställa räknaren
  nodesAsked.clear();     // nollställa noder vi redan frågade
  HelpFound = false;
  waitingForResponse = false;

  // Retrieve all nodes in the mesh
  allNodes = std::vector<uint32_t>(mesh.getNodeList().begin(), mesh.getNodeList().end());
  nodesAsked.push_back(mesh.getNodeId());  // skippa att fråga sig själv
  nodesAsked.push_back(GateWayNode); // skippa att fråga till gateway

  requestHelpTask.enable();
  Serial.println("requestHelpTask enabled");
}

// -----------------------------------------------------------------------------
// Funktion för att kolla om vi skickar all_ready till alla accepterade
// -----------------------------------------------------------------------------
void sendAllReadyMessage() {
  StaticJsonDocument<128> allReadyMsg;
  allReadyMsg["type"] = "all_ready";
  allReadyMsg["node_id"] = mesh.getNodeId();

  String msg;
  serializeJson(allReadyMsg, msg);

  for (uint32_t nodeId : acceptedNodes) {
    mesh.sendSingle(nodeId, msg);
    Serial.printf("Sent ALL_READY message to node %u\n", nodeId);
  }
  AllReady = true;
}

// -----------------------------------------------------------------------------
// Funktion för att kolla om vi kallas i loop om man vill skicka global "help"
// -----------------------------------------------------------------------------
void pressHelpButton() {
  if (digitalRead(helpButton) == LOW) {
    delay(50);
    if (digitalRead(helpButton) == LOW) {
      Serial.println("Help button pressed");
      waitingForHelpButtonPress = true;

      // Skicka en broadcast så att ALLA noder sätter waitingForAnswer = true:
      StaticJsonDocument<128> doc;
      doc["type"] = "global_help_node"; // nytt typnamn
      doc["node_id"] = mesh.getNodeId();
      // Man kan skicka POI:n, 
      doc["location"]["x"] = myPosition.x;  // helpRequestLocation.x
      doc["location"]["y"] = myPosition.y;

      String msg;
      serializeJson(doc, msg);
      mesh.sendBroadcast(msg);

      Serial.println("Broadcasted global_help_node to everyone!");
      updateMessageDisplay("Broadcasted global_help_node to everyone!");
    }
  }
}
// -----------------------------------------------------------------------------
// Funktion för att kolla om jag står precis på en brandpos som inte är tilldelad
// då tar jag uppgiften
// -----------------------------------------------------------------------------
void checkForUnassignedFireAtMyPosition() {
  // Bara om jag är ledig eller patrullerar
    if (currentState != IDLE && currentState != PATROLLING) {
        return;
    }
    // Loopar genom alla activeEvents
    for (auto &evPair : activeEvents) {
        uint32_t eid = evPair.first;
        EventData &ev = evPair.second;

        // 1) Kolla om det är en fire_detected
        if (ev.type == "fire_detected") {
            // 2) Kolla om den inte är assigned/handled
            if (!ev.assigned && !ev.handled) {
                // 3) Kolla om positionen matchar myPosition
                if (ev.location.x == myPosition.x && ev.location.y == myPosition.y) {
                    Serial.printf("[checkForUnassignedFireAtMyPosition] Found unassigned fire event %u at my pos (%d,%d). Taking it!\n",
                                  eid, myPosition.x, myPosition.y);
                    // 4) Ta uppgiften
                    assignTask(eid, ev);

                }
            }
        }
    }
}

// -----------------------------------------------------------------------------
// styr lysdidoden genom färgen
// -----------------------------------------------------------------------------
void setColor(uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(0, strip.Color(red, green, blue));
  strip.show();
}

// -----------------------------------------------------------------------------
// Funktion för att måla en svensk flagga på skärmen
// -----------------------------------------------------------------------------
void drawSwedenFlag(int x, int y, int width, int height) {
  // Blue background
  tft.fillRect(x, y, width, height, TFT_BLUE);

  // Yellow horizontal cross
  int crossThickness = height / 5;  // Adjust thickness
  tft.fillRect(x, y + (height / 2) - (crossThickness / 2), width, crossThickness, TFT_YELLOW);

  // Yellow vertical cross
  tft.fillRect(x + (width / 3) - (crossThickness / 2), y, crossThickness, height, TFT_YELLOW);
}

void drawHorby(){
  xpos = 80;
  ypos = 220;

  uint16_t pngw = 0, pngh = 0;  // To store width and height of image

  int16_t rc = png.openFLASH((uint8_t *)horby, sizeof(horby), pngDraw);

  if (rc == PNG_SUCCESS) {
    Serial.println("Successfully opened png file");
    pngw = png.getWidth();
    pngh = png.getHeight();
    Serial.printf("Image metrics: (%d x %d), %d bpp, pixel type: %d\n", pngw, pngh, png.getBpp(), png.getPixelType());

    tft.startWrite();
    uint32_t dt = millis();
    rc = png.decode(NULL, 0);
    tft.endWrite();
    Serial.print(millis() - dt);
    Serial.println("ms");
    tft.endWrite();

    // png.close(); // Required for files, not needed for FLASH arrays
  }
}

// -----------------------------------------------------------------------------
// Funktion för att skriva ut nodens nuvarande state på skärmen
// -----------------------------------------------------------------------------
void updateStateDisplay() {
  static State previousState = IDLE;
  static FireState previousFireState = MOVE_TO_FIRE;
  static POIState previousPOIState = MOVE_TO_POI;
  static FlammableMaterialState previousFlammableState = MOVE_TO_FLAMMABLE_MATERIAL;

  String state = "";

  // här väljer vilken state som ska visas på skärmen
  switch (currentState) {
    case IDLE:
      state = "IDLE";
      break;
    case PATROLLING:
      state = "PATROLLING";
      break;
    case FIRE:
      switch (fireState) {
        case MOVE_TO_FIRE:
          state = "MOVE_TO_FIRE";
          break;
        case EXTINGUISH_FIRE:
          state = "EXTINGUISH_FIRE";
          break;
        case EXTINGUISH_SMOKE:
          state = "EXTINGUISH_SMOKE";
          break;
      }
      break;
    case POI:
      switch (poiState) {
        case MOVE_TO_POI:
          state = "MOVE_TO_POI";
          break;
        case WAIT_FOR_HELPERS:
          state = "WAIT_FOR_HELPERS";
          break;
        case ASSIST_PERSON:
          state = "ASSIST_PERSON";
          break;
        case WAIT_TO_MOVE_POI:
          state = "WAIT_TO_MOVE_POI";
          break;
        case MOVE_POI_TO_FIRETRUCK:
          state = "MOVE_POI_TO_FIRETRUCK";
          break;
      }
      break;
    case FLAMMABLE_MATERIAL:
      switch (flammableMaterialState) {
        case MOVE_TO_FLAMMABLE_MATERIAL:
          state = "MOVE_TO_FLAMMABLE_MATERIAL";
          break;
        case CARRY_FLAMMABLE_TO_FIRETRUCK:
          state = "CARRY_TO_FIRETRUCK";
          break;
      }
      break;
  }

  // Endast uppdatera skärmen om state ändras 
  if (currentState != previousState ||
      fireState != previousFireState ||
      poiState != previousPOIState ||
      flammableMaterialState != previousFlammableState) {
    
    // Updatera state på skärmen
    tft.fillRect(5, 40, tft.width(), 60, TFT_BLACK); // Clear state area
    tft.setCursor(5, 40);
    tft.setTextColor(TFT_WHITE);
    tft.println("Current state:");

    tft.setCursor(5, 70);
    tft.println(state);

    // Updatera föregående states 
    previousState = currentState;
    previousFireState = fireState;
    previousPOIState = poiState;
    previousFlammableState = flammableMaterialState;
  }
}

// -----------------------------------------------------------------------------
// Funktion för att skriva ut text på skärmen
// -----------------------------------------------------------------------------
void updateMessageDisplay(String message) {
  static String previousMessage = "";


  if (message != previousMessage) {
    
    tft.fillRect(5, 120, tft.width(), 80, TFT_BLACK); // Clear message area
    tft.setCursor(5, 120);
    tft.setTextColor(TFT_WHITE);
    tft.println("Message Received:");

    tft.setCursor(5, 150);
    tft.println(message);

    // Updatera föregående meddelande
    previousMessage = message;
  }
}



// -----------------------------------------------------------------------------
// Setup Funktionen
// -----------------------------------------------------------------------------
void setup() {

  Serial.begin(115200); // initierar seriell kommunikation

  //Sätter I/O pinnar för knappar och lysdioden
  pinMode(yesButton, INPUT_PULLUP);
  pinMode(helpButton, INPUT_PULLUP);
  pinMode(noButton, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  // startar skärmen och inställningar
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), TFT_GREEN);
  tft.setFreeFont(&FreeSans9pt7b);
  updateStateDisplay();
  updateMessageDisplay("System Initialized");
  //drawSwedenFlag(70, 220, 100, 60);
  drawHorby();
  strip.begin();  
  strip.show();   

  // Initierar mesh nätverket
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);

  // Lägger till Tasks i schemaläggaren
  userScheduler.addTask(stateMachineTask);
  userScheduler.addTask(sendPositionTask);
  userScheduler.addTask(taskCheckButtonPresses);
  userScheduler.addTask(requestHelpTask);
  userScheduler.addTask(sendCurrentStateTask);
  stateMachineTask.enable();
  sendPositionTask.enable();

  Serial.println("Node started and ready to receive tasks!");
  delay(100);

}

// -----------------------------------------------------------------------------
// Loop funktionen
// -----------------------------------------------------------------------------
void loop() {
  mesh.update();
}
