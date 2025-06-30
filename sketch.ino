#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin definitions
#define LED1_PIN 2        // Red LED - Task A
#define LED2_PIN 3        // Green LED - Task B  
#define LED3_PIN 6        // Yellow LED - Emergency
#define EMERGENCY_BTN 7   // Emergency button
#define MENU_BTN 8        // Menu button
#define CPU_LED 13        // Built-in LED - CPU load indicator

// LCD initialization
LiquidCrystal_I2C lcdMain(0x27, 16, 2);  // Main status LCD
LiquidCrystal_I2C lcdMenu(0x26, 16, 2);  // Menu navigation LCD

// Task timing variables
unsigned long lastTaskA = 0;
unsigned long lastTaskB = 0;
unsigned long lastTaskC = 0;
unsigned long lastMenuUpdate = 0;
unsigned long systemStartTime = 0;

// Task intervals (ms)
const unsigned long TASK_A_INTERVAL = 1000;  // 1 second
const unsigned long TASK_B_INTERVAL = 2000;  // 2 seconds
const unsigned long TASK_C_INTERVAL = 100;   // 100 ms

// LED states
bool led1State = false;
bool led2State = false;

// System state variables
String lastCommand = "none";
bool emergencyActive = false;
volatile bool emergencyTriggered = false;
volatile unsigned long emergencyTriggerTime = 0;

// Enhanced menu system
int currentMenuPage = 0;
const int MAX_MENU_PAGES = 5;  // Increased menu pages
unsigned long lastMenuButtonPress = 0;
const unsigned long MENU_DEBOUNCE_TIME = 300;

// Serial command buffer
String serialBuffer = "";

// System monitoring
unsigned long taskACount = 0;
unsigned long taskBCount = 0;
unsigned long taskCCount = 0;
unsigned long emergencyCount = 0;
float cpuUsage = 0.0;
unsigned long lastCpuCalc = 0;
unsigned long activeTime = 0;

// Enhanced features
bool systemPaused = false;
int brightness = 255;  // PWM brightness for LEDs
bool autoMode = true;   // Auto task scheduling vs manual

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("MiniOS v1.5 Starting...");
  
  // Initialize pins
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(CPU_LED, OUTPUT);
  pinMode(EMERGENCY_BTN, INPUT_PULLUP);
  pinMode(MENU_BTN, INPUT_PULLUP);
  
  // Initialize LCDs
  lcdMain.init();
  lcdMain.backlight();
  lcdMenu.init();
  lcdMenu.backlight();
  
  // Attach emergency interrupt with proper debouncing
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_BTN), emergencyISR, FALLING);
  
  // Enable internal pullup for emergency button (redundant safety)
  digitalWrite(EMERGENCY_BTN, HIGH);
  
  // Record system start time
  systemStartTime = millis();
  
  // Initial LCD updates
  updateMainLCD();
  updateMenuLCD();
  
  Serial.println("MiniOS v1.5 Ready!");
  Serial.println("Commands: status, ledon, ledoff");
}

void loop() {
  unsigned long currentTime = millis();
  bool cpuActive = false;
  unsigned long loopStart = currentTime;
  
  // Handle emergency interrupt with debouncing
  if (emergencyTriggered && (currentTime - emergencyTriggerTime > 50)) {
    emergencyTask();
    emergencyTriggered = false;
    emergencyCount++;
    cpuActive = true;
  }
  
  // Skip normal tasks if system is paused
  if (!systemPaused) {
    // Task A - LED1 blinker (1 second interval)
    if (autoMode && (currentTime - lastTaskA >= TASK_A_INTERVAL)) {
      taskA();
      lastTaskA = currentTime;
      taskACount++;
      cpuActive = true;
    }
    
    // Task B - LED2 blinker (2 second interval)
    if (autoMode && (currentTime - lastTaskB >= TASK_B_INTERVAL)) {
      taskB();
      lastTaskB = currentTime;
      taskBCount++;
      cpuActive = true;
    }
  }
  
  // Task C - Serial polling (always active)
  if (currentTime - lastTaskC >= TASK_C_INTERVAL) {
    taskC();
    lastTaskC = currentTime;
    taskCCount++;
    cpuActive = true;
  }
  
  // Check menu button
  checkMenuButton();
  
  // Update LCDs periodically
  if (currentTime - lastMenuUpdate >= 250) {  // Faster updates
    updateMainLCD();
    updateMenuLCD();
    lastMenuUpdate = currentTime;
  }
  
  // Calculate CPU usage
  if (currentTime - lastCpuCalc >= 1000) {
    cpuUsage = (activeTime / 10.0);  // Convert to percentage
    activeTime = 0;
    lastCpuCalc = currentTime;
  }
  
  // CPU load indicator with breathing effect
  if (cpuActive) {
    digitalWrite(CPU_LED, HIGH);
    activeTime += (millis() - loopStart);
  } else {
    // Breathing LED when idle
    int breath = (sin((currentTime / 1000.0) * 2 * PI) + 1) * 127;
    analogWrite(CPU_LED, breath);
  }
}

// Task A: Toggle LED1 every second with brightness control
void taskA() {
  led1State = !led1State;
  if (led1State) {
    analogWrite(LED1_PIN, brightness);
  } else {
    analogWrite(LED1_PIN, 0);
  }
}

// Task B: Toggle LED2 every 2 seconds with brightness control
void taskB() {
  led2State = !led2State;
  if (led2State) {
    analogWrite(LED2_PIN, brightness);
  } else {
    analogWrite(LED2_PIN, 0);
  }
}

// Task C: Poll serial for commands
void taskC() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

// Process serial commands - Enhanced command set
void processSerialCommand(String command) {
  command.trim();
  command.toLowerCase();
  
  unsigned long timestamp = millis() - systemStartTime;
  Serial.print("Echo [");
  Serial.print(timestamp);
  Serial.print("ms]: ");
  Serial.println(command);
  
  if (command == "status") {
    lastCommand = "status";
    Serial.println("=== MiniOS v1.5 Status ===");
    Serial.println("LED1: " + String(led1State ? "ON" : "OFF"));
    Serial.println("LED2: " + String(led2State ? "ON" : "OFF"));
    Serial.println("Emergency: " + String(emergencyActive ? "ACTIVE" : "INACTIVE"));
    Serial.println("System: " + String(systemPaused ? "PAUSED" : "RUNNING"));
    Serial.println("Mode: " + String(autoMode ? "AUTO" : "MANUAL"));
    Serial.println("CPU Usage: " + String(cpuUsage, 1) + "%");
    Serial.println("Uptime: " + String(timestamp / 1000) + "s");
    Serial.println("Task Counts - A:" + String(taskACount) + " B:" + String(taskBCount) + " C:" + String(taskCCount));
    Serial.println("Emergency Count: " + String(emergencyCount));
  }
  else if (command == "ledon") {
    lastCommand = "ledon";
    led1State = true;
    analogWrite(LED1_PIN, brightness);
    Serial.println("LED1 turned ON");
  }
  else if (command == "ledoff") {
    lastCommand = "ledoff";
    led1State = false;
    analogWrite(LED1_PIN, 0);
    Serial.println("LED1 turned OFF");
  }
  else if (command == "pause") {
    lastCommand = "pause";
    systemPaused = !systemPaused;
    Serial.println("System " + String(systemPaused ? "PAUSED" : "RESUMED"));
  }
  else if (command == "auto") {
    lastCommand = "auto";
    autoMode = !autoMode;
    Serial.println("Mode: " + String(autoMode ? "AUTO" : "MANUAL"));
  }
  else if (command.startsWith("bright ")) {
    lastCommand = "bright";
    int newBright = command.substring(7).toInt();
    if (newBright >= 0 && newBright <= 255) {
      brightness = newBright;
      Serial.println("Brightness set to: " + String(brightness));
    } else {
      Serial.println("Brightness must be 0-255");
    }
  }
  else if (command == "reset") {
    lastCommand = "reset";
    taskACount = taskBCount = taskCCount = emergencyCount = 0;
    systemStartTime = millis();
    Serial.println("System counters reset");
  }
  else if (command == "help") {
    lastCommand = "help";
    Serial.println("=== MiniOS v1.5 Commands ===");
    Serial.println("status    - Show system status");
    Serial.println("ledon     - Turn LED1 on");
    Serial.println("ledoff    - Turn LED1 off");
    Serial.println("pause     - Toggle system pause");
    Serial.println("auto      - Toggle auto/manual mode");
    Serial.println("bright N  - Set LED brightness (0-255)");
    Serial.println("reset     - Reset counters");
    Serial.println("help      - Show this help");
  }
  else {
    lastCommand = "unknown";
    Serial.println("Unknown command. Type 'help' for commands.");
  }
}

// Emergency task - Enhanced with priority handling
void emergencyTask() {
  emergencyActive = true;
  Serial.println("!!! EMERGENCY TRIGGERED !!!");
  
  // Save current LED states
  bool saved_led1 = led1State;
  bool saved_led2 = led2State;
  
  // Turn off other LEDs during emergency
  analogWrite(LED1_PIN, 0);
  analogWrite(LED2_PIN, 0);
  
  // Rapid blink sequence - 5 times at 100ms intervals
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED3_PIN, HIGH);
    delay(100);
    digitalWrite(LED3_PIN, LOW);
    delay(100);
  }
  
  // Restore LED states
  if (saved_led1) analogWrite(LED1_PIN, brightness);
  if (saved_led2) analogWrite(LED2_PIN, brightness);
  
  emergencyActive = false;
  Serial.println("Emergency task completed - system restored");
}

// Emergency interrupt service routine - Fixed with debouncing
void emergencyISR() {
  static unsigned long lastInterrupt = 0;
  unsigned long currentTime = millis();
  
  // Simple debouncing in ISR
  if (currentTime - lastInterrupt > 200) {
    emergencyTriggered = true;
    emergencyTriggerTime = currentTime;
    lastInterrupt = currentTime;
  }
}

// Check menu button with debouncing
void checkMenuButton() {
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(MENU_BTN);
  unsigned long currentTime = millis();
  
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    if (currentTime - lastMenuButtonPress > MENU_DEBOUNCE_TIME) {
      currentMenuPage = (currentMenuPage + 1) % MAX_MENU_PAGES;
      lastMenuButtonPress = currentTime;
      updateMenuLCD();
    }
  }
  
  lastButtonState = currentButtonState;
}

// Update main LCD display
void updateMainLCD() {
  lcdMain.clear();
  
  // Line 1: LED states
  lcdMain.setCursor(0, 0);
  lcdMain.print("LED1:");
  lcdMain.print(led1State ? "ON " : "OFF");
  lcdMain.print(" LED2:");
  lcdMain.print(led2State ? "ON" : "OFF");
  
  // Line 2: Last command and emergency flag
  lcdMain.setCursor(0, 1);
  lcdMain.print("Cmd:");
  lcdMain.print(lastCommand.substring(0, 8));  // Truncate if too long
  if (emergencyActive) {
    lcdMain.setCursor(15, 1);
    lcdMain.print("!");
  }
}

// Update menu LCD display - Enhanced with more pages
void updateMenuLCD() {
  lcdMenu.clear();
  unsigned long uptime = (millis() - systemStartTime) / 1000;
  
  // Show page indicator
  lcdMenu.setCursor(14, 0);
  lcdMenu.print(String(currentMenuPage + 1) + "/5");
  
  switch (currentMenuPage) {
    case 0:
      // Page 1: Title and uptime
      lcdMenu.setCursor(0, 0);
      lcdMenu.print("MiniOS v1.5");
      lcdMenu.setCursor(0, 1);
      lcdMenu.print("Up: ");
      lcdMenu.print(uptime);
      lcdMenu.print("s ");
      lcdMenu.print(systemPaused ? "PAUSED" : "RUN");
      break;
      
    case 1:
      // Page 2: Task intervals and modes
      lcdMenu.setCursor(0, 0);
      lcdMenu.print("Tasks A:1s B:2s");
      lcdMenu.setCursor(0, 1);
      lcdMenu.print("Mode: ");
      lcdMenu.print(autoMode ? "AUTO" : "MANUAL");
      break;
      
    case 2:
      // Page 3: Task execution counts
      lcdMenu.setCursor(0, 0);
      lcdMenu.print("Exec A:");
      lcdMenu.print(taskACount);
      lcdMenu.print(" B:");
      lcdMenu.print(taskBCount);
      lcdMenu.setCursor(0, 1);
      lcdMenu.print("Shell:");
      lcdMenu.print(taskCCount);
      lcdMenu.print(" Emrg:");
      lcdMenu.print(emergencyCount);
      break;
      
    case 3:
      // Page 4: System performance
      lcdMenu.setCursor(0, 0);
      lcdMenu.print("CPU: ");
      lcdMenu.print(cpuUsage, 1);
      lcdMenu.print("%");
      lcdMenu.setCursor(0, 1);
      lcdMenu.print("Bright: ");
      lcdMenu.print(brightness);
      lcdMenu.print("/255");
      break;
      
    case 4:
      // Page 5: Status indicators
      lcdMenu.setCursor(0, 0);
      lcdMenu.print("LED1:");
      lcdMenu.print(led1State ? "ON " : "OFF");
      lcdMenu.print("LED2:");
      lcdMenu.print(led2State ? "ON" : "OFF");
      lcdMenu.setCursor(0, 1);
      lcdMenu.print("Cmd: ");
      lcdMenu.print(lastCommand);
      if (emergencyActive) {
        lcdMenu.setCursor(15, 1);
        lcdMenu.print("!");
      }
      break;
  }
}