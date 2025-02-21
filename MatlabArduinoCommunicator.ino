#define NUM_BANDS 6  // Adjust this to match bandNum in MATLAB
int pwmPins[NUM_BANDS] = {3, 5, 6, 9, 10, 11};  // Assign PWM pins

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_BANDS; i++) {
    pinMode(pwmPins[i], OUTPUT);
  }
  Serial.begin(115200); 
  Serial.println("START"); // Use println() to send a proper line-terminated signal
}

void loop() {
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n'); // Read entire message
    int bandLevels[NUM_BANDS];  // Store brightness levels

    int index = 0;
    char *token = strtok(receivedData.c_str(), ",");  // Tokenize message
    while (token != NULL && index < NUM_BANDS) {
      bandLevels[index] = atoi(token);  // Convert to int
      token = strtok(NULL, ",");
      index++;
    }
    for (int k=0; k < NUM_BANDS; k++) {
      analogWrite(pwmPins[k],bandLevels[k]);
  }
}
}
