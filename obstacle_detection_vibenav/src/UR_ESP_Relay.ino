
// ultrasonic pins
#define TRIG_FRONT 4
#define ECHO_FRONT 5

// Relay
#define Relay_Front 19


void setup() {
  Serial.begin(115200); //for debugging
  delay(1000);           // Give time for serial to connect
  Serial.println("ESP32 started!");

  //setup ultrasonic pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  //relay
  pinMode(Relay_Front, OUTPUT);
}

void activateRelay(int RELAY_PIN) {
  digitalWrite(RELAY_PIN, LOW); // Turn on the relay
}

void deactivateRelay(int RELAY_PIN) {
  digitalWrite(RELAY_PIN, HIGH); // Turn off the relay
}

// Function to measure disance
long getDistance(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034/2; //to cm
  return distance;

}

long getFilteredDistance(int trigPin, int echoPin) {
  long total = 0;
  for (int i = 0; i < 5; i++) {
    total += getDistance(trigPin, echoPin);
    delay(50);
  }
  return total / 5;
}

void loop() {
  long frontDist = getFilteredDistance(TRIG_FRONT, ECHO_FRONT);  // Use filtered!
  Serial.print("Filtered Distance: ");
  Serial.println(frontDist);

  if (frontDist > 0 && frontDist < 50) {
    activateRelay(Relay_Front);
  } else {
    deactivateRelay(Relay_Front);
  }
  
  delay(200); // Small delay to avoid flooding
}

// Follow the same for both side sensors & relays
