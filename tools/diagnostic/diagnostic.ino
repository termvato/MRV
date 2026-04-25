// Simple Teensy 4.0 diagnostic sketch
// Upload via Arduino IDE to verify the board works independently of PlatformIO.
// Expected output (115200 baud):
//   MRV diagnostic — Teensy 4.0
//   Loop 1
//   Loop 2
//   ...  (LED blinks each iteration)

void setup() {
    pinMode(13, OUTPUT);
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("MRV diagnostic — Teensy 4.0");
}

uint32_t count = 0;

void loop() {
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
    Serial.print("Loop ");
    Serial.println(++count);
}
