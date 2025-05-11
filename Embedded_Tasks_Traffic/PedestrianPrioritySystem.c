#include <wiringPi.h>
#include <stdio.h>

// Define GPIO pins using WiringPi pin numbering
#define RED 0
#define YELLOW 2
#define GREEN 3
#define TRIG 4
#define ECHO 5

// Global variables for tracking pedestrian detection and timing
unsigned long last_ultrasonic_check = 0;
unsigned long pedestrian_end_time = 0;
unsigned long pedestrian_block_end = 0;
int pedestrian_count = 0;

// Setup function to initialize GPIO modes
void setup() {
    wiringPiSetup();  // Initialize WiringPi

    // Set pin modes for traffic lights
    pinMode(RED, OUTPUT);
    pinMode(YELLOW, OUTPUT);
    pinMode(GREEN, OUTPUT);

    // Set pin modes for ultrasonic sensor
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    // Ensure all lights are initially off
    digitalWrite(RED, LOW);
    digitalWrite(YELLOW, LOW);
    digitalWrite(GREEN, LOW);
}

// Function to get distance from ultrasonic sensor
float get_distance() {
    // Send 10µs trigger pulse
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    // Wait for echo to go high
    while (digitalRead(ECHO) == LOW);
    long startTime = micros();

    // Wait for echo to go low again
    while (digitalRead(ECHO) == HIGH);
    long travelTime = micros() - startTime;

    // Calculate distance in cm
    float distance = travelTime * 0.034 / 2;
    return distance;
}

// Function to check for pedestrian using ultrasonic sensor
void check_ultrasonic() {
    unsigned long now = millis();

    // Only check if not in block time and after 2 seconds since last check
    if (now > pedestrian_block_end && now - last_ultrasonic_check >= 2000) {
        last_ultrasonic_check = now;

        float distance = get_distance();  // Measure distance
        printf("Distance: %.2f cm\n", distance);

        // If object detected within 10 cm
        if (distance < 10.0) {
            pedestrian_count++;
            pedestrian_end_time = now + 6000;  // Keep red for 6 sec
            printf("Pedestrian detected! Count=%d\n", pedestrian_count);

            // If more than 3 pedestrians detected in short time, block detection
            if (pedestrian_count >= 4) {
                pedestrian_block_end = now + 20000;  // Block for 20 sec
                pedestrian_count = 0;
                printf("Pedestrian Detection Blocked for 20 sec\n");
            }
        }
    }
}

// Traffic light control functions
void traffic_red() {
    digitalWrite(RED, HIGH);
    digitalWrite(YELLOW, LOW);
    digitalWrite(GREEN, LOW);
}

void traffic_yellow() {
    digitalWrite(RED, LOW);
    digitalWrite(YELLOW, HIGH);
    digitalWrite(GREEN, LOW);
}

void traffic_green() {
    digitalWrite(RED, LOW);
    digitalWrite(YELLOW, LOW);
    digitalWrite(GREEN, HIGH);
}

void clear_all() {
    digitalWrite(RED, LOW);
    digitalWrite(YELLOW, LOW);
    digitalWrite(GREEN, LOW);
}

// Function to run one full traffic light cycle
void run_traffic_cycle() {
    unsigned long now = millis();

    // If pedestrian mode is active, keep red light on
    if (now < pedestrian_end_time) {
        traffic_red();
        printf("Pedestrian Priority Mode - RED ON\n");

        // Keep checking ultrasonic even during pedestrian mode
        while (millis() < pedestrian_end_time) {
            check_ultrasonic();
            delay(100);
        }
        return;  // Exit to resume normal traffic cycle
    }

    // Normal RED phase (4 seconds)
    traffic_red();
    for (int i = 0; i < 40; i++) {
        check_ultrasonic();
        delay(100);
    }

    // YELLOW phase (2 seconds)
    traffic_yellow();
    for (int i = 0; i < 20; i++) {
        check_ultrasonic();
        delay(100);
    }

    // GREEN phase (4 seconds)
    traffic_green();
    for (int i = 0; i < 40; i++) {
        check_ultrasonic();
        delay(100);
    }

    // Turn off all lights
    clear_all();
}

// Main function
int main() {
    setup();  // Initialize everything

    // Run traffic system in an infinite loop
    while (1) {
        check_ultrasonic();    // Check for pedestrians regularly
        run_traffic_cycle();   // Run traffic light cycle
    }

    return 0;
}
