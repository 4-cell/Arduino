// PS/2 Keyboard AA Detection
// Detects when keyboard sends 0xAA (self-test passed signal)

#define DATA_PIN 2    // PS/2 Data line (use interrupt-capable pin)
#define CLOCK_PIN 3   // PS/2 Clock line (use interrupt-capable pin)

volatile int bit_count = 0;
volatile unsigned int incoming_data = 0;
volatile bool data_ready = false;

void setup() {
  Serial.begin(9600);
  
  // Set pins as inputs with pull-up resistors
  pinMode(DATA_PIN, INPUT_PULLUP);
  pinMode(CLOCK_PIN, INPUT_PULLUP);
  
  // Attach interrupt to clock pin (falling edge triggered)
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), ps2_interrupt, FALLING);
  
  Serial.println("PS/2 Keyboard AA Detector Started");
  Serial.println("Waiting for keyboard startup...");
  Serial.println("Expected sequence: 0xAA (self-test passed)");
  Serial.println();
}

void loop() {
  if (data_ready) {
    data_ready = false;
    
    // Extract the 8 data bits (ignore start, parity, and stop bits)
    unsigned char received_byte = (incoming_data >> 1) & 0xFF;
    
    // Print received byte in hex format
    Serial.print("Received: 0x");
    if (received_byte < 16) Serial.print("0");
    Serial.print(received_byte, HEX);
    Serial.print(" (");
    Serial.print(received_byte);
    Serial.print(")");
    
    // Check if it's the AA signal
    if (received_byte == 0xAA) {
      Serial.println(" <-- KEYBOARD SELF-TEST PASSED (AA)!");
      Serial.println("*** Keyboard startup successful! ***");
    } else {
      Serial.println();
    }
    
    // Reset for next byte
    bit_count = 0;
    incoming_data = 0;
  }
  
  // Small delay to prevent overwhelming serial output
  delay(10);
}

void ps2_interrupt() {
  // Read the data line
  int data_bit = digitalRead(DATA_PIN);
  
  // Store the bit in our data buffer
  // Alternative to |= operator: incoming_data = incoming_data | (data_bit << bit_count);
  incoming_data = incoming_data | (data_bit << bit_count);
  bit_count++;
  
  // PS/2 sends 11 bits total: start(0) + 8 data + parity + stop(1)
  if (bit_count >= 11) {
    data_ready = true;
  }
}

// Alternative version with parity checking
void ps2_interrupt_with_parity() {
  static unsigned char parity = 1;  // Odd parity
  
  int data_bit = digitalRead(DATA_PIN);
  
  if (bit_count == 0) {
    // Start bit (should be 0)
    if (data_bit != 0) {
      // Invalid start bit, reset
      bit_count = 0;
      incoming_data = 0;
      parity = 1;
      return;
    }
  } else if (bit_count >= 1 && bit_count <= 8) {
    // Data bits
    // Alternative to |= operator: incoming_data = incoming_data | (data_bit << (bit_count - 1));
    incoming_data = incoming_data | (data_bit << (bit_count - 1));
    // Alternative to ^= operator: parity = parity ^ data_bit;
    parity = parity ^ data_bit;  // Update parity
  } else if (bit_count == 9) {
    // Parity bit
    if (data_bit != parity) {
      Serial.println("Parity error detected!");
    }
  } else if (bit_count == 10) {
    // Stop bit (should be 1)
    if (data_bit == 1) {
      data_ready = true;
    } else {
      Serial.println("Invalid stop bit!");
    }
    parity = 1;  // Reset parity for next byte
  }
  
  bit_count++;
  
  if (bit_count > 10) {
    if (!data_ready) {
      // Reset if we didn't get valid data
      bit_count = 0;
      incoming_data = 0;
      parity = 1;
    }
  }
}
