// RGB LED pins
const int redPin = 28;
const int greenPin = 30;
const int bluePin = 32;

// Button input pin
const int buttonPin = 3;

// Analog sensor pin
const int analogPin = A3;

// Variables to manage button state
int buttonPushCounter = 0;
int buttonState = 0;
int lastButtonState = 0;

// Variables for analog input
int val = 0;
float voltage = 0.0;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  pinMode(buttonPin, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  // Read the button state
  buttonState = digitalRead(buttonPin);

  // Read and print analog voltage
  val = analogRead(analogPin);
  voltage = (val / 1023.0) * 5.0;
  Serial.print("Analog input value = ");
  Serial.print(val);
  Serial.print(", Voltage = ");
  Serial.println(voltage);

  // Check if button was pressed (with debouncing)
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
      buttonPushCounter++;
      Serial.println("Button Pressed");
      Serial.print("Press Count: ");
      Serial.println(buttonPushCounter);
    }
    delay(50); // Debounce delay
  }
  lastButtonState = buttonState;

  // Determine the color mode
  int mode = buttonPushCounter % 2;

  // Set LED color based on mode
  switch (mode) {
    case 0: setColor(255, 0, 0); break;        // Red
    case 1: setColor(0, 255, 0); break;        // Green
   
  }

  delay(200); // Small delay to reduce flickering
}

// Function to set the RGB LED color
void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}
