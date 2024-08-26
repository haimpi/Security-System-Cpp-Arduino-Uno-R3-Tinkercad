
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define ECHO_PIN 11
#define TRIG_PIN 12
#define SERVO_PIN 13
#define PIR_PIN A0 
#define RED_LED_PIN A1
#define GREEN_LED_PIN A3
#define BLUE_LED_PIN A2

#define DISTANCE_THRESHOLD 50


LiquidCrystal_I2C lcd(0x20,16,2);

Servo myServo;

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {10, 9, 8, 7}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {6, 5, 4, 3}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);


void lcdSetup() {
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Enter password:");
    lcd.setCursor(0, 1);
}

void ultrasonicSetup() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void PIRSetup()
{
    pinMode(PIR_PIN, INPUT);
}

void rgbLedSetup() {
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);  
}

void buttonSetup()
{
    cli();

  	DDRD &= ~(1<<PD2);
  	EICRA |= 1<<ISC00;
    EICRA |= 1<<ISC01;
  	EIMSK |= 1<<INT0; 
  	
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    OCR2A = 124; // Set compare match register for 1kHz increments
    TCCR2A |= (1 << WGM21); // CTC mode
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler
    TIMSK2 |= (1 << OCIE2A); // Enable timer compare interrupt
   
    sei();
}

void setup() {
    Serial.begin(9600);
    lcdSetup();
    ultrasonicSetup();
    rgbLedSetup();
  	buttonSetup();
    myServo.attach(SERVO_PIN);
    myServo.write(0);
    delay(1000);
}

unsigned int timerCount = 0;
bool buttonPressed = false;
String enteredPassword = "";  // To store the entered password
const String correctPassword = "1234";  // The correct password
int buttonPressCount = 0;
bool lockedUp = true;
bool closeToTheSensor = false;
bool pirDetected = false;

ISR(INT0_vect){
  	buttonPressed = true;
  	buttonPressCount++;
}

ISR(TIMER2_COMPA_vect) {
    if (buttonPressCount > 0 && lockedUp==true) {
        timerCount++;
        if (buttonPressCount == 1 && timerCount >= 125) {
            digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
            timerCount = 0;
        }
        else if (buttonPressCount == 2 && timerCount >= 64) {
            digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
            timerCount = 0;
        }
        else if (buttonPressCount > 2 && timerCount>=32) {
            digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
            timerCount = 0;
        }
    }
}
                
void loop()
{
    char key = keypad.getKey();
    if (key != NO_KEY)
    {
        lcd.print(key);
        enteredPassword+=key;
    }
  
	if(buttonPressed==true)
    {      
   		if (enteredPassword == correctPassword)
        {
          if(lockedUp == true)
          {
            unlock();
            lockedUp = false;
            jsonPrint("locking", "false");
          }
          else
          {
            lockedUp = true;
            lock();
            jsonPrint("locking", "true");
          }
          
          buttonPressCount = 0;
        }
        else
        {
          jsonPrint("worngPassord",String(buttonPressCount));
        }
    
      	
      enteredPassword="";
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Enter password:");
      lcd.setCursor(0, 1);
      buttonPressed=false;
    }
  if(lockedUp)
  {
  	checkPIR();
  }
  
  checkDistance();
}

void unlock()
{
  	for(int i = 0; i <= 90; i++)
    {
    	myServo.write(i);
      	delay(10);
    }
  
  	digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
}

void lock()
{
    for(int i = 90; i >= 0; i--)
    {
    	myServo.write(i);
      	delay(10);
    }
  
  	digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
}

void jsonPrint(String key, String value) {
  // Format the key-value pair as JSON
  String jsonString = "{\"" + key + "\": \"" + value + "\"}";
  
  // Send the JSON string over the serial port
  Serial.println(jsonString);
}

void checkPIR()
{
    int pirValue = analogRead(PIR_PIN);
  	if(pirValue > 512 && !pirDetected)
    {
      jsonPrint("message", "Someone in the room");
      pirDetected = true;
    }
  
  if(pirValue < 512)
  {
    pirDetected = false;
  }
}

void checkDistance()
{
   long distance = measureDistance();
  // Check if distance is below the threshold and the flag is false
  if (distance < DISTANCE_THRESHOLD && !closeToTheSensor) {
    // Print a message in JSON format
    jsonPrint("message", "Someone crossed the threshold");
    
    // Set the flag to true to prevent repeated messages
    closeToTheSensor = true;
  }
  
  // Check if distance is above the threshold to reset the flag
  if (distance > DISTANCE_THRESHOLD) {
    closeToTheSensor = false;
  }
}

long measureDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    long distance = (duration / 2) / 29.1;
    return distance;
}