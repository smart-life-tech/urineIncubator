#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <PID_v1.h>

#define RELAY_PIN 2
#define BUZZER_PIN 3
#define LED_ACTIVE_PIN 4
#define LED_ALARM_PIN 5
#define BUTTON_UP_PIN 6
#define BUTTON_DOWN_PIN 7
#define BUTTON_SET_PIN 8
#define ONE_WIRE_BUS 9

// Define variables for PID
double setpoint, input, output;
double Kp = 2, Ki = 5, Kd = 1; // PID constants
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

LiquidCrystal_I2C lcd(0x27, 16, 2); // Address 0x27, 16 columns, 2 rows

int buttonStateUp = 0;
int buttonStateDown = 0;
int buttonStateSet = 0;
int lastButtonStateSet = 0;

void setup()
{
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_ACTIVE_PIN, OUTPUT);
  pinMode(LED_ALARM_PIN, OUTPUT);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SET_PIN, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(1000);     // 1 second sample time
  pid.SetOutputLimits(0, 255); // Output limits for the relay

  // Read setpoint from EEPROM
  setpoint = EEPROM.read(0);

  // Start temperature sensors
  sensors.begin();

  // Start serial communication
  Serial.begin(9600);
  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  int deviceCount = sensors.getDeviceCount();
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");
  Serial.println("");
}

void loop()
{
  buttonStateUp = digitalRead(BUTTON_UP_PIN);
  buttonStateDown = digitalRead(BUTTON_DOWN_PIN);
  buttonStateSet = digitalRead(BUTTON_SET_PIN);

  if (buttonStateUp == LOW)
  {
    delay(100);
    // Increment setpoint when SET button is pressed
    setpoint += 0.1;
    // Save setpoint to EEPROM
    EEPROM.update(0, setpoint);
    Serial.println(setpoint);
  }
  if (buttonStateDown == LOW)
  {
    delay(100);
    // Increment setpoint when SET button is pressed
    setpoint -= 0.1;
    // Save setpoint to EEPROM
    EEPROM.update(0, setpoint);
    Serial.println(setpoint);
  }

  // Read temperatures from sensors and average them
  sensors.requestTemperatures();
  float temp1 = sensors.getTempCByIndex(0);
  float temp2 = sensors.getTempCByIndex(1);
  // input = (temp1 + temp2) / 2.0;
  input = map(analogRead(0), 0, 1023, 40, 70);
  // Compute PID control signal
  pid.Compute();

  // Activate/deactivate relay based on PID output
  if (output > 0)
  {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_ACTIVE_PIN, HIGH);
  }
  else
  {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_ACTIVE_PIN, LOW);
  }

  // Check for over-temperature condition
  if (input > setpoint)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_ALARM_PIN, HIGH);
  }
  else
  {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_ALARM_PIN, LOW);
  }

  // Print values to LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Setpoint: ");
  lcd.print(setpoint);
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(input);
  lcd.print("C");
  if (setpoint > 65)
    setpoint = 60;
  if (setpoint < 55)
    setpoint = 55;
  Serial.print("working");
  delay(5000); // Delay for 0.5 second
}
