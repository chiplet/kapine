#define N_SENSORS 2
#define N_MAGNETS 4

// sensors
const int photogate_sensor_pins[] = {A0, A1};
const int photogate_led_pins[] = {2, 3};
int readings[N_SENSORS] = {0};
const int photogate_threshold = 200;

const int electromagnet_pins[] = {5, 6, 7, 8};

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < N_SENSORS; i++)
  {
    pinMode(photogate_sensor_pins[i], INPUT);
    pinMode(photogate_led_pins[i], OUTPUT);
  }

  for (int i = 0; i < N_MAGNETS; i++)
  {
      pinMode(electromagnet_pins[i], OUTPUT);
      digitalWrite(electromagnet_pins[i], LOW);
  }
  Serial.begin(115200);
}

// accelerate until sensor_pin is pulled down or duration has elapsed
void ACCELERATE(int pin, int duration, int sensor_pin)
{
  int start = millis();
  digitalWrite(pin, HIGH); // ACCELERATE !!!
  while (millis() - start < duration)
  {
    // start listening to photogate
    int reading = analogRead(sensor_pin);
    if (reading < photogate_threshold)
    {
      break;
    }
  }
  digitalWrite(pin, LOW); // ACCELERATE !!!
}

// accelerate until sensor_pin is pulled down or duration has elapsed
void ACCELERATE2()
{
  digitalWrite(electromagnet_pins[3], HIGH);
  delay(30);
  digitalWrite(electromagnet_pins[3], LOW);

  // magnet 2
  const int timeout = 300;
  int start = millis();
  // digitalWrite(electromagnet_pins[3], HIGH); // ACCELERATE !!!
  digitalWrite(electromagnet_pins[2], HIGH); // ACCELERATE !!!

  while (millis() - start < timeout)
  {
    // start listening to photogate
    int reading = analogRead(photogate_sensor_pins[1]);
    if (reading < photogate_threshold)
    {
      break;
    }
  }
  Serial.println(String()+"time elapsed"+(millis() - start));
  // digitalWrite(electromagnet_pins[3], LOW);
  digitalWrite(electromagnet_pins[2], LOW);

  digitalWrite(electromagnet_pins[1], HIGH);
  delay(50);
  digitalWrite(electromagnet_pins[1], LOW);

  // magnets 0 and 1
  start = millis();
  // digitalWrite(electromagnet_pins[1], HIGH); // ACCELERATE !!!
  digitalWrite(electromagnet_pins[0], HIGH); // ACCELERATE !!!

  while (millis() - start < timeout)
  {
    // start listening to photogate
    int reading = analogRead(photogate_sensor_pins[0]);
    if (reading < photogate_threshold)
    {
      break;
    }
  }
  // digitalWrite(electromagnet_pins[1], LOW);
  digitalWrite(electromagnet_pins[0], LOW);
}

int accel_duration = 1000;
void loop() {
  ACCELERATE2();
  exit(0);

  /*
  // ACCELERATE(electromagnet_pin, 40);
  // send data only when you receive data:
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    String command = Serial.readString();

    if (command == "a")
    {
      Serial.print("ACCELERATE!!! ");
      //ACCELERATE(electromagnet_pins[0], accel_duration, A1);
      ACCELERATE2();
      Serial.println("done");
    }
    else if (command == "+")
    {
      Serial.println(String()+"Duration: "+accel_duration);
      accel_duration += 5;
    }
    else if (command == "-")
    {
      Serial.println(String()+"Duration: "+accel_duration);
      accel_duration -= 5;
    }
    else
    {
      Serial.println(String()+"Unknown command: "+command);
    }
  }
  */

  // start listening to photogate (photogate debug leds)
  /*
  for (int i = 0; i < N_SENSORS; i++)
  {
    readings[i] = analogRead(photogate_sensor_pins[i]);
    digitalWrite(photogate_led_pins[i], (readings[i] < photogate_threshold) ? HIGH : LOW); // set photogate led if reading is below the threshold
  }
  */

  // loop at 100 Hz for now
  delay(10);
}
