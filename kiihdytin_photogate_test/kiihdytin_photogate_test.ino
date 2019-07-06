const int latchPin = 4;
const int clockPin = 3;
const int dataPin = 2;

short readings[8] = {};

// 1 = relay on
// 0 = relay off
// during the actual shifting the bits are inverted to 
// account for the npn pnp mess up
byte relay_statuses = 0;

void setup()
{
  // put your setup code here, to run once:
  // set mux select pins as output
  DDRB |= 0b00000111;   // set PORTB0, PORTB1, PORTB2 as outputs (pins 8, 9, 10)
  pinMode(A0, INPUT);

  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  
  Serial.begin(115200);
}

int counter = 0;
int timeout_ms = 1000;

void loop()
{
  // pulse magnet 0 on at startup
  relay_statuses = 0;
  set_relay_bit(0, 1);
  update_relays();
  delay(100);
  relay_statuses = 0;
  update_relays();

  int m0_start = millis(); // first magnet starting time

  // set magnet 1 on and shut off the others
  relay_statuses = 0;
  set_relay_bit(1, 1);
  update_relays();

  int m1_start = millis();
  
  while (millis() - m1_start < timeout_ms)
  {
    read_sensors();
    if (readings[0] < 100)
    {
      Serial.println("Sensor 0 triggered.");
      break;
    }
  }

  // set magnet 2 on and shut off the others
  relay_statuses = 0;
  set_relay_bit(2, 1);
  update_relays();

  int m2_start = millis();
  
  while (millis() - m2_start < timeout_ms)
  {
    read_sensors();
    if (readings[1] < 100)
    {
      Serial.println("Sensor 1 triggered.");
      break;
    }
  }

  // set magnet 3 on and shut off the others
  relay_statuses = 0;
  set_relay_bit(3, 1);
  update_relays();

  int m3_start = millis();
  
  while (millis() - m3_start < timeout_ms)
  {
    read_sensors();
    if (readings[2] < 100)
    {
      Serial.println("Sensor 2 triggered.");
      break;
    }
  }

  // set magnet 4 on and shut off the others
  relay_statuses = 0;
  set_relay_bit(4, 1);
  update_relays();

  int m4_start = millis();
  
  while (millis() - m4_start < timeout_ms)
  {
    read_sensors();
    if (readings[3] < 100)
    {
      Serial.println("Sensor 3 triggered.");
      break;
    }
  }

  // set magnet 5 on and shut off the others
  relay_statuses = 0;
  set_relay_bit(5, 1);
  update_relays();

  int m5_start = millis();
  
  while (millis() - m5_start < timeout_ms)
  {
    read_sensors();
    if (readings[4] < 100)
    {
      Serial.println("Sensor 4 triggered.");
      break;
    }
  }

  // shut down all relays
  relay_statuses = 0;
  update_relays();

  delay(300);
  exit(0);
}

void set_relay_bit(int index, int state)
{
  relay_statuses &= ~(1 << index); // clear bit
  relay_statuses |= ((state ? 1 : 0) << index); // set bit to requested state
}

void update_relays()
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, ~relay_statuses);
  digitalWrite(latchPin, HIGH);
}

void read_sensors()
{
  for (int i = 0; i < 8; i++)
  {
    PORTB &= ~0b00000111;
    PORTB |= i;
        
    int r = analogRead(A0);
    readings[i] = r;    
  }
}
