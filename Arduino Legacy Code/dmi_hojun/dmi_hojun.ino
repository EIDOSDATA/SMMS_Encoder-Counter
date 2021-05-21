#define WHEEL_RADIUS                  0.275  //0.2805 for twizzy //0.324 //kona  //0.275  // 7.5 inch radius for morning
#define DMI_TICKS_PER_REVOLUTION      1000

#define TARGET_DISTANCE               10.0//이번 트위지에선 5.0m, 이전에는 10.0m//10.0
#define TARGET_PULSE_NUMBER           10

#define DIVISOR                       5
int EncoderPulseCount=0,staPsCnt=0;
int pulse_number = 100;
int ENCODER_PULSE = 3;
int DMI_SIGNAL = 4;

#define DELAY_PERIOD 1

#define diameter (2 * PI * WHEEL_RADIUS)
#define rotationForShoot ((TARGET_DISTANCE/diameter))
#define pulse_number (int)((((rotationForShoot * DMI_TICKS_PER_REVOLUTION) / TARGET_PULSE_NUMBER)/DIVISOR))

void OccurEncoderPulse()
{
    EncoderPulseCount++;
}


void setup() {
  Serial.begin(115200);
  Serial.println("Started!");
  // put your setup code here, to run once:
  pinMode(DMI_SIGNAL, OUTPUT);
  digitalWrite(DMI_SIGNAL,LOW);
  pinMode(ENCODER_PULSE, INPUT);
  
  EncoderPulseCount = 0;  
  Serial.println(pulse_number);

  
  attachInterrupt(digitalPinToInterrupt(ENCODER_PULSE), OccurEncoderPulse, FALLING);
}

void loop() {
  if(EncoderPulseCount >= pulse_number){
    EncoderPulseCount -= pulse_number;
    digitalWrite(DMI_SIGNAL,HIGH);
    delayMicroseconds(1);
    digitalWrite(DMI_SIGNAL,LOW);
  }
//    delayMicroseconds(1);
}
