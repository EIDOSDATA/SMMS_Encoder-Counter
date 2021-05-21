#define DIVISOR 5
#define INSUARANCE 6

int CDS1 = A4;
int CDS2 = A5;
//int CDS3 = A2;
//int CDS4 = A3;
int CDSValue1;
int CDSValue2;

int DMI = 3;
int RELEASE = 2;

char ReadChar;
int divisor_count;

int32_t shoot_mills;
int32_t mils;
bool flag=false;
//bool func_flag=false;
//int flag = 0;
int32_t stacount=0;

void func_flag() {
  CDSValue1 += analogRead(CDS1);
  CDSValue2 += analogRead(CDS2);
  divisor_count++;
//  Serial.println(divisor_count);

  if (divisor_count >= DIVISOR) {
    Serial.print(CDSValue1 / divisor_count);
    Serial.print(",");
    Serial.println(CDSValue2 / divisor_count);

    CDSValue1 = 0;
    CDSValue2 = 0;
    divisor_count = 0;
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(RELEASE, OUTPUT);
  digitalWrite(RELEASE, LOW);

  pinMode(INSUARANCE,OUTPUT);
  digitalWrite(INSUARANCE,HIGH);

  pinMode(DMI, INPUT); //INPUT or
  attachInterrupt(digitalPinToInterrupt(DMI), func_flag, RISING);
  divisor_count = 0;
  shoot_mills = millis();
}

void loop() {
//  int current = millis();
//  if( current - shoot_mills > 250){
//    shoot_mills = current;
//
//     digitalWrite(INSUARANCE, HIGH);
//     digitalWrite(RELEASE, HIGH);
//     flag=true;
//     mils=millis();
//  }
  
  //delayMicroseconds(1);
  if(flag && millis()-mils>=100)
  {
    digitalWrite(RELEASE, 0);
    flag = false;
    digitalWrite(INSUARANCE, 1);
//    if(flag && millis()-mils>=100)
//    {
//
//    }600
  }
}


void serialEvent()
{
  while (Serial.available() > 0)
  {
    ReadChar = (char)Serial.read();
    if (ReadChar == 's')  //  || (ReadChar == 'S')
    {
      if(flag){ //error 
        digitalWrite(RELEASE, LOW);
        delay(1);
      }
      digitalWrite(INSUARANCE, HIGH);
      digitalWrite(RELEASE, HIGH);
      flag=true;
      mils=millis();
//      Serial.println("shoot!");
    } else {
      Serial.println("unvalid key received");
    }
  }
}
