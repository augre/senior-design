//Kicsi delayt rakni bele



#define SENS 100
 
 
 int motor1Pin1 = 9; // H-bridge leg 1
 int motor1Pin2 = 8; // H-bridge leg 2
 int enable1 = 10; // H-bridge enable pin
 int motor2Pin1 = 11; // H-bridge leg 1
 int motor2Pin2 = 12; // H-bridge leg 2
 int enable2 = 13;
 
 int flip;

int check()
{
  char dataIn = 'S'; 
  char determinant='A';
  if (Serial.available() > 0)    //Check for data on the serial lines.
  {   
    dataIn = Serial.read();  //Get the character sent by the phone and store it in 'dataIn'.
    if (dataIn == 'F')
    {     
      determinant = 'F';
    }  
    else if (dataIn == 'B')
    { 
      determinant = 'B'; 
    }
    else if (dataIn == 'A')
    { 
      determinant = 'A'; 
    }
    flip=determinant;
  }
  else determinant=flip;
  return determinant;
}

void forward(){
digitalWrite(motor1Pin1, HIGH);
digitalWrite(motor1Pin2, LOW);
digitalWrite(enable1, HIGH);
digitalWrite(motor2Pin1, HIGH);
digitalWrite(motor2Pin2, LOW);
digitalWrite(enable2, HIGH);
}

void leftwheel(char d){
  //1 backwards
  if (d=='b'){
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    digitalWrite(enable2, HIGH);
  }
  //0 forward
  else{
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    digitalWrite(enable2, HIGH);
  }
    
}

void rightwheel(char d){
  //1 backwards
   if (d=='b'){
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(enable1, HIGH);
   }
   //0 forward
   else{
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(enable1, HIGH);
   }
}

void stopm(){
  digitalWrite(enable1, LOW);
  digitalWrite(enable2, LOW);
}




void setup() {
 // set the switch as an input:
 Serial.begin (9600);

// set all the other pins you're using as outputs:
 pinMode(motor1Pin1, OUTPUT);
 pinMode(motor1Pin2, OUTPUT);
 pinMode(enable1, OUTPUT);
 pinMode(motor2Pin1, OUTPUT);
 pinMode(motor2Pin2, OUTPUT);
 pinMode(enable2, OUTPUT);
 
 //feedback LEDs
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);


 }



void loop() {
  
  int det = check();
  
  int sensorValue = analogRead(A0);
  int sensorValue1 = analogRead(A1);
  int sensorValue2 = analogRead(A2);
  int sensorValue3 = analogRead(A3);
  int sensorValue4 = analogRead(A4);
  int sensorValue5 = analogRead(A5);
 
 
  if (det=='F'){
    digitalWrite(6, HIGH);
    stopm();
    leftwheel('f');
    rightwheel('f');
  }
  else if (det=='B'){
    digitalWrite(6, LOW);
    stopm();
    leftwheel('b');
  }
  else if (det=='A'){
     if (((sensorValue<SENS) || (sensorValue1<SENS) || (sensorValue2<SENS) || (sensorValue3<SENS) || (sensorValue4<SENS) || (sensorValue5<SENS))){
      if (sensorValue < SENS){
        //digitalWrite(6, HIGH);
        stopm();
        leftwheel('b');
        rightwheel('f');
      }
     // else digitalWrite(6,LOW);
     
      if (sensorValue1 < SENS){
       // digitalWrite(4, HIGH);
        stopm();
        //leftwheel('f');
        rightwheel('f');
      }
    //  else digitalWrite(4,LOW);
     
      if (sensorValue2 < SENS){
       // digitalWrite(2, HIGH);
        stopm();
        leftwheel('f');
        rightwheel('f');
      }
     // else digitalWrite(2,LOW);
      
      if (sensorValue3 < SENS){
       // digitalWrite(6, HIGH);
        stopm();
        leftwheel('f');
        //rightwheel('f');
      }
      
      
      if (sensorValue4 < SENS){
       // digitalWrite(6, HIGH);
        stopm();
        leftwheel('f');
        rightwheel('b');
      }
      
      if (sensorValue5 < SENS){
       // digitalWrite(6, HIGH);
        stopm();
        leftwheel('f');
        rightwheel('b');
      }
      
      Serial.print(sensorValue);
      Serial.print("\t");
      Serial.print(sensorValue1);
      Serial.print("\t");
      Serial.print(sensorValue2);
      Serial.print("\t");
      Serial.print(sensorValue3);
      Serial.print("\t");
      Serial.print(sensorValue4);
      Serial.print("\t");
      Serial.print(sensorValue5);
      Serial.print("\n");
    }
    else stopm();
  }
 }
