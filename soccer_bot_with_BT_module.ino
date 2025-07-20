char t;
int LmotorIN = 8;
int LmotorOUT = 7;
int RmotorIN = 5;
int RmotorOUT = 4;
int LBluelight = 10;
int RBluelight = 11;
int LmotorSpeed = 9;
int RmotorSpeed = 3;
int Speed = 255;
int res = 0;

void setup() {
pinMode(LmotorIN,OUTPUT);   //left motors forward
pinMode(LmotorOUT,OUTPUT);   //left motors reverse
pinMode(RmotorIN,OUTPUT);   //right motors forward
pinMode(RmotorOUT,OUTPUT); //right motors reverse
digitalWrite(LBluelight,OUTPUT);
digitalWrite(RBluelight,OUTPUT);//Led
pinMode(LmotorSpeed,OUTPUT);
pinMode(RmotorSpeed,OUTPUT);

Serial.begin(9600);
 
}
 
void loop() {
  if(Serial.available()){
    t = Serial.read();
    Serial.println(t);
  

switch (t){
  case 'F': forward( );     ; break;
    
  case 'B': backward()      ; break;
  case 'L': left()          ; break;
  case 'R': right()         ; break;
  case 'S': Stop()          ; break;
  case 'W': LEDon()         ; break;
  case 'w': LEDoff()        ; break;
//  case 'G': forwardleft()        ; break;
//  case 'i': forwardRight()        ; break;
//  case 'H': Backwardleft()        ; break;
//  case 'J': BackwardRight()        ; break;



  case '0': res = 100     ; break;
  case '1': res = 110     ; break;
  case '2': res = 115    ; break;
  case '3': res = 118     ; break;
  case '4': res = 120     ; break;
  case '5': res = 128     ; break;
  case '6': res = 135     ; break;
  case '7': res = 140     ; break;
  case '8': res = 150     ; break;
  case '9': res = 160     ; break;
  case 'q': res = 255     ; break;

}
}}
void forward()
{
    digitalWrite(LmotorIN,HIGH);
    digitalWrite(LmotorOUT,LOW);
    digitalWrite(RmotorIN,HIGH);
    digitalWrite(RmotorOUT,LOW);
    analogWrite(LmotorSpeed,Speed);
    analogWrite(RmotorSpeed,Speed);
}

void backward()
{
    digitalWrite(LmotorIN,LOW);
    digitalWrite(LmotorOUT,HIGH);
    digitalWrite(RmotorIN,LOW);
    digitalWrite(RmotorOUT,HIGH);
    analogWrite(LmotorSpeed,Speed);
    analogWrite(RmotorSpeed,Speed);

}
void left()
{
    digitalWrite(LmotorIN,HIGH);
    digitalWrite(RmotorIN,HIGH);
    analogWrite(LmotorSpeed,Speed-res);
    analogWrite(RmotorSpeed,Speed);
}
void right()
{
    digitalWrite(RmotorIN,HIGH);    
    digitalWrite(LmotorIN,HIGH);
    analogWrite(LmotorSpeed,Speed);
    analogWrite(RmotorSpeed,Speed-res);
}

void Stop()
{
    digitalWrite(LmotorIN,LOW);
    digitalWrite(LmotorOUT,LOW);
    digitalWrite(RmotorIN,LOW);
    digitalWrite(RmotorOUT,LOW);
}
void LEDon(){
    digitalWrite(LBluelight,HIGH);
    digitalWrite(RBluelight,HIGH);
}
void LEDoff (){
    digitalWrite(LBluelight,LOW);
    digitalWrite(RBluelight,LOW);
  }

void forwardleft (){
    digitalWrite(LmotorIN,HIGH);
    digitalWrite(LmotorOUT,LOW);
    digitalWrite(RmotorIN,HIGH);
    digitalWrite(RmotorOUT,LOW);
    analogWrite(LmotorSpeed,Speed - res);
    analogWrite(RmotorSpeed,Speed - res);
    

  }
void forwardRight (){
    digitalWrite(LmotorIN,HIGH);
    digitalWrite(LmotorOUT,LOW);
    digitalWrite(RmotorIN,HIGH);
    digitalWrite(RmotorOUT,LOW);
    analogWrite(LmotorSpeed,Speed - res);
    analogWrite(RmotorSpeed,Speed -res);

  }
void Backwardleft (){
    digitalWrite(LmotorIN,LOW);
    digitalWrite(LmotorOUT,HIGH);
    digitalWrite(RmotorIN,LOW);
    digitalWrite(RmotorOUT,HIGH);
    analogWrite(LmotorSpeed,Speed - res);
    analogWrite(RmotorSpeed,Speed -res);

  }
void BackwardRight (){
    digitalWrite(LmotorIN,LOW);
    digitalWrite(LmotorOUT,HIGH);
    digitalWrite(RmotorIN,LOW);
    digitalWrite(RmotorOUT,HIGH);
    analogWrite(LmotorSpeed,Speed -res);
    analogWrite(RmotorSpeed,Speed -res);

  }