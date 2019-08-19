//Detects two buttons are pushed. Implemented chattering measures.

//Pull UP
const int buttonON = LOW;
const int buttonOFF = HIGH;

const int button1Pin = 2;
const int button2Pin = 4;

int button1State = 0;
int button2State = 0;

const int ledPin = 13;

void setup() {
  Serial.begin(9600);
  
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop(){
  
  //read the state of the button1 value
  button1State = digitalRead(button1Pin);
  //read the state of the button2 value
  button2State = digitalRead(button2Pin);

  //display on serial monitor
  Serial.print("button1: ");
  Serial.print(button1State);
  Serial.print("     ");
  Serial.print("button2: ");
  Serial.print(button2State);
  Serial.print("\n");

  //pushing button1 and not pushing button2
  if(button1State == buttonON && button2State == buttonOFF){
    //Chattering measures
    delay(10);  //10ms -> 0.010s
    
    if(button1State == buttonON && button2State == buttonOFF){
      digitalWrite(ledPin, LOW);
    }
    
  //not pushing button1 and pushing button2
  }else if(button1State == buttonOFF && button2State == buttonON){
    //Chattering measures
    delay(10);  //10ms -> 0.010s

    if(button1State == buttonOFF && button2State == buttonON){
      digitalWrite(ledPin, LOW);
    }
    
  //pushing button1 and pushing button2 ///////////////////////////////////////////////////
  }else if(button1State == buttonON && button2State == buttonON){
    //Chattering measures and Simultaneous pushing measures
    delay(10);  //10ms -> 0.010s

    if(button1State == buttonON && button2State == buttonON){
      digitalWrite(ledPin, HIGH);
    }
  
  //not pushing button1 and not pushing button2
  }else{
    digitalWrite(ledPin, LOW);
  }
  
}

