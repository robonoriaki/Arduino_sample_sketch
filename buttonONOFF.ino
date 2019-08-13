/*Take the state of a button and show you that on the serial monitor.*/
/*Get button state and display on serial monitor.*/

//Pull UP
const int buttonON = LOW;
const int buttonOFF = HIGH;

const int button1Pin = 2;

int button1State = HIGH;

const int ledPin = 13;

void setup() {
  Serial.begin(9600);
  
  pinMode(button1Pin, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop(){
  Serial.print("button: ");
  Serial.print(button1State);
  Serial.print("\n");
  
  //read the state of the button1 value
  button1State = digitalRead(button1Pin);
  
  if(button1State == buttonON){
    digitalWrite(ledPin, LOW);
    delay(500);
  }else{
    digitalWrite(ledPin, HIGH);
    delay(500);
  }
}

