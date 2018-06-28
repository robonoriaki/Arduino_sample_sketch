#include <Servo.h>
Servo myServo; // create a servo object

float angle; // variable to hold the angle for the servo motor
int flag = 0;

void setup() {
     pinMode(2,INPUT) ;    //スイッチに接続ピンをデジタル入力に設定
     pinMode(4,INPUT) ;    //スイッチに接続ピンをデジタル入力に設定
     pinMode(8,OUTPUT) ;  //ＬＥＤに接続ピンをデジタル出力に設定
     myServo.attach(9); // attaches the servo on pin 9 to the servo object
     Serial.begin(9600);
}
void loop() {
  digitalWrite(8,LOW) ;

   if(flag == 0){
      angle = 40;

      flag = 1;
   }
   
     if (digitalRead(2) == HIGH) {     //スイッチの状態を調べる
          digitalWrite(8,HIGH) ;      //スイッチが押されているならLEDを点灯
          // set the servo position
          if(angle != 40){
            angle = angle - 0.1;
          }
          //myServo.write(40);
          
          delay(5);
     } else if(digitalRead(4) == HIGH) {
          digitalWrite(8,LOW) ;       //スイッチが押されていないならLEDを消灯
          // set the servo position
          if(angle != 180){
            angle = angle + 0.1;
          }
          //myServo.write(100);
          delay(5);
     }
   myServo.write(angle);
   Serial.print("angle = ");
   Serial.print(angle);
   Serial.print("\n");
   
   //delay(10);
}
