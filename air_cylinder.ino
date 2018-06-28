int flag = 0;

void setup() {
     pinMode(2,INPUT) ;    //スイッチに接続ピンをデジタル入力に設定
     pinMode(4,INPUT) ;    //スイッチに接続ピンをデジタル入力に設定
     pinMode(8,OUTPUT) ;  //ＬＥＤに接続ピンをデジタル出力に設定
     pinMode(12,OUTPUT) ;    //シリンダに接続ピンをデジタル出力に設定
     pinMode(13,OUTPUT) ;    //シリンダに接続ピンをデジタル出力に設定
     Serial.begin(9600);
}
void loop() {
   if(flag == 0){
    digitalWrite(8,LOW) ;
      flag = 1;
   }
   
     if (digitalRead(2) == HIGH) {     //スイッチの状態を調べる
          digitalWrite(8,HIGH) ;      //スイッチが押されているならLEDを点灯
          digitalWrite(12,HIGH) ;
          digitalWrite(13,LOW) ;
     } else if(digitalRead(4) == HIGH) {
          digitalWrite(8,LOW) ;       //スイッチが押されていないならLEDを消灯
          digitalWrite(12,LOW) ;
          digitalWrite(13,HIGH) ;
     }else if(digitalRead(2) == LOW && digitalRead(4) == LOW){
      digitalWrite(12,LOW);
      digitalWrite(13,LOW) ;
     }
   
   
   //delay(10);
}
