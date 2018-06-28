//********************************
//加速度センサの値を取得するプログラム2
//********************************
//1 -> Vcc
//2 -> +
//3 -> GND
//5 -> GND
//6 -> x axis
//7 -> y axis
//8 -> z axis

void setup()
{
// シリアルモニターの初期化をする
Serial.begin(9600) ;
}
void loop()
{
int i ;
long x,y,z;
//50回センサ値を読み込んで平均を算出
x=y=z=0;
for (i=0 ; i < 50 ; i++) {
x = x + analogRead(3) ; // Ｘ軸
y = y + analogRead(4) ; // Ｙ軸
z = z + analogRead(5) ; // Ｚ軸
}
x = x / 50 ;
y = y / 50 ;
z = z / 50 ;
int rotateX = (x-277)/2.48 - 90; //角度を求める式
Serial.print("X:") ;
Serial.print(x) ;
Serial.print(" ") ;
Serial.print(rotateX) ;
int rotateY = (y-277)/2.48 - 90;
Serial.print(" Y:") ;
Serial.print(y) ;
Serial.print(" ") ;
Serial.print(rotateY) ;
delay(50) ;
}
