///http://airoboticsandsoon.hatenablog.jp/entry/2017/12/05/213509からプログラムを拝借

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */


/*
  [w、x、y、z]形式の実際の四元数の構成要素を表示する場合は
  「OUTPUT_READABLE_QUATERNION」のコメントを外してください
  （処理などのリモートホストでの解析には最適ではありません）
*/
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

/*
  重力が除去された加速コンポーネントを表示する場合は、「OUTPUT_READABLE_REALACCEL」のコメントを外してください。 
  この加速度基準フレームは方向を補償されないので、+ Xはセンサに応じて常に+ Xで、重力の影響はありません。 向きを補正したい場合は、代わりにOUTPUT_READABLE_WORLDACCELを使用してください。
*/
// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

/*
 重力が取り除かれ、ワールド座標系の基準に合わせて調整された加速度成分を見たい場合は、
 「OUTPUT_READABLE_WORLDACCEL」のコメントを外してください
 （この場合、磁力計は存在しないため、ヨーは初期の向きに相対します）。 いくつかのケースでは非常に便利です。
*/
// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL


/*
   InvenSenseティーポットのデモに使用されている形式に一致する出力が必要な場合は、
  「OUTPUT_TEAPOT」のコメントを外してください
*/
// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT
#define OUTPUT_TEAPOT


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
                        /// DMP initが成功した場合にtrueを設定する
bool dmpReady = false;  // set true if DMP init was successful
                        ///MPUからの実際の割り込みステータスバイトを保持
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
                        ///各デバイス動作後のステータスの復帰（0 =成功、！0 =エラー）  
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
                        ///予想されるDMPパケットサイズ（デフォルトは42バイト）
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
                        ///現在FIFO内にあるすべてのバイト数
uint16_t fifoCount;     // count of all bytes currently in FIFO
                        ///FIFO記憶バッファ
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container ///クォータニオンコンテナ
VectorInt16 aa;         // [x, y, z]            accel sensor measurements ///加速度センサの測定
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements ///無重力加速度センサの測定
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements ///ワールドフレーム加速度センサの測定
VectorFloat gravity;    // [x, y, z]            gravity vector ///重力ベクトル
float euler[3];         // [psi, theta, phi]    Euler angle container ///オイラー角コンテナ
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector ///ヨー/ピッチ/ロールコンテナと重力ベクトル

// packet structure for InvenSense teapot demo ///InvenSenseティーポットデモのパケット構造
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE 中断検出ルーチン                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high ///MPU割り込み端子がハイ・レベルになったかどうかを示します
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP 初期設定                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically) ///I2Cバスに参加する（I2Cdevライブラリはこれを自動的には行いません）
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties ///400kHzのI2Cクロック。 コンパイルに問題がある場合は、この行にコメントする
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    /*
      シリアル通信を初期化します
      （Teapot Demoの出力に必要な115200が選択されていますが、
      実際はあなたのプロジェクトによって異なります）
    */
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    /*
       注：Teensy @ 3.3VやArduino Pro Mini（3.3V）のような8MHz以上のホストプロセッサは、
      ボーレートがプロセッサティックとあまりにもずれるため、このボーレートを確実に処理できません。
      このような場合には、38400以下を使用するか、UARTタイマー用に何らかの外部分離クリスタルソリューションを
      使用する必要があります。
    */
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device ///デバイスを初期化する
    ///Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection ///接続を確認する
    /// Serial.println(F("Testing device connections..."));
    ///Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready ///準備が整うのを待つ
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
   /// while (Serial.available() && Serial.read()); // empty buffer ///空のバッファ
   /// while (!Serial.available());                 // wait for data ///データを待つ
   /// while (Serial.available() && Serial.read()); // empty buffer again ///再び空のバッファ

    // load and configure the DMP ///DMPをロードして設定する
    ///Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity ///ここで独自のジャイロオフセットを供給し、最小感度
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip ///私のテストチップの工場出荷時のデフォルト値は1688

    // make sure it worked (returns 0 if so) //それが機能していることを確認（そうであれば0を返します）
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready //準備ができたらDMPをオンにします
        ///Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection //Arduino割り込み検出を有効にする
        ///Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        ///DMP Readyフラグを設定して、main loop（）関数がそれを使用しても問題ないことを知っているようにします
       /// Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison ///後で比較するために期待されるDMPパケットサイズを得る
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
       /// Serial.print(F("DMP Initialization failed (code "));
       /// Serial.print(devStatus);
       /// Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP メインプログラムループ                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything ///プログラミングが失敗した場合は何もしない
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available //MPU割り込みまたは余分なパケットの使用を待つ
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here ///他のプログラムの動作のものはここに
        // .
        // .
        // .
        /*
           もしあなたが本当に妄想的であれば、他のものの間で頻繁にテストして、
           mpuInterruptが真であるかどうかを調べることができます。 
           while（）ループからMPUデータを直ちに処理する
        */
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte ///割り込みフラグをリセットし、INT_STATUSバイトを取得する
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count ///現在のFIFOカウントを取得する
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)  
    ///オーバーフローをチェック（コードがあまりにも非効率的でない限り、これは起こりません）
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly  ///リセットして、きれいに続けることができます
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));///

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    ///それ以外の場合は、DMPデータレディ割り込みをチェックします（これは頻繁に発生するはずです）
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        ///利用可能なデータ長が正しいのを待って、非常に短い待ち時間
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO ///FIFOからパケットを読み込む
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        ///使用可能なパケットが1つ以上ある場合には、ここでトラックFIFOカウント
        // (this lets us immediately read more without waiting for an interrupt)
        ///（これにより、割り込みを待つことなくすぐにもっと読むことができます）
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            ///簡単な行列形式で四元数を表示する：w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
           /// Serial.print("quat\t");
           ///  Serial.print(q.w);
           /// Serial.print("\t");
           ///  Serial.print(q.x);
           ///  Serial.print("\t");
           ///  Serial.print(q.y);
           ///  Serial.print("\t");
           ///  Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees ///オイラー角を度で表示する
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            /// Serial.print("euler\t");
           ///  Serial.print(euler[0] * 180/M_PI);
           ///  Serial.print("\t");
           ///  Serial.print(euler[1] * 180/M_PI);
           ///  Serial.print("\t");
           ///  Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees ///オイラー角を度で表示する
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
           /// Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            ///Serial.print("\t");
            Serial.print(",");
            Serial.print(ypr[1] * 180/M_PI);
            ///Serial.print("\t");
            Serial.print(",");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.println(",");
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
           /// Serial.print("areal\t");
           ///  Serial.print(aaReal.x);
           ///  Serial.print("\t");
           ///  Serial.print(aaReal.y);
           ///  Serial.print("\t");
           ///  Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            /*
               初期世界フレーム加速度を表示し、重力を除去するように調整され、
               四元数からの既知の向きに基づいて回転する
            */
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
           /// Serial.print("aworld\t");
           ///  Serial.print(aaWorld.x);
           ///  Serial.print("\t");
           ///  Serial.print(aaWorld.y);
           ///  Serial.print("\t");
           ///  Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            ///InvenSenseティーポットのデモ形式で四元数の値を表示：
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            /// Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose ///packetCount、目的に応じて0xFFでループする
        #endif

        // blink LED to indicate activity ///アクティビティを示す点滅LED
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
