/* -----------------------------------------------------------------------------
 *  Prog_whttlRotate_pwm1_func.ino:
 * ESP32_DevKitC_ver4+TB6612FNG モータドライバでモータを回すプログラム
 * LEDCでpwmを出力し、両車輪の角速度を計測するプログラム 10s間回転後、停止。
 * アクチュエータは、pololuのエンコーダ付きDS-16413-DG01D-Eで確認済み。
 * 2020.03.23  by jeong
--------------------------------------------------------------------------------*/

//#include <RotaryEncoder.h>  //エンコーダを使うためのライブラリのインクルード
#include <Ticker.h>
#include "mobileRobot.h"    //ロボット制御用ヘッダファイル 

Ticker timerLoop;
void timerloop();                   //タイマー割込み timerloop関数

//----ユーザグルーバル変数--------
int dutyRatioL;                                 //左車輪用dutyRatio用変数     
int dutyRatioR;                                 //右車輪用dutyRatio用変数   
float newAngL;                                  //左車輪用回転角度用変数 
float newAngR;                                  //右車輪用回転角度用変数
double newAngVelL;                              //左車輪用回転角速度変数 
double newAngVelR;                              //右車輪用回転角速度変数 
//------------------------------


///////////SETUP///////////////////////////////
void setup(){
  
  //Serial_1との通信速度(PCとの通信) 
  Serial.begin(115200);          //9600 bpsに設定 

  //ロボット制御用セットアップ関数
  robotSetup(); 
  timerLoop.attach_ms(SAMP_TIMEms,timerloop);  //timerloop関数をSAMP_TIMEmsで設定
}


//------- プログラムループ(繰り返し実行)
void loop() {
  
  if( newAngL > 1000 ) dutyRatioL = 0;    //10sになったら、左車輪を止める
  else dutyRatioL = 80;

  }


void timerloop(){

  //一定時間間隔で実行されるべきユーザプログラム
  
 double time_ms = millis();               //プログラムの実行時間を計測

 newAngL = getAngleL();                   //角度計算関数の実行
 newAngVelL = getAngVelL();               //角速度計算関数の実行
 newAngR = getAngleR();                   //角度計算関数の実行
 newAngVelR = getAngVelR();               //角速度計算関数の実行
 
 //--【左車輪をdutyRatio=60%で回転】
// if( time_ms > 10000 ) dutyRatioL = 0;    //10sになったら、左車輪を止める
// else dutyRatioL = 80;
 
 //--【右車輪をdutyRatio=60%で回転】
// if( time_ms > 10000 ) dutyRatioR = 0;    //10sになったら、右車輪を止める
// else dutyRatioR = 80;

  motorOutputL(dutyRatioL);  
  motorOutputR(dutyRatioR);   
 
  //--【counter値と車輪角度を表示」 
  if(0){
  Serial.print("  time:");  Serial.print(time_ms);
  Serial.print("  angL:");  Serial.print(newAngL);
  Serial.print("  angVelL:");  Serial.print(newAngVelL);
  Serial.print("  angR:");  Serial.print(newAngR); 
  Serial.print("  angVelR:");  Serial.print(newAngVelR);
  }
  if(1){
  Serial.print("  Light:");  Serial.print(getLightSen());
  Serial.print("  Sound:");  Serial.print(getSoundSen());
  Serial.print("  Distance:");  Serial.print(getDistanceSen());
  Serial.println();
  }
 }







 
