/* -----------------------------------------------------------------------------------
 *   *  Prog_wheelAngle_encoder1_func.ino:
 *  ESP32_DevKitC_ver4で２相エンコーダをカウントする基本プログラム
 *  pololuのエンコーダ付きDS-16413-DG01D-Eで確認済み。
 *  DIOに外部interrupt機能を使って、2個のエンコーダを位相係数モードでカウント
 *  RotaryEncoderのライブラリが必要(library管理からインストール）
 *  2020.03.23 by jeong
--------------------------------------------------------------------------------------*/

#include <RotaryEncoder.h>  //エンコーダを使うためのライブラリのインクルード

#define EncL_A_PIN 19   // 左車輪用Encoder A相入力ピン
#define EncL_B_PIN 23   // 左車輪用Encoder B相入力ピン　
#define EncR_A_PIN 5    // 右車輪用Encoder A相入力ピン
#define EncR_B_PIN 18   // 右車輪用Encoder B相入力ピン 
#define Cnt2Ang    360./132.  //カウンター値を角度に変換 角度= CNT*(360度/1回転時のカウンター値） 
  
//--【Encoder classの宣言】 
RotaryEncoder encoderL(EncL_A_PIN, EncL_B_PIN);   //IO19とIO23を左車輪カウンター入力用として設定
RotaryEncoder encoderR(EncR_B_PIN, EncR_A_PIN);   //IO5とIO18を右車輪　　”　（左車輪と逆方向）

//--【 Encoder外部割込みサービスの呼び出し】 
void IRAM_ATTR ISR() {           
   encoderL.tick();               //左車輪用エンコーダの状態を確認
 　encoderR.tick();               //右車輪用エンコーダの状態を確認
}

//--Setup関数：ピンの機能を使うためのいろんな初期設定を行う
void setup() {

 //確認用serial monitor通信設定 
  Serial.begin(9600);     

 //--【Encoder用pinを外部interruptに設定】
  attachInterrupt(EncL_A_PIN,ISR,CHANGE);   //IO19番ピンを外部割込み用として設定
  attachInterrupt(EncL_B_PIN,ISR,CHANGE);   //IO23番ピンを外部割込み用として設定
  attachInterrupt(EncR_A_PIN,ISR,CHANGE);   //IO5番ピンを外部割込み用として設定
  attachInterrupt(EncR_B_PIN,ISR,CHANGE);   //IO18番ピンを外部割込み用として設定  
}


//--- プログラムループ(繰り返し実行)
void loop() {

  //--【counter値の読み込み】  
  int newCntL = encoderL.getPosition();  //左車輪のカウンター値を読み込む
  int newCntR = encoderR.getPosition();  //左車輪のカウンター値を読み込む
  
  //--【counter値を角度に車輪の角度に換算】
  float newAngL = newCntL*Cnt2Ang;       //左車輪の角度を計算
  float newAngR = newCntR*Cnt2Ang;       //左車輪の角度を計算
  
  //--【counter値と車輪角度を表示」 
  Serial.print("cntL:");  Serial.print(newCntL);
  Serial.print("  angL:");  Serial.print(newAngL);
  Serial.print("　cntR:");  Serial.print(newCntR);
  Serial.print("  angR:");  Serial.print(newAngR);
  Serial.println();

  delay(100);
 }
