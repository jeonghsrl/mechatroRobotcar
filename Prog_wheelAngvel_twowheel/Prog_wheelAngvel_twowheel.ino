/* -----------------------------------------------------------------------------
 *  Prog_whttlRotate_pwm1_func.ino:
 * ESP32_DevKitC_ver4+TB6612FNG モータドライバでモータを回すプログラム
 * LEDCでpwmを出力し、両車輪の角速度を計測するプログラム 10s間回転後、停止。
 * アクチュエータは、pololuのエンコーダ付きDS-16413-DG01D-Eで確認済み。
 * 2020.03.23  by jeong
--------------------------------------------------------------------------------*/


#include <RotaryEncoder.h>  //エンコーダを使うためのライブラリのインクルード
#include <Ticker.h>         //タイマー割込みを使うためのライブラリのインクルード
  
//-------------グルーバル定義(プログラム内で値が変わらないものを定義しておく）

//pwm発生用のタイマーチャンネル用定義
#define LEDC_CHANNEL_0 0       //左車輪用pwmA チャンネル0
#define LEDC_CHANNEL_1 1       //右車輪用pwmB チャンネル1
#define LEDC_TIMER_BIT 13      //LEDC PWMタイマー 13bit
#define LEDC_SERVO_FREQ 2500   //pwm 周波数2.5khz

//モータドライバ制御用PIN番号の定義
#define PWMA_PIN 25    //左車輪用PWM出力用IO pin番号    
#define AIN2_PIN 26    //左車輪用方向制御用IO pin番号
#define AIN1_PIN 27    //左車輪用方向制御用IO pin番号
#define BIN2_PIN 12    //右車輪用方向制御用IO pin番号
#define BIN1_PIN 14    //右車輪用方向制御用IO pin番号
#define PWMB_PIN 13    //右車輪用PWM出力用IO pin番号
#define dutyMax 100     //車輪用dutyの最大値

//エンコーダ読み込み用PIN番号の定義
#define EncL_A_PIN 19   // 左車輪用Encoder A相入力ピン
#define EncL_B_PIN 23   // 左車輪用Encoder B相入力ピン
#define EncR_A_PIN 5    // 右車輪用Encoder A相入力ピン
#define EncR_B_PIN 18   // 右車輪用Encoder B相入力ピン　　 
#define Cnt2Ang    360./132.  //カウンター値を角度に変換 角度= CNT*(360度/1回転時のカウンター値） 

//----------関数定義
void motorOutputL(int mDuty);  //左モータドライバにduty比を与える関数
void motorOutputR(int mDuty);  //右モータドライバにduty値を与える関数
void IRAM_ATTR ISR();
void timerloop();                   //タイマー割込み timerloop関数

//-------変数定義
Ticker timerLoop; 
RotaryEncoder encoderL(EncL_A_PIN, EncL_B_PIN);   //IO19とIO23を左車輪カウンター入力用として設定
RotaryEncoder encoderR(EncR_B_PIN, EncR_A_PIN);   //IO5とIO18を右車輪　　”　（左車輪と逆方向）
int dutyRatioL=0;                                 //左車輪用dutyRatio用変数     
int dutyRatioR=0;                                 //右車輪用dutyRatio用変数   
float newAngL = 0;                                //左車輪用回転角度用変数 
float newAngR = 0;                                //右車輪用回転角度用変数
double newAngVelL=0;                              //左車輪用回転角速度変数 
double newAngVelR=0;                              //右車輪用回転角速度変数 
double SAMP_TIMEms =100;                          //タイマー割込み時間

///////////SETUP///////////////////////////////
void setup(){
  
  //Serial_1との通信速度(PCとの通信) 
  Serial.begin(115200);          //9600 bpsに設定 
 
  //--【PIN mode（機能）の設定】　
  pinMode(PWMA_PIN,OUTPUT);    //PWMA_PINをpwm用のOUTPUTに設定
  pinMode(AIN1_PIN,OUTPUT);    //AIN1_PINを回転方向用のOUTPUTに設定
  pinMode(AIN2_PIN,OUTPUT);    //AIN1_PINを回転方向用のOUTPUTに設定 
  pinMode(PWMB_PIN,OUTPUT);    //PWMB_PINをpwm用のOUTPUTに設定
  pinMode(BIN1_PIN,OUTPUT);    //BIN1_PINを回転方向用のOUTPUTに設定
  pinMode(BIN2_PIN,OUTPUT);    //BIN2_PINを回転方向用のOUTPUTに設定
 

  //--【LEDCの設定】
  ledcSetup(LEDC_CHANNEL_0, LEDC_SERVO_FREQ, LEDC_TIMER_BIT);  //左車輪用LEDC_CHANNEL_0の設定
  ledcSetup(LEDC_CHANNEL_1, LEDC_SERVO_FREQ, LEDC_TIMER_BIT);  //右車輪用LEDC_CHANNEL_1の設定

  //--【LEDCとpwm用pinの接続】
  ledcAttachPin(PWMA_PIN, LEDC_CHANNEL_0);      //LEDC_CHANNEL_0をPWMA_PINへ出力 
  ledcAttachPin(PWMB_PIN, LEDC_CHANNEL_1);      //LEDC_CHANNEL_1をPWMB_PINへ出力
  delay(1000);

   //--【Encoder用pinを外部interruptに設定】
  attachInterrupt(EncL_A_PIN,ISR,CHANGE);   //IO19番ピンを外部割込み用として設定
  attachInterrupt(EncL_B_PIN,ISR,CHANGE);   //IO23番ピンを外部割込み用として設定
  attachInterrupt(EncR_A_PIN,ISR,CHANGE);   //IO5番ピンを外部割込み用として設定
  attachInterrupt(EncR_B_PIN,ISR,CHANGE);   //IO18番ピンを外部割込み用として設定  

  timerLoop.attach_ms(SAMP_TIMEms,timerloop);  //timerloop関数をSAMP_TIMEmsで設定
}


//------- プログラムループ(繰り返し実行)
void loop() {
  }


void timerloop(){
  
 double time_ms = millis();               //プログラムの実行時間を計測

 newAngL = getAngleL();                   //角度計算関数の実行
 newAngVelL = getAngVelL();               //角速度計算関数の実行
 newAngR = getAngleR();                   //角度計算関数の実行
 newAngVelR = getAngVelR();               //角速度計算関数の実行
 

 //--【左車輪をdutyRatio=60%で回転】
 if( time_ms > 10000 ) dutyRatioL = 0;    //10sになったら、左車輪を止める
 else dutyRatioL = 80;
 
 //--【右車輪をdutyRatio=60%で回転】
 if( time_ms > 10000 ) dutyRatioR = 0;    //10sになったら、右車輪を止める
 else dutyRatioR = 80;

  motorOutputL(dutyRatioL);  
  motorOutputR(dutyRatioR);   
 
  //--【counter値と車輪角度を表示」 
  Serial.print("  time:");  Serial.print(time_ms);
  Serial.print("  angL:");  Serial.print(newAngL);
  Serial.print("  angVelL:");  Serial.print(newAngVelL);
  Serial.print("  angR:");  Serial.print(newAngR); 
  Serial.print("  angVelR:");  Serial.print(newAngVelR);
  Serial.println();
  
 }


/*------------------------------------------
 * float getAngleL()
 * 引数：なし
 * 返り値：angle
 * 左車輪の角度(degee)を計算して返す関数
-------------------------------------------- */
float getAngleL(){
  
  //--【counter値の読み込み】  
  int newCnt = encoderL.getPosition();  //左車輪のカウンター値を読み込む
  
  //--【counter値を角度に車輪の角度に換算】
  float angle = newCnt*Cnt2Ang;       //左車輪の角度を計算
    
  return angle;
}


/*------------------------------------------
 * float getAngleR()
 * 引数：なし
 * 返り値：angle
 * 右車輪の角度(degee)を計算して返す関数
-------------------------------------------- */
float getAngleR(){
  
  //--【counter値の読み込み】  
  int newCnt = encoderR.getPosition();  //左車輪のカウンター値を読み込む
  
  //--【counter値を角度に車輪の角度に換算】
  float angle = newCnt*Cnt2Ang;       //左車輪の角度を計算
    
  return angle;
}


/*------------------------------------------
 * float getAngVelL()
 * 引数：なし
 * 返り値：angleVel
 * 左車輪の角度(degee)を計算して返す関数
-------------------------------------------- */
double getAngVelL(){

  static int oldCnt=0;                              //１サイクルタイム前のカウント格納用変数
  int newCnt=encoderL.getPosition();                //現在のカウント読み込み
  double diffAngle = (newCnt-oldCnt)*Cnt2Ang;       //角度差分値計算
  double angleVel = diffAngle/(SAMP_TIMEms*0.001);  //角速度の計算 

  oldCnt = newCnt;                                   //現在のカウントを次回の計算のために保存
  
  return angleVel;
}


/*------------------------------------------
 * float getAngVelR()
 * 引数：なし
 * 返り値：angleVel
 * 右車輪の角度(degee)を計算して返す関数
-------------------------------------------- */
double getAngVelR(){

  static int oldCnt=0;                              //１サイクルタイム前のカウント格納用変数
  int newCnt=encoderR.getPosition();                //現在のカウント読み込み
  double diffAngle = (newCnt-oldCnt)*Cnt2Ang;       //角度差分値計算
  double angleVel = diffAngle/(SAMP_TIMEms*0.001);  //角速度の計算 

  oldCnt = newCnt;                                   //現在のカウントを次回の計算のために保存
  
  return angleVel;
}



/*--------------------------------------------
 * void motorOutputL(int mDuty)
 * 引数：mDuty (duty比)
 * 返り値：なし
 * 回転方向、duty制限後のduty値を計算し出力する関数* 
 ----------------------------------------------*/
void motorOutputL(int mDuty){

   uint32_t duty = 0;
    
  //--【引数のduty比を、正と負にわけて処理】
   if(mDuty < 0) { 
      digitalWrite(AIN1_PIN,LOW);            //時計方向(-)回転 
      digitalWrite(AIN2_PIN,HIGH);
      mDuty = -1*mDuty;                      //duty比を正の数に変換
      if(mDuty > 100) mDuty = 100;           //duty比を100%に制限
      duty = (mDuty/100.)*8192;             //正の整数に変換(duty 100の場合8192)　　
      ledcWrite(LEDC_CHANNEL_0,duty);        //LEDC_CHANNELへpwmを出力
   }
   else {
      digitalWrite(AIN1_PIN,HIGH);           //反時計方向(+)回転 
      digitalWrite(AIN2_PIN,LOW);
      mDuty =  1*mDuty;
      if(mDuty > 100) mDuty = 100;           
      duty = (mDuty/100.)*8192;
      ledcWrite(LEDC_CHANNEL_0,duty);        
   }
}



/*--------------------------------------------
 * void motorOutputR(int mDuty)
 * 引数：mDuty (duty比)
 * 返り値：なし
 * 右車輪の回転方向、duty制限後のduty値を計算し出力する関数* 
 ----------------------------------------------*/
 void motorOutputR(int mDuty){
   
   uint32_t duty = 0;
    
  //--【引数のduty比を、正と負にわけて処理】
   if(mDuty < 0) { 
      digitalWrite(BIN1_PIN,LOW);            //時計方向(-)回転 
      digitalWrite(BIN2_PIN,HIGH);
      mDuty = -1*mDuty;                      //duty比を正の数に変換
   }
   else {
      digitalWrite(BIN1_PIN,HIGH);           //反時計方向(+)回転 
      digitalWrite(BIN2_PIN,LOW);
      mDuty =  1*mDuty;
   }

   if(mDuty > 100) mDuty = 100;           //duty比を100%に制限
   duty = (mDuty/100.)*8192;             //正の整数に変換(duty 100の場合8192)　　
   ledcWrite(LEDC_CHANNEL_1,duty);        //LEDC_CHANNELへpwmを出力

}


/*--------------------------------------------
 * void IRAM_ATTR ISR()
 * 引数：なし　 返り値：なし
 * Encoder外部割込みサービスの呼び出し 
 ----------------------------------------------*/
void IRAM_ATTR ISR() {            
  encoderL.tick();               //左車輪用エンコーダの状態を確認
  encoderR.tick();               //右車輪用エンコーダの状態を確認
}






 
