/* -----------------------------------------------------------------------------
 *  Prog_wheelAngle_encoder2_pwm2_func.ino:
 * ESP32_DevKitC_ver4+TB6612FNG でエンコーダ付きモータを回すプログラム
 * LEDCで両車輪をpwmで回転し、両車輪がある角度になると車輪を止めるプログラム
 * アクチュエータは、pololuのエンコーダ付きDS-16413-DG01D-Eで確認済み。
 * 2020.03.23  by jeong
--------------------------------------------------------------------------------*/

#include <RotaryEncoder.h>  //エンコーダを使うためのライブラリのインクルード
  
//-------------グルーバル定義(プログラム内で値が変わらないものを定義しておく）
//回転方向
#define CCW 1    //反時計方向
#define CW  -1   //時計方向

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
void motion();                      //車輪動作プログラム
void IRAM_ATTR ISR();

//-------変数定義
RotaryEncoder encoderL(EncL_A_PIN, EncL_B_PIN);   //IO19とIO23を左車輪カウンター入力用として設定
RotaryEncoder encoderR(EncR_B_PIN, EncR_A_PIN);   //IO5とIO18を右車輪　　”　（左車輪と逆方向）
int dutyRatioL=0;                                 //左車輪用dutyRatio用変数     
int dutyRatioR=0;                                 //右車輪用dutyRatio用変数   
float newAngL = 0;                                //左車輪用回転角度用変数 
float newAngR = 0;                                //右車輪用回転角度用変数


///////////SETUP///////////////////////////////
void setup(){
  
  //Serial_1との通信速度(PCとの通信) 
  Serial.begin(9600);          //9600 bpsに設定 
 
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
  
}


//------- プログラムループ(繰り返し実行)
void loop() {

 //--【ロボットの動作】 
  motion();
 
  //--【counter値と車輪角度を表示」 
  Serial.print("dutyL:");  Serial.print(dutyRatioL);
  Serial.print("  angL:");  Serial.print(newAngL);
  Serial.print("  dutyR:");  Serial.print(dutyRatioR);
  Serial.print("  angR:");  Serial.print(newAngR); 
  Serial.println();
  
  delay(100);
  
 }


/*--------------------------------------
 * void motion()
 * ロボットの動作を書く関数
-------------------------------------- */
void motion(){

 //--【counter値の読み込み】  
  int newCntL = encoderL.getPosition();  //左車輪のカウンター値を読み込む
  int newCntR = encoderR.getPosition();  //左車輪のカウンター値を読み込む
  
  //--【counter値を角度に車輪の角度に換算】
  newAngL = newCntL*Cnt2Ang;       //左車輪の角度を計算
  newAngR = newCntR*Cnt2Ang;       //左車輪の角度を計算
 
 
 //--【左車輪が360度を越えるとdutyRatioLを0%にして停止】
  if(newAngL > 360) dutyRatioL = 0;
  else dutyRatioL = 60;
  
  if(newAngR > 360) dutyRatioR = 0;
  else dutyRatioR= 60;
  
  motorOutputL(dutyRatioL); 
  motorOutputR(dutyRatioR); 
     
}

/*--------------------------------------------
 * void motorOutputL(int mDuty)
 * 引数：mDuty (duty比)
 * 返り値：なし
 * 回転方向、duty制限後のduty値を計算し出力する関数* 
 ----------------------------------------------*/
void motorOutputL(int mDuty){

   int dirState=0;          //モータ回転方向状態変数
   int dirIn1 = LOW;        //左車輪回転方向変数In1
   int dirIn2 = LOW;        //左車輪回転方向変数In2
    
  //--【引数のduty比を、正のduty比と回転方向を分離】
   if(mDuty < 0) { mDuty = -1*(mDuty); dirState = CW;}
   else {mDuty = mDuty; dirState = CCW;}

   //--【dutyを最大値に制限する】
   if(mDuty > dutyMax) mDuty = dutyMax;  

  //--【duty比をduty値に変換】 
   uint32_t duty = (mDuty*8192./100.);    //duty 100の場合8192

   //--【回転方向をdirInへ反映】
   if(dirState == CW)  {dirIn1=LOW;  dirIn2=HIGH;}; //時計方法  (-)
   if(dirState == CCW) {dirIn1=HIGH; dirIn2=LOW;};  //反時計方向(+)    
    
   //回転方向用pinへ信号を出力 
    digitalWrite(AIN1_PIN,dirIn1);
    digitalWrite(AIN2_PIN,dirIn2);

   //LEDC_CHANNELへpwmを出力
   ledcWrite(LEDC_CHANNEL_0,duty); 

}


/*--------------------------------------------
 * void motorOutputR(int mDuty)
 * 引数：mDuty (duty比)
 * 返り値：なし
 * 右車輪の回転方向、duty制限後のduty値を計算し出力する関数* 
 ----------------------------------------------*/
 void motorOutputR(int mDuty){

   int dirState=0;          //モータ回転方向状態変数
   int dirIn1 = LOW;        //右車輪回転方向変数In1
   int dirIn2 = LOW;        //右車輪回転方向変数In2
    
  //--【引数のduty比を、正のduty比と回転方向を分離】
   if(mDuty < 0) { mDuty = -1*(mDuty); dirState = CW;}
   else {mDuty = mDuty; dirState = CCW;}   

   //--【dutyを最大値に制限する】
   if(mDuty > dutyMax) mDuty = dutyMax;  

  //--【duty比をduty値に変換】 
   uint32_t duty = (mDuty*8192./100.);    //duty 100の場合8192

   //--【回転方向をdirInへ反映】
   if(dirState == CW)  {dirIn1=LOW;  dirIn2=HIGH;}; //時計方法  (-)
   if(dirState == CCW) {dirIn1=HIGH; dirIn2=LOW;};  //反時計方向(+)    
    
   //回転方向用pinへ信号を出力 
    digitalWrite(BIN1_PIN,dirIn1);
    digitalWrite(BIN2_PIN,dirIn2);

   //LEDC_CHANNELへpwmを出力
   ledcWrite(LEDC_CHANNEL_1,duty); 
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






 
