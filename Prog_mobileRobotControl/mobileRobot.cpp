
#include <RotaryEncoder.h>  //エンコーダを使うためのライブラリのインクルード
#include <Ticker.h> 
#include "mobileRobot.h"
   
RotaryEncoder encoderL(EncL_A_PIN, EncL_B_PIN);   //IO19とIO23を左車輪カウンター入力用として設定
RotaryEncoder encoderR(EncR_B_PIN, EncR_A_PIN);   //IO5とIO18を右車輪　　”　（左車輪と逆方向）

//-------変数定義

void robotSetup(){
  
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

   //--【Encoder用pinを外部interruptに設定】
  attachInterrupt(EncL_A_PIN,ISR,CHANGE);   //IO19番ピンを外部割込み用として設定
  attachInterrupt(EncL_B_PIN,ISR,CHANGE);   //IO23番ピンを外部割込み用として設定
  attachInterrupt(EncR_A_PIN,ISR,CHANGE);   //IO5番ピンを外部割込み用として設定
  attachInterrupt(EncR_B_PIN,ISR,CHANGE);   //IO18番ピンを外部割込み用として設定  

 
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



/*--------------------------------------------
 * void getLightSen()
 * 引数：なし　 返り値：照度
 * 光センサの照度を返す関数 
 ----------------------------------------------*/
float getLightSen(){

  int dat = analogRead(LightSen_PIN);
  float v = dat*3.6/4096;
  float current = v/0.2;
  float ev = 0.46*current - 0.52;

  return ev;
}

/*--------------------------------------------
 * void getLightSen()
 * 引数：なし　 返り値：音電圧
 * 光センサの照度を返す関数 
 ----------------------------------------------*/
float getSoundSen(){

  int dat = analogRead(SoundSen_PIN);
  return dat;
}

/*--------------------------------------------
 * void getDistanceSen()
 * 引数：なし　 返り値：音電圧
 * 光センサの照度を返す関数 
 ----------------------------------------------*/
float getDistanceSen(){

  int dat = analogRead(DistSen_PIN);
  float v = dat*3.6/4096;
  float L = 0.182/v;
  
  return L;
}
