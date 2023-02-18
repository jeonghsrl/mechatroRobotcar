
/* -----------------------------------------------------------------------------
 *  Prog_whttlRotate_pwm2_func.ino:
 * ESP32_DevKitC_ver4+TB6612FNG モータドライバでモータを回すプログラム
 * LEDCでpwmを出力し、左右車輪を変数と関数を用いて回転させるプログラム
 * アクチュエータは、pololuのエンコーダ付きDS-16413-DG01D-Eで確認済み。
 * 2020.03.23  by jeong
--------------------------------------------------------------------------------*/

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

//----------関数定義
void motorOutputL(int mDuty);  //左モータドライバにduty値を与える関数
void motorOutputR(int mDuty);  //右モータドライバにduty値を与える関数
void motion();  //車輪動作関数

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
  ledcAttachPin(PWMA_PIN, LEDC_CHANNEL_0);                     //LEDC_CHANNEL_0をPWMA_PINへ出力    
  ledcAttachPin(PWMB_PIN, LEDC_CHANNEL_1);                     //LEDC_CHANNEL_1をPWMB_PINへ出力

  delay(1000);
   
  //---【動作プログラム】
  motion();    //車輪の動作を作るプログラム
}

/*--------------------------------------
 * void motion()
 * ロボットの動作を書く関数
-------------------------------------- */
void motion(){
  
 //--【duty比の入力】
  int dutyRatioL = 60;   //左車輪をduty比60%でCCW方向回転   
  int dutyRatioR = 60;   //右車輪をduty比60%でCCW方向回転   
   
 //--【duty比をモータドライバへ出力する関数の呼び出し】
   motorOutputL(dutyRatioL);  //左車輪へduty比を出力
   motorOutputR(dutyRatioR);  //右車輪へduty比を出力

 //--【表示】
  Serial.print("  L_Wheel: ");  Serial.print(dutyRatioL);  
  Serial.print("  R_Wheel: ");  Serial.print(dutyRatioR);  
  Serial.println();
  delay(3000);    //3s間 delay

 //--【duty比の入力】
  dutyRatioL = -60;     //左車輪をduty比60%でCW方向回転 
  dutyRatioR = -60;     //右車輪をduty比60%でCW方向回転 
  motorOutputL(dutyRatioL);  //左車輪へduty比を出力
  motorOutputR(dutyRatioR);  //右車輪へduty比を出力
  Serial.print("  L_Wheel: ");  Serial.print(dutyRatioL);  
  Serial.print("  R_Wheel: ");  Serial.print(dutyRatioR);  
  Serial.println();
  delay(3000);    //3s delay

 //--【duty比の入力】
  dutyRatioL = 0;     //左車輪をduty比0%で停止 
  dutyRatioR = 0;     //右車輪をduty比0%で停止 
  motorOutputL(dutyRatioL);  //左車輪へduty比を出力
  motorOutputR(dutyRatioR);  //右車輪へduty比を出力
  Serial.print("  L_Wheel: ");  Serial.print(dutyRatioL);  
  Serial.print("  R_Wheel: ");  Serial.print(dutyRatioR);  
  Serial.println();
   
}

/*--------------------------------------------
 * void motorOutputL(int mDuty)
 * 引数：mDuty (duty比)
 * 返り値：なし
 * 左車輪の回転方向、duty制限後のduty値を計算し出力する関数* 
 ----------------------------------------------*/
void motorOutputL(int mDuty){

   uint32_t duty = 0;
    
  //--【引数のduty比を、正と負にわけて処理】
   if(mDuty < 0) { 
      digitalWrite(AIN1_PIN,LOW);            //時計方向(-)回転 
      digitalWrite(AIN2_PIN,HIGH);
      mDuty = -1*mDuty;                      //duty比を正の数に変換
   }
   else {
      digitalWrite(AIN1_PIN,HIGH);           //反時計方向(+)回転 
      digitalWrite(AIN2_PIN,LOW);
      mDuty =  1*mDuty;
   }

   if(mDuty > 100) mDuty = 100;           //duty比を100%に制限
   duty = (mDuty/100.)*8192;             //正の整数に変換(duty 100の場合8192)　　
   ledcWrite(LEDC_CHANNEL_0,duty);        //LEDC_CHANNELへpwmを出力

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


//------- プログラムループ(繰り返し実行)
void loop() {  }
 
