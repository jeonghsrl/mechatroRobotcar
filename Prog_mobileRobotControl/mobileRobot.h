/*

*/

#ifndef MobileRobot_h
#define Mobilerobot_h   

//-------------グルーバル定義(プログラム内で値が変わらないものを定義しておく）

#define SAMP_TIMEms 100                          //タイマー割込み時間

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

//センサ用PIN番号の定義
#define LightSen_PIN A6     // 光センサアナログ入力ピン
#define SoundSen_PIN A3     // 音センサアナログ入力ピン
#define DistSen_PIN A0      // 距離センサアナログ入力ピン
 
//----------関数定義
void robotSetup();
void motorOutputL(int mDuty);  //左モータドライバにduty比を与える関数
void motorOutputR(int mDuty);  //右モータドライバにduty値を与える関数
void IRAM_ATTR ISR();
float getAngleL();
float getAngleR();
double getAngVelL();
double getAngVelR();

//センサ用関数
float getLightSen();
float getSoundSen();
float getDistanceSen();


#endif
