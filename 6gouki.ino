#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#define Pi 3.1416

/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/
//Define L298N Dual H-Bridge Motor Controller Pins

#define dir1PinL  7    //左のモータ（順回転でHIGH、逆回転でLOW）
#define dir2PinL  8    //左のモータ（順回転でLOW、逆回転でHIGH）
#define speedPinL 6    //モーターの回転速度を調整するためのピン。Pulse Width Modulation

#define dir1PinR  2    //右のモータ（順回転でHIGH、逆回転でLOW）
#define dir2PinR  4    //右のモータ（順回転でLOW、逆回転でHIGH）
#define speedPinR 5    //モーターの回転速度を調整するためのピン。Pulse Width Modulation

//測距センサ、障害物センサ
#define forwardPin  0   //前の測距センサ
#define backPin  1   //後ろの障害物センサ

/*調整するパラメータ --------------------------------------------------------------------------- */
//重心のずれ
#define D_CENTER 10

//測距センサのパラメータ
#define BUF_SIZE 5 //8
#define SOKKYO_THRESHOLD_FORWARD 0.1
#define SOKKYO_THRESHOLD_BACK 0.1 
#define distance_count 10 //何回分の障害物センサーの値の平均値を信用ある値として使うか

//加速度センサ
#define ACmeter_count 40   //過去何回分の加速度センサーの値の平均値を信用ある値として使うか
//２０の時おおよそ誤差0.05度以内
//20の時getAngle一回にかかる時間が30m sec
#define ACmeterVPin  12  //ACmeter の動作用電源５V、ほかのピンでも可
Adafruit_MMA8451 accel = Adafruit_MMA8451(); // create instance of the MMA8452 class
  
//  Hardware hookup:
//  Arduino --------------- MMA8452Q Breakout
//    3.3V  ---------------     3.3V
//    GND   ---------------     GND
//  SDA (A4) --\/330 Ohm\/--    SDA
//  SCL (A5) --\/330 Ohm\/--    SCL

/* ローパスフィルタのパラメータ */
#define K 0.05

//set_angleのパラメータ：
#define SET_ANGLE_SPEED 255 //回転速度の設定
#define SET_ANGLE_THRESHOLD 1   //set_angleでの許容幅
#define E0 0
#define E1 3
#define E2 0.2          //移動時間の設定：keep((1+E0*sin(d_angle))*(abs(E1 * d_angle + E2 * d_angle_sum))

// PID制御のパラメータ  e0 = d_angle; u = Kp*e0 + Ki*(es) - Kd*(e0-e1);
//Kc = 0.5, Tc = 10s, Kp = 0.6*Kc, Ki = Kp/Ti = Kp/(Tc/2), Kd = Kp*(Tc/8)
#define Kp 2
#define Ki 0.2
#define Kd 1
#define ES_MAX  360000
// 
float calculate_mean(float *buf){
  float sum = 0.0;
  for (int i = 0; i < BUF_SIZE; i++){
    sum += buf[i];
  }
  return sum/BUF_SIZE;
}

/***************motor control***************/

/*---------------------------------------*/
float x_g_array[2] = {0.0, 0.0};
float y_g_array[2] = {0.0, 0.0};
//ローパスフィルタ
void LPF(float val, float *vec){
  vec[0] = (1-K)*vec[1] + K*val;
  vec[1] = vec[0];
}
/*---------------------------------------*/


void software_reset() {
  asm volatile ("  jmp 0");
}

/**************加速度センサのため*************/
void setupMMA8451() {
  pinMode( ACmeterVPin, OUTPUT);
  digitalWrite(ACmeterVPin, HIGH);
  
  // シリアルポートを9600 bps[ビット/秒]で初期化
  Wire.begin();
  Serial.begin(9600);

  Serial.println("Adafruit MMA8451 connecting...");

  while(1){
    if (! accel.begin()) {
      Serial.println("Couldnt start");
    }else{
      break;
    }
  }
  Serial.println("MMA8451 found!");

  accel.setRange(MMA8451_RANGE_2_G);

  Serial.print("Range = ");
  Serial.print(2 << accel.getRange());
  Serial.println("G");
}
/************加速度センサのため***************/

/***********加速度センサからの入力に応じた処理***********/

  /*ここに、gForceX, gForceY, gForceZの値に応じた処理を書く*/
  /*この加速度センサはx、y、z軸いずれも、鉛直上を向いているときに、1000を出力。*/
  /*姿勢は、車が鉛直壁に張り付いているとして、z軸方向が壁の法線方向と考えられるので、加速度のx軸、y軸成分(gForceX, gForeceY)を読み取り、それらの変化から姿勢を制御することを目標とする。*/

//現在の角度を測定

float gForceX, gForceY, gForceZ; //↑を重力加速度の何倍か、に変換した値

float current_angle; //現在の機体が向いている方向。前はmoving_angleという名前だった。-180~180度。
sensors_event_t event;

void get_angle(){  //読み取ったgの値から機体が向いている方向を計算して、current_angleに代入する
  gForceX = 0;
  gForceY = 0;
  
  if (accel.begin()) {
  for(int i=0;i<ACmeter_count;i++){ //振動による計測のブレが大きいので、過去ACmeter_count回の平均をとって現在の加速度の値とする
      
	  accel.getEvent(&event);  
    gForceX += event.acceleration.x;
    gForceY += event.acceleration.y;
  }
  }else stop_Stop();
  //水平右方向を０度とするロボットの向いている角度。範囲は-180度～180度
  current_angle= atan2(-gForceY, gForceX)*180/Pi;

//  Serial.print("    current_angle,gForceX,gForceY = ");
//  Serial.println(current_angle );
//  Serial.println(gForceX );
//  Serial.println(gForceY );
}
void printData() {//シリアルモニタに出力
  //Serial.print("  gForce");
  //Serial.print(" X=");
  //Serial.print(gForceX);
  //Serial.print('g');
  //Serial.print(" Y=");
  //Serial.print(gForceY);
  //Serial.print('g');
//  Serial.print(" Z=");
//  Serial.print(gForceZ);
//  Serial.println('g');
	Serial.print("    current_angle = ");
	Serial.print(current_angle);
	Serial.print("  \n");
}

//current_angleとgoal_angleの差を求める。そのままだと差の絶対値が180を超えてしまう（正しい向きに回転しなくなるor激しく回転してしまうので困る）ので、差を-180～180の間に収める
float calculate_d_angle(float goal_angle) {
	get_angle();
	float d_angle = current_angle - goal_angle;
	if (abs(d_angle) > 180) {
		if (d_angle > 0) { d_angle -= 360; }
		else { d_angle += 360; }
	}
	return d_angle;
}

//測距センサ
float voltage_buf[BUF_SIZE] = {};
int buf_counter;
int buf_flag;
int tmp_val;
float tmp_voltage;
float sum_of_tmp_voltage;
float mean_of_tmp_voltage;

void reset_dist() {
	tmp_val = 0;
	tmp_voltage = 0.0;
	sum_of_tmp_voltage = 0.0;
	mean_of_tmp_voltage = 0.0;
	buf_counter = 0;
	buf_flag = 0;
}

bool check_dist(int pin) {
	//voltage_bufにdistance_count回測った平均を記録していく
  sum_of_tmp_voltage = 0;
	for (int i = 0; i < distance_count; i++) {
		tmp_val = analogRead(pin);
		tmp_voltage = (tmp_val * 5) / 1024.0;
		sum_of_tmp_voltage += tmp_voltage;
	}
	mean_of_tmp_voltage = sum_of_tmp_voltage / (distance_count * 1.0);
	voltage_buf[buf_counter] = mean_of_tmp_voltage;
	buf_counter++;
	if (buf_counter == BUF_SIZE) {
		buf_counter = 0;
	}

//    Serial.println("mean_of_tmp_voltage");
//    Serial.println(mean_of_tmp_voltage);
	//BUF_SIZE回以上測定していて
	//かつbuf_size前の電圧と今の電圧の差が大きければ障害物があったとみなす
	if (buf_flag >= BUF_SIZE) {
    switch(pin)
    {
      case forwardPin:
          if ((voltage_buf[buf_counter + 1] - mean_of_tmp_voltage) > SOKKYO_THRESHOLD_FORWARD) {
			      return 1;
		      }
    case backPin:
          if ((voltage_buf[buf_counter + 1] - mean_of_tmp_voltage) > SOKKYO_THRESHOLD_BACK) {
            return 1;
          }
	  }
	}
	else {
		buf_flag++;
	}
	return 0;
}


//モーターの操作
void drive_motors(float L_speed, float R_speed)  //モーターに送るspeed（電圧）と方向を指示する
{
	if (L_speed >= 0) {
		if (L_speed > 255) {
			L_speed = 255;
		}
		digitalWrite(dir1PinL, HIGH);
		digitalWrite(dir2PinL, LOW);
		analogWrite(speedPinL, L_speed);
	}
	else {
		if (L_speed < (-255)) {
			L_speed = (-255);
		}
		digitalWrite(dir1PinL, LOW);
		digitalWrite(dir2PinL, HIGH);
		analogWrite(speedPinL, -L_speed);
	}

	if (R_speed >= 0) {
		if (R_speed > 255) {
			R_speed = 255;
		}
		digitalWrite(dir1PinR, HIGH);
		digitalWrite(dir2PinR, LOW);
		analogWrite(speedPinR, R_speed);
	}
	else {
		if (R_speed < (-255)) {
			R_speed = (-255);
		}
		digitalWrite(dir1PinR, LOW);
		digitalWrite(dir2PinR, HIGH);
		analogWrite(speedPinR, -R_speed);
	}
}

void go_Forward(int speed_var, int goal_angle) //フィードバック制御なし、ただまっすぐ進もうとする
{
	drive_motors(speed_var + D_CENTER * cos(goal_angle), speed_var - D_CENTER * cos(goal_angle));  //第二項はそれぞれ重心のずれの補正分（D_CENTERで調整）
	//Serial.print("  GO FORWARD\n");
}

void go_Back(int speed_var, int goal_angle)  //後ろに進む（注意！goal_angleは機体を向かせたい方向なので、後ろに進む場合進行方向とは真逆）
{
	drive_motors(-speed_var + D_CENTER * cos(goal_angle), -speed_var - D_CENTER * cos(goal_angle));
	//Serial.print("  GO BACK\n");
}

/* 左右に転回 speed:反時計回りを正 center:転回中心(L:-1 ~ O:0 ~ R:1) */
void turn(int m_speed, float center) {
	static int out_L = 0;
	static int out_R = 0;
	if (center > 0) {
		out_L = m_speed;
		out_R = -m_speed * (1 - center) / (1 + center);
	}
	else {
		out_L = -m_speed * (1 - center) / (1 + center);
		out_R = m_speed;
	}
	drive_motors(out_L, out_R);

	if (m_speed > 0) {
		Serial.println("TURN LEFT");
	}
	else {
		Serial.println("TURN RIGHT");
	}
}
void stop_Stop()    //モーターの回転を停止させる
{
	digitalWrite(dir1PinL, LOW);
	digitalWrite(dir2PinL, LOW);
	digitalWrite(dir1PinR, LOW);
	digitalWrite(dir2PinR, LOW);
	//Serial.print("  STOP!\n");
}

void keep(int ms)  //msミリ秒だけ現在の電圧をキープ、そして停止する。前はmovementという関数だった
{
	delay(ms);
	stop_Stop();
}

//指定した角度に方向を直す。goal_angleが角度の目標値。goal_angleは-180度～180度の範囲で指定
void set_angle(float goal_angle) {
	float d_angle;
	d_angle = calculate_d_angle(goal_angle);
  float d_angle_sum = 0;
	while (abs(d_angle) > SET_ANGLE_THRESHOLD) {
    d_angle_sum += d_angle;
		//目標角度に修正
		if (d_angle < 0) {
			turn(SET_ANGLE_SPEED, 0); //回転スピードの設定
		}
		else {
			turn(SET_ANGLE_SPEED*(-1), 0); //回転スピードの設定
		}

		keep((1 + E0 * cos(d_angle))*(abs(E1 * d_angle + E2 * d_angle_sum)));//回転時間の設定
		d_angle = calculate_d_angle(goal_angle);
//		Serial.print("SET ANGLE: ");
//		Serial.println(goal_angle);
//		Serial.print("\n");
	}
}

//制御

//フィードバック制御あり。
//角度を常に一定にキープしながら進むことができる。
//move_timeは移動時間。
//両輪の平均がm_speedとなる


void go_Feedback(int goal_angle, int move_time = 10000, float m_speed = 255)
{
  //float  d_center = 0;  //= D_CENTER*cos(goal_angle);
  //int state = 0;
	float e1 = 0; // 単位時間前の偏差
  int es = 0;  //偏差の積分
	//float e2 = 0; // 2単位時間前の偏差
	float e0 = 0;
	float u;
	float outL = m_speed;
	float outR = m_speed; //左右のモーターに出力される値
	float time_count = 0;  //time_countは経過した時間。これがmove_timeを下回っている間、while loopが回り続ける
	reset_dist();
  set_angle(goal_angle);

	drive_motors(m_speed, m_speed);
	delay(50);
	while (time_count < move_time) {
		//以降、d_angleを0に収束させるフィードバック制御：
		float d_angle = calculate_d_angle(goal_angle);
//		if (abs(d_angle) > 30) {  //あまりに違う方向を向いている場合はその場で回転して角度を修正
//			//端に乗り上げているケースが多いので一度バック
//			drive_motors(-m_speed, -m_speed);
//			keep(300);
//			set_angle(goal_angle);
//			stop_Stop();
//			delay(500);
//			e1 = 0;
//			e2 = 0;
//			outL = m_speed;
//			outR = m_speed;
//			drive_motors(m_speed, m_speed);
//			delay(50);
//      d_center=0;
//		}
//		else  // 以下PID制御
//		{
			//e2 = e1;
			e1 = e0;
			e0 = d_angle;
      es += d_angle;
      if(es > ES_MAX){
        es = ES_MAX;
      }
      else if (es < -ES_MAX){
        es = -ES_MAX;
      }


      u = e0 * Kp + es * Ki - (e0 - e1) * Kd;

      drive_motors(m_speed + u,m_speed - u);
//      //以下2値制御
//      Serial.println(d_angle);
//      if(d_angle>0.1){
//        drive_motors(m_speed+d_center+1,m_speed-d_center-1);
//        state ++;
//      }
//      else if(d_angle<-0.1){
//        drive_motors(m_speed-d_center-1,m_speed+d_center+1);
//        state--;
//      }else{
//        //state =0;
//      }
//
//      if(state> 10){
//        d_center+=1;
//        state=0;
//      }else if (state < -10){
//        d_center-=1;
//        state=0;
//      }

    if(m_speed>0){
			if (check_dist(forwardPin) == 1)break;
			}
			else{
		    if (check_dist(backPin) == 1)break;
		  }
		}
}


//消去アルゴリズム
void do_erase_all() {

	//(0)まず、何往復かを特定
	int i;
	int num_of_roundtrip = 3;
	//num_of_roundtrip = measure_vertical_length(89, 20000);
	drive_motors(255, 0);
	delay(100);
	set_angle(2);

	//(1)次にnum_of_roundtrip分だけ往復運動を左上から開始
	go_Feedback(0, 10000, 250);
	for (int i = 0; i < 1; i++) {
		drive_motors(-155, 155);
		delay(10);
		set_angle(0);
		go_Feedback(0, 1000, 250);

		drive_motors(155, -155);
		delay(10);
		set_angle(180);
		go_Feedback(180, 1000, 250);
	}
	//(2)右端に移動して
	drive_motors(155, -155);
	delay(10);
	set_angle(0);
	go_Feedback(0, 1000, 150);

	//(3)右下に移動
	drive_motors(-255, 0);
	delay(100);
	set_angle(89);
	go_Feedback(89, 20000, 150);
	drive_motors(0, 255);
	delay(100);
	set_angle(178);

	//(4)再びnum_of_roundtrip分だけ往復運動を左上から開始。ただし今回は上がる。
	for (int i = 0; i < 1; i++) {
		go_Feedback(180, 1000, 250);
		drive_motors(-255, -255);
		delay(500);
		drive_motors(255, 0);
		delay(1800);
		go_Feedback(0, 1000, 250);
		drive_motors(-255, -255);
		delay(500);
		drive_motors(0, 255);
		delay(1800);
	}

	//(5)左端に移動
	drive_motors(255, 255);
	delay(100);
	go_Feedback(178, 20000, 250); //左にまっすぐ進む。int goal_angle, int move_time

	//左上に移動
	drive_motors(255, 0);
	delay(100);
	set_angle(89);
	go_Feedback(89, 20000, 250);
	drive_motors(255, 0);
	delay(100);
	set_angle(0);
}

void do_erase_simple(){
  //set_angle(90);
  drive_motors(255,250);
  reset_dist();
  while(!check_dist(forwardPin)){
    delay(1);
  }
  drive_motors(-150,-255);
  delay(200);
  reset_dist();
  drive_motors(-245,-255);
  while(!check_dist(backPin)){
    delay(1);
  }
  stop_Stop();
  drive_motors(0,255);
  keep(250);
  
}


void setup()
{
	pinMode(dir1PinL, OUTPUT);
	pinMode(dir2PinL, OUTPUT);
	pinMode(speedPinL, OUTPUT);
	pinMode(dir1PinR, OUTPUT);
	pinMode(dir2PinR, OUTPUT);
	pinMode(speedPinR, OUTPUT);
	stop_Stop();

	//加速度センサを使うため
	setupMMA8451();

	/*
	//マイクロスイッチを使う
	pinMode(13, OUTPUT);
	pinMode(microswitch_pin, INPUT_PULLUP);
	*/

	//測距センサ
	pinMode(A0, INPUT);
	pinMode(13, OUTPUT); //LED点灯用
}

void test() {
  go_Feedback(0);  
  drive_motors(200,-255);
	delay(100);
  go_Feedback(180);  
  drive_motors(-255,-255);
  delay(100);
}


void loop() {
	//do_erase_all();
 //do_erase_simple();
 test();
}
