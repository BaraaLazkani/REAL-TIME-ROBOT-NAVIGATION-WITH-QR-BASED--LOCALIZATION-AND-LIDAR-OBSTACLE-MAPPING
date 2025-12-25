struct VelOutput {
  float v;
  float omega;
  bool done;
};
 
struct StateOut {
  float x;
  float y;
  float theta;
};


#include <BasicLinearAlgebra.h>
using namespace BLA;
byte i_s = 0;

BLA::Matrix<3,1> x_t = {0, 0, 0};
BLA::Matrix<3,3> P_t = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};
BLA::Matrix<2,2> Q_t = {0.04, 0, 0, 0.000064};
BLA::Matrix<3,3> R_t = {0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.1};
//BLA::Matrix<3,3> H = {1,0,0, 0,1,0, 0,0,1};
BLA::Matrix<3,3> I = {1,0,0, 0,1,0, 0,0,1};

float z_t[6][4];
String X="",Y="",T="",II="";
byte count=0;


////////////////////////////////////Aruco locations
const byte num_marker = 8;
float markers[num_marker][4] = {
  {1*0.6, 0*0.6, -M_PI/2, 7},{3*0.6, -2*0.6, -M_PI/2, 9},{4*0.6, -1*0.6, -M_PI/2, 8}
};
String x;   //to store serial input
StateOut ss = {0,0,0}; //to store the current state


StateOut prediction(float v, float imu_meas, float dt) {
  
  float theta = x_t(2, 0);
  x_t(0, 0) += dt * v * cos(theta);
  x_t(1, 0) += dt * v * sin(theta);
  x_t(2, 0) =  imu_meas;

  
  BLA::Matrix<3,3> f_x = {
    1, 0, -dt * v * sin(theta),
    0, 1, dt * v * cos(theta),
    0, 0, 1
  };

  BLA::Matrix<3,2> f_n = {
    dt * cos(theta), 0,
    dt * sin(theta), 0,
    0, dt
  };
  P_t = f_x * P_t * ~f_x + f_n * Q_t * ~f_n;
  
  
  return {x_t(0,0),x_t(1,0),x_t(2,0)}; 
}



bool get_tag_position(int tag_id, float* tag_pose) {
  for (int i = 0; i < num_marker; i++) {
    if ((int)markers[i][3] == tag_id) {
      tag_pose[0] = markers[i][0];
      tag_pose[1] = markers[i][1];
      tag_pose[2] = markers[i][2];
      return true;
    }
  }
  return false;
}

///////////////////MPU//////////////////////////////
#include "Wire.h"
#include <MPU6050_light.h>


MPU6050 mpu(Wire);

StateOut update(float z_t[][4], int z_count) {
  float tag_pose[3];

  for (int i = 0; i < z_count; i++) {
    if (!get_tag_position((int)z_t[i][3], tag_pose)) continue;

    float dtheta = tag_pose[2] - mpu.getAngleZ()*M_PI/180;

    BLA::Matrix<3,3> R_wt = {
      cos(tag_pose[2]), -sin(tag_pose[2]), tag_pose[0],
      sin(tag_pose[2]),  cos(tag_pose[2]), tag_pose[1],
      0, 0, 1
    };

    BLA::Matrix<3,3> R_ct = {
      cos(dtheta), -sin(dtheta), z_t[i][0],
      sin(dtheta),  cos(dtheta), z_t[i][1],
      0, 0, 1
    };

   
    BLA::Matrix<3,3> zz_matrix = R_wt * Inverse(R_ct) ;

    BLA::Matrix<3,1> zz = {
      zz_matrix(0,2),
      zz_matrix(1,2),
      atan2(zz_matrix(1,0), zz_matrix(0,0))
    };

    // K = P_t * H^T * (H * P_t * H^T + R)^-1
    BLA::Matrix<3,3> S = P_t  + R_t;
    BLA::Matrix<3,3> K = P_t * Inverse(S);
    x_t = x_t + K * (zz - x_t);
    P_t = (I - K ) * P_t;

  }
  
  return {x_t(0,0),x_t(1,0),x_t(2,0)};
}


byte nn=0;
#define in1 7
#define in2 6
#define ena1 5  // Left motor PWM
#define in3 8
#define in4 9
#define ena2 10  // Right motor PWM

// Robot parameters
    // rad/s (adjust based on your robot)
const float kp = 0.9;           // Proportional gain for distance
const float ka = 8;           // Gain for angle to goal
float threshold =0.22; 



VelOutput compute_vel(float state[3], float goal[2]) {
  
  // Calculate distance to goal

  float rho = sqrt( (goal[0] - state[0])*( goal[0] - state[0]) + (goal[1] - state[1])*(goal[1] - state[1]));
  
  // Calculate angles
  float alpha = -state[2] + atan2(goal[1] - state[1],  goal[0] - state[0]);
  
  // Compute velocities
  float v = kp * rho;
  float omega = ka * alpha;
  
  // Constrain outputs
  v = constrain(v, -0.836, 0.836);
  omega = constrain(omega, -12.863, 12.863);
  
  
  return {v, omega, rho < threshold};
}

// Set motor speeds (converts m/s to PWM)
void set_motor_speeds(float left_pwm, float right_pwm) {
  // Convert m/s to PWM (0-255)
  
  // Left motor
  digitalWrite(in1, left_pwm > 0 ? HIGH : LOW);
  digitalWrite(in2, left_pwm > 0 ? LOW : HIGH);
  analogWrite(ena1, abs(left_pwm));
  
  // Right motor
  digitalWrite(in4, right_pwm > 0 ? HIGH : LOW);
  digitalWrite(in3, right_pwm > 0 ? LOW : HIGH);
  analogWrite(ena2, abs(right_pwm));
    
   
 
}

float l_goalx = 10;
float l_goaly = 10;
float goalx = 3*0.6;
float goaly = -1*0.6;
int free_sectors[80];
bool flag_s = 0;
bool flag_d = 1;

void dynamic_window(){
  
  
  if (sqrt((ss.x-goalx)*(ss.x-goalx) + (ss.y-goaly)*(ss.y-goaly))<1){
    l_goalx = goalx;
    l_goaly = goaly;
    
  }
  else{
  
  float min_dist = 10000;
  float l_x, l_y, dist;
  int theta_sec;
  for(int i=0;i<i_s;i++){
    theta_sec = free_sectors[i];
    l_x = ss.x + 0.8*cos(3*M_PI/2 - theta_sec*M_PI/180 + ss.theta);
    l_y = ss.y + 0.8*sin(3*M_PI/2 - theta_sec*M_PI/180 + ss.theta);
    dist = sqrt((l_x-goalx)*(l_x-goalx) + (l_y-goaly)*(l_y-goaly));
    if (dist < min_dist){
      min_dist = dist;
      l_goalx = l_x;
      l_goaly = l_y;
      }
  }}
  Serial.print("l_goal ");
  Serial.print(l_goalx);
  Serial.print(" ");
  Serial.println(l_goaly);
}


float state[3] = {0, 0, 0};

 ////////////////
float timer = 0;
float serial_timer = 0;
float real_v = 0;
float dt;
/////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);
  
 
  while(!Serial.available() && Serial.readStringUntil('\n')!="START");
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena1, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena2, OUTPUT);

  ///////////////////////MPU////////////////////////////////
  Wire.begin();
  
  byte status = mpu.begin();
  
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  
  ////////////////////////////////////////////////////////
  timer = millis();
    
}
float transition_timer = 0;

void loop() {


  if(flag_s!=0 and flag_d==1){
    flag_d = 0;
    dynamic_window();
  }
 
  count=0;
    // Example goal (replace with your goal coordinates)
  // Compute velocities
  float aa[2] = {l_goalx,l_goaly};
  VelOutput dd = compute_vel(state, aa);
  //bool done = ;

  if(flag_s == 0){
    dd.v = 0;
    dd.omega = 0;
  }

    dt = (millis() - timer)/1000;
    
    real_v += -mpu.getAccX()*dt*9.81;
    timer = millis();
    if (real_v<0) real_v = 0;
   
    //Serial.println(Serial.available());
    if(Serial.available()>25){
    while(Serial.available()>0){Serial.read();}}
      
  
    if(Serial.available()){ 
    x = Serial.readStringUntil('#');
    
    if(x[0]=='s'){
      Serial.println(x);
      i_s = 0;
      flag_s = 1;
      while(x.length()>1){
        x.remove(0,1);
        String sect = "";
        while(x[0]!='s'){
          sect += x[0];
          x.remove(0,1);}
        free_sectors[i_s] = sect.toInt();
        i_s++;
       
      }
      
    }
    
    else if(x[1]=='x'){
    
    
    nn=x[0]-'0';
    x.remove(0,1);
    while(nn>0){
    count=count+1; 
    X="";
    Y="";
    T="";
    II="";
    
    if(x[0] == 'x'){
      x.remove(0, 1);
      while(x[0]!='y'){
        X+=x[0];
        x.remove(0, 1);
        }
        x.remove(0, 1);
        while(x[0]!='t'){
        Y+=x[0];
        x.remove(0, 1);
        }
        x.remove(0, 1);
        while(x[0]!='i'){
        T+=x[0];
        x.remove(0, 1);
        }
        x.remove(0, 1);
        while(x[0]!='x' && x.length()>0){
        II+=x[0];
        x.remove(0, 1);
        }
        z_t[count-1][0]=X.toFloat();
        z_t[count-1][1]=Y.toFloat();
        z_t[count-1][2]=T.toFloat();
        z_t[count-1][3]=II.toInt();
        
    }
    nn=nn-1;
    }

  }}
  
 mpu.update();

    if (X!=""){
      ss = prediction((real_v+dd.v)*0.5, mpu.getAngleZ()*M_PI/180, dt);
      ss = update(z_t, count);
      }
      
 
    else{
      ss = prediction((real_v+dd.v)*0.5, mpu.getAngleZ()*M_PI/180, dt);
   
      }
      
      state[0] = ss.x;
      state[1] = ss.y;
      state[2] = ss.theta;
      
      if(millis()-serial_timer>50){
      Serial.print(state[0]);
      Serial.print(" ");
      Serial.print(state[1]);
      Serial.print(" ");
      Serial.println(state[2]);
      
      serial_timer = millis();
      }
  
  
  if (!dd.done or !(millis()-transition_timer)>3000) {
    // Convert to wheel speeds
    float left_speed = (2.0 * dd.v - dd.omega * 0.13) / (2.0);
    float right_speed = (2.0 * dd.v + dd.omega * 0.13) / (2.0);
    left_speed = constrain((left_speed*150/0.492),-255,255);
    right_speed = constrain((right_speed*150/0.492),-255,255);
    
    // Drive motors
    set_motor_speeds(left_speed, right_speed);


  } else {
    transition_timer = millis();
    flag_d = 1;
    Serial.println("transition ");
    if (sqrt((ss.x-goalx)*(ss.x-goalx) + (ss.y-goaly)*(ss.y-goaly))<threshold){
    set_motor_speeds(0, 0);
    Serial.println("Goal reached!");
    while(1);}
  }
  
  
   // Control loop delay
  X="";
  delay(10); 
}
