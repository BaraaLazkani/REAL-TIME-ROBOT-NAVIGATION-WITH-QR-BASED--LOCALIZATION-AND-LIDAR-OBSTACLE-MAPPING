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

BLA::Matrix<3,1> x_t = {0, 0, 0};
BLA::Matrix<3,3> P_t = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};
BLA::Matrix<2,2> Q_t = {0.04, 0, 0, 0.000064};
BLA::Matrix<3,3> R_t = {0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.1};
BLA::Matrix<3,3> H = {1,0,0, 0,1,0, 0,0,1};
BLA::Matrix<3,3> I = {1,0,0, 0,1,0, 0,0,1};

const int MAX_POINTS = 10;  // Adjust based on your needs
float z_t[MAX_POINTS][4];
String X="",Y="",T="",II="";
float xarr[MAX_POINTS];
float yarr[MAX_POINTS];
float tarr[MAX_POINTS];
float iarr[MAX_POINTS];
float last_time = 0.0;
int count=0;


////////////////////////////////////Aruco locations
const int num_marker = 9;
float markers[num_marker][4] = {
  {3*0.4, 4*0.4, -M_PI/2, 6},{4*0.4, -2*0.4, -M_PI/2, 7},{9*0.4, 3*0.4, -M_PI/2, 9},{11*0.4, -1*0.4, -M_PI/2, 10},{13*0.4, 2*0.4, -M_PI/2, 12},{15*0.4, -1*0.4, -M_PI/2, 1},{2*0.4, 0*0.4, -M_PI/2, 8},{18*0.4, 2*0.4, -M_PI/2, 0}
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
    int id = (int)z_t[i][3];
    if (!get_tag_position(id, tag_pose)) continue;

    float dx = z_t[i][0];
    float dy = z_t[i][1];
    float dtheta = tag_pose[2] - mpu.getAngleZ()*M_PI/180;

    float cos_world = cos(tag_pose[2]);
    float sin_world = sin(tag_pose[2]);
    float cos_robot = cos(dtheta);
    float sin_robot = sin(dtheta);

    BLA::Matrix<3,3> R_wt = {
      cos_world, -sin_world, tag_pose[0],
      sin_world,  cos_world, tag_pose[1],
      0, 0, 1
    };

    BLA::Matrix<3,3> R_ct = {
      cos_robot, -sin_robot, dx,
      sin_robot,  cos_robot, dy,
      0, 0, 1
    };

   
    BLA::Matrix<3,3> zz_matrix = R_wt * Inverse(R_ct) ;

    BLA::Matrix<3,1> zz = {
      zz_matrix(0,2),
      zz_matrix(1,2),
      atan2(zz_matrix(1,0), zz_matrix(0,0))
    };

    // K = P_t * H^T * (H * P_t * H^T + R)^-1
    BLA::Matrix<3,3> S = H * P_t * ~H + R_t;
    BLA::Matrix<3,3> K = P_t * ~H * Inverse(S);
    x_t = x_t + K * (zz - H * x_t);
    P_t = (I - K * H) * P_t;

  }
  
  return {x_t(0,0),x_t(1,0),x_t(2,0)};
}


int nn=0;
#define in1 7
#define in2 6
#define ena1 5  // Left motor PWM
#define in3 8
#define in4 9
#define ena2 10  // Right motor PWM

// Robot parameters
const float MAX_SPEED = 0.836;    // m/s (adjust based on your robot)
const float MAX_OMEGA = 12.863;    // rad/s (adjust based on your robot)
const float WHEEL_RADIUS = 0.032; // meters (adjust to your robot)
const float WHEEL_BASE = 0.13;   // meters (distance between wheels, adjust)
const float kp = 0.9;           // Proportional gain for distance
const float ka = 20;           // Gain for angle to goal
const float kb = 0;          // Gain for final orientation
float threshold =0.25; 



VelOutput compute_vel(float state[3], float goal[2]) {
  float x = state[0];
  float y = state[1];
  float theta = state[2];
  
  // Calculate distance to goal
  float dx = goal[0] - x;
  float dy = goal[1] - y;
  float rho = sqrt(dx*dx + dy*dy);
  
  // Calculate angles
  float alpha = -theta + atan2(dy, dx);
  float beta = -theta - alpha;
  
  // Compute velocities
  float v = kp * rho;
  float omega = ka * alpha + kb * beta;
  
  // Constrain outputs
  v = constrain(v, -MAX_SPEED, MAX_SPEED);
  omega = constrain(omega, -MAX_OMEGA, MAX_OMEGA);
  
  // Check if goal reached
  bool done = (rho < threshold);  // 5cm threshold
  
  return {v, omega, done};
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
float state[3] = {0, 0, 0};
int point = 0;
float goal[3][3] = {{0.353, -0.353},{0.806,-0.564},{1.6,0}}; ////////////////
 ////////////////
float timer = 0;
float real_v = 0;
float dt;
/////////////////////////////////////////////////

void setup() {

  Serial.begin(9600);
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
void loop() {
  mpu.update();
  count=0;
    // Example goal (replace with your goal coordinates)
  // Compute velocities
  float aa[2] = {goal[point][0],goal[point][1]};
  VelOutput dd = compute_vel(state, aa);
  float v = dd.v;
  float omega = dd.omega;
  bool done = dd.done;
  
  if (!done) {
    // Convert to wheel speeds
    float left_speed = (2.0 * v - omega * WHEEL_BASE) / (2.0);
    float right_speed = (2.0 * v + omega * WHEEL_BASE) / (2.0);
    float left_pwm = constrain((left_speed*150/0.492),-255,255);
    float right_pwm = constrain((right_speed*150/0.492),-255,255);
    
    // Drive motors
    set_motor_speeds(left_pwm, right_pwm);

    dt = (millis() - timer)/1000;
    real_v += -mpu.getAccX()*dt*9.81;
    timer = millis();
    if (real_v<0) real_v = 0;
    
    //Serial.print("v ");
    //Serial.print(v);
    //Serial.print(" ");
    //Serial.println(real_v);
    
    if(Serial.available()){ 
    x = Serial.readStringUntil('#');

    if(x[0]=='s'){
      Serial.println(x);
    }
    else{
    
    Serial.println(x);
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
        xarr[count-1]=X.toFloat();
        yarr[count-1]=Y.toFloat();
        tarr[count-1]=T.toFloat();
        iarr[count-1]=II.toInt();
        
    }
    nn=nn-1;
    }

  }}
  
 
for (int i = 0; i < count; i++) {
    z_t[i][0] = xarr[i];  // x
    z_t[i][1] = yarr[i];  // y
    z_t[i][2] = tarr[i];  // t
    z_t[i][3] = iarr[i];  // i
  }


    if (X!=""){
      ss = prediction((real_v), mpu.getAngleZ()*M_PI/180, dt);
      ss = update(z_t, count);
      }
      
 
    else{
      ss = prediction((real_v), mpu.getAngleZ()*M_PI/180, dt);
   
      }
      
      state[0] = ss.x;
      state[1] = ss.y;
      state[2] = ss.theta;
      Serial.print(state[0]);
      Serial.print(" ");
      Serial.print(state[1]);
      Serial.print(" ");
      Serial.println(state[2]);
    

    
  } else {
    // Stop when reached goal
    point += 1;
    Serial.print("transition ");
    Serial.println(point);
    if (point==(sizeof(goal)/sizeof(goal[0]))){
    set_motor_speeds(0, 0);
    Serial.println("Goal reached!");
    while(1);}
  }
  


  delay(10);  // Control loop delay
  X="";
  
}
