#include "ros/ros.h"
#include "std_msgs/String.h"
#include "test_thread.h"

#define PI 3.14159265
//Dynamixel Control



uint8_t dxl_error = 0;                          // Dynamixel error
uint8_t param_goal_position[4];      //목표위치를 저장하는 파라미터 배열(크기 4)를 선언한다.
int dxl_comm_result = COMM_TX_FAIL;
int32_t dxl1_present_position = 0;  //1번 모터의 현재위치를 0으로 한다. 
int32_t dxl2_present_position = 0;  //2번 모터의 현재위피를 0으로 한다. 


bool dxl_addparam_result = false;                // addParam result
bool dxl_getdata_result = false;

//Declare object
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
double i = 0;

bool is_run = true; //전역변수
int Control_Cycle = 10; //ms //컨트롤 주기를 10미리세크로 초기화한다.


double m = PI;
//시간 관련
double t = 0;
double dt = 10; //ms
double T = 3000; //1000ms=1s

double L1 = 0.12;   //1번 링크의 팔길이를 0.12로 초기화한다.
double L2 = 0.1;    //2번 링크의 팔길이를 0.1로 초기화한다. 

int dxl_present_posi1 = 0;  //15번 라인과 비교해보기 // 우선 모터1의 초기 위치를 초기화 한것으로 보임.
int dxl_goal_posi1 = 1000;   //13번 라인과 비교해보기 // 1번 모터의 목표 위치를 초기화 한것으로 보임.


//팔의 끝점의 정보를 담고있는 구조체 End_point선언 (멤버변수 x,y 총 두개를 담고있다.)
struct End_point {
  double x;
  double y;
};
//각 관절의 사잇각에 대한 정보를 담고있는 구조체 Joint 선언. (멤버변수 TH1,TH2 총 두개를 담고있다.)
struct Joint {
  double TH1;
  double TH2;
};



//functions

//main 함수 앞에 미리 추후에 정의 할 함수들을 선언해놓는다.
void processK(void);
void dxl_initailize(void);
void set_dxl_goal(int dxl_1_posi, int dxl_2_posi);
void dxl_go(void);
int radian_to_tick(double radian);
void read_dxl_postion(void);
void dxl_add_param(void);
struct End_point EP_goal;
struct End_point get_present_XY(void);
int radian_to_tick1(double radian);
int radian_to_tick2(double radian);
double tick_to_radian_1(int tick);
double tick_to_radian_2(int tick);
struct Joint J_goal;
struct Joint Compute_IK(struct End_point EP);


void msgCallback(const std_msgs::String::ConstPtr& msg)
{
MsgData Data;
Data.data=msg->data;
if(msg->data=="U"){
dxl_initailize();

//time_spec이라는 구조체의 객체인 next_time를 선언하였다. 

  static struct timespec next_time;
//static struct timespec curr_time;


//아래의 clock_gettime이라는 함수의 형식이다. 
//int clock_gettime(clockid_t clk_id, struct timespec *tp);

//첫번째 매개변수로 clk_id가 입력되는데 clk_id에는 여러가지 종류가있고 그에 대한 설명은 다음 주소에서 확인가능하다.
//https://hand-over.tistory.com/74

//우리는 이 중 CLOCK_MONOTONIC 라고하는 clk_id를 사용할 것인데 시계의 작동 기준은 다음과 같다.
//단조시계로 특정 시간부터 흐른 시간을 측정한다.(일반적으로 부팅이후 시간) 
//시스템 관리자는 이 값을 초기화 할 수 있습니다.

//다음은 두번째 매개변수로 입력되는 포인터변수 tp의 자료형인 timespec 구조체에 대한 선언내용이다. 

// struct timespec {
//         time_t    tv_sec;      /* seconds */ 
//         long      tv_nsec;     /* nanoseconds */
// };

//clock_gettime이라는 함수의 두번째 요소로 next_time(객체)의 주소를 입력해주었다.  

  clock_gettime(CLOCK_MONOTONIC, &next_time); 

  while(is_run) //1하면 종료가 잘 안됨. is run으로//is_run은 처음에 전역변수로 true라고 초기화 했었다. 
  {
    next_time.tv_sec += (next_time.tv_nsec + Control_Cycle * 1000000) / 1000000000;////ms
    next_time.tv_nsec = (next_time.tv_nsec + Control_Cycle * 1000000) % 1000000000;

     
     
     
    processK();
     
 
       

    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&next_time,NULL);
  }
 





}
else if(msg->data=="K"){






}
}

int main(int argc, char **argv)
{

ros::init(argc, argv, "topic_subscriber");

ros::NodeHandle nh;

ros::Subscriber thread =nh.subscribe("thread",100,msgCallback);

ros::spin();

return 0;


}

void processK(void) {

// 끝점에 대한 정보를 담고있는 구조체 End_point의 객체인 target과 EP_goal을 선언한다.
 static struct End_point target; 


 static struct End_point EP_goal;
  EP_goal.x = 0;
  EP_goal.y = 0.2;

  //J_goal = Compute_IK(EP_goal); //IK(역기구학 함수)에 좌표값을 넣으면 나오는 리턴 값을 J_goal에 저장한다.
  //set_dxl_goal(J_goal.TH1,J_goal.TH2); //위의 함수에서 결과 값으로 나온 세타값을 set_dxl_goal이라는 함수에 대입한다.
  //dxl_go();
  //groupSyncWrite.clearParam();

  read_dxl_postion();
  static struct End_point E;
  E =  get_present_XY();
  ROS_INFO("x:%lf, y:%lf",E.x,E.y);

  if (t <= T) {

  target.x =  E.x + (EP_goal.x - E.x )*0.5*(1 - cos(PI* t/T));
  target.y =  E.y + (EP_goal.y - E.y )*0.5*(1 - cos(PI* t/T));

  static struct Joint joint_goal;
  joint_goal = Compute_IK(target);
  set_dxl_goal(radian_to_tick1(joint_goal.TH1), radian_to_tick2(joint_goal.TH2));
  dxl_go();

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();

  
  }
  else{
   // t = 0;
  }

   if (t > T && t<6000) {

    read_dxl_postion();
  static struct End_point E;
  E =  get_present_XY();
  ROS_INFO("x:%lf, y:%lf",E.x,E.y);

EP_goal.x=0;
EP_goal.y=0.15;


  target.x =  E.x + (EP_goal.x - E.x )*0.5*(1 - cos(PI* (t-3000)/T));
  target.y =  E.y + (EP_goal.y - E.y )*0.5*(1 - cos(PI* (t-3000)/T));

  static struct Joint joint_goal;
  joint_goal = Compute_IK(target);
  set_dxl_goal(radian_to_tick1(joint_goal.TH1), radian_to_tick2(joint_goal.TH2));
  dxl_go();

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();

  
  }
  else{
   // t = 0;
  }
if (t > 6000 && t<9000) {

    read_dxl_postion();
  static struct End_point E;
  E =  get_present_XY();
  ROS_INFO("x:%lf, y:%lf",E.x,E.y);

EP_goal.x = 0.025+0.025*cos(m);
EP_goal.y = 0.15+0.025*sin(m);



  target.x =  E.x + (EP_goal.x - E.x )*0.5*(1 - cos(PI* (t-6000)/T));
  target.y =  E.y + (EP_goal.y - E.y )*0.5*(1 - cos(PI* (t-6000)/T));

  static struct Joint joint_goal;
  joint_goal = Compute_IK(target);
  set_dxl_goal(radian_to_tick1(joint_goal.TH1), radian_to_tick2(joint_goal.TH2));
  dxl_go();

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();

  m=m+0.01;
  
  }
  else{
   // t = 0;
  }
  if (t > 9000 && t<12000) {

    read_dxl_postion();
  static struct End_point E;
  E =  get_present_XY();
  ROS_INFO("x:%lf, y:%lf",E.x,E.y);

EP_goal.x=0.05;
EP_goal.y=0.2;


  target.x =  E.x + (EP_goal.x - E.x )*0.5*(1 - cos(PI* (t-9000)/T));
  target.y =  E.y + (EP_goal.y - E.y )*0.5*(1 - cos(PI* (t-9000)/T));

  static struct Joint joint_goal;
  joint_goal = Compute_IK(target);
  set_dxl_goal(radian_to_tick1(joint_goal.TH1), radian_to_tick2(joint_goal.TH2));
  dxl_go();

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();

  
  }
  else{
   // t = 0;
  }

  t=t+dt;
}

//open port, set baud, torqeu on dxl 1,2
void dxl_initailize(void) {

  portHandler->openPort();

  portHandler->setBaudRate(BAUDRATE);

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_add_param();
}

//goal position 지정
void set_dxl_goal(int dxl_1_posi, int dxl_2_posi) {

  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(0));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(0));

  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);

  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_LOWORD(0));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(0));

  dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
}

//통신보내주는 역할
void dxl_go(void) {
  dxl_comm_result = groupSyncWrite.txPacket();
}

struct Joint Compute_IK(struct End_point EP) {

  //IK soluttion
  /*
  struct Joint J;
  J.TH1 = atan2(sin(J.TH1), cos(J.TH2));
  J.TH2 = atan2(sqrt((1 - pow(cos(J.TH1), 2))), cos(J.TH2));
  return J;
  */

  double x = EP.x;
  double y = EP.y;
  double alpha = atan2(y,x);
  double L = sqrt(pow(x,2)+pow(y,2));
  double beta = acos((pow(L1,2)+pow(L2,2)-pow(L,2))/(2*L1*L2));
  double gamma = atan2(x,y);
  double delta = acos((pow(L1,2)+pow(L,2)-pow(L2,2))/(2*L1*L));

  double th2 = PI - beta;
  double th1 = (PI)/2 - gamma - delta;

  printf("%f , %f",th1,th2);

  struct Joint J;
  J.TH1 = th1;
  J.TH2 = th2;
  return J;
}


struct End_point Compute_FK(struct Joint J) {

  //FK soluttion

  struct End_point E;
  E.x = L1 * cos(J.TH1) + L2 * cos(J.TH1 + J.TH2);
  E.y = L1 * sin(J.TH1) + L2 * sin(J.TH1 + J.TH2);
  return E;
}


//라디안을 tick으로 변환해주는 함수이다. 
int radian_to_tick1(double radian){
  int tick=radian*(2048/PI)+1024;
  return tick;
}

int radian_to_tick2(double radian){
  int tick = radian*(2048/PI);
  if(tick < 0){
    tick += 4096;
  }
  return tick;
}

//tick을 라디안으로 변환해주는 함수이다.
//왜 1024를 더 할까? 아래 함수는 이해되지만 첫번째꺼는 이해가 안됨.
double tick_to_radian_1(int tick){
   double radian = (PI/2048)*(tick-1024);
   return radian;
}

double tick_to_radian_2(int tick){
  double radian = (PI/(double)2048)*tick;
  return radian;

}


//잘 모르겠지만 현재의 모터들의 위치값을 읽어오는 함수인듯하다.
void read_dxl_postion(void){

 dxl_comm_result = groupSyncRead.txRxPacket();
 dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
 dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
}


//잘 모르겠지만 1번 모터,2번모터의 파라미터를 읽어오는 함수인듯하다.
void dxl_add_param(void){

  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
    }
  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
  if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
    }
  }

struct End_point get_present_XY(void){

  struct Joint j;
  j.TH1 = tick_to_radian_1(dxl1_present_position) ;
  j.TH2 = tick_to_radian_2(dxl2_present_position);
  return Compute_FK(j);
}
