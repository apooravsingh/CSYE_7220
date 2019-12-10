#include <Servo.h>
#include <IRremote.h>
#define IN1  8   //K1、K2 motor direction
#define IN2  9     //K1、K2 motor direction
#define IN3  10    //K3、K4 motor direction
#define IN4  12   //K3、K4 motor direction
#define ENA  5    // Needs to be a PWM pin to be able to control motor speed ENA
#define ENB  6    // Needs to be a PWM pin to be able to control motor speed ENB
#define LED1 2  //lefe led connect to D2
#define LED2 3  //right led connect to D3
#define SERVO     11  //servo connect to D11
#define IRPIN  13 //IR receiver Signal pin connect to Arduino pin 13
#define echo    A3 // Ultrasonic Echo pin connect to A2
#define trig    A2  // Ultrasonic Trig pin connect to A3
#define buzzer     7 //buzzer connect to D7
#define RSPEED   255  //right motor speed
#define LSPEED   255  //left motor speed
#define IR_AVOIDANCE          0x00FF38C7       //code from IR controller "OK" button
#define IR_STOP               0x00FF9867       //code from IR controller "0" button
enum DN
{
  START_AVOIDANCE,//start avoidance
  STOP_AVOIDANCE,//stop avoidance
  DEF
}Drive_Num=DEF;
int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 30; //distance limit for obstacles in front
const int sidedistancelimit = 18; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
Servo head;
IRrecv IR(IRPIN);  //   IRrecv object  IR get code from IR remoter
decode_results IRresults;
/motor control/
void go_ahead()//go ahead
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4,HIGH);
  //delay(t);
}
void go_back() //go back
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4,LOW);
  //delay(t);
}
void go_stop() //stop
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4,LOW);
}
void turn_left()//turn left
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  //delay(t);
}
void turn_right()//turn right
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  //delay(t);
}
/*set motor speed */
void set_motorspeed(int lspeed,int rspeed)
{
  analogWrite(ENA,lspeed);
  analogWrite(ENB,rspeed);
}
void buzz_on()   //open buzzer
{
  digitalWrite(buzzer, LOW);
}
void buzz_off()  //close buzzer
{
  digitalWrite(buzzer, HIGH);
}
void alarm() {
  buzz_on();
  delay(100);
  buzz_off();
}
/**control led**/
void open_led(int led_num)
{
  if (led_num == 1)  digitalWrite(LED1,LOW);
  else digitalWrite(LED2,LOW);
}
void close_led(char led_num)
{
   if (led_num == 1)  digitalWrite(LED1,HIGH);
   else digitalWrite(LED2,HIGH);
}
/detection of ultrasonic distance/
int watch() {
  long howfar;
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(15);
  digitalWrite(trig, LOW);
  howfar = pulseIn(echo, HIGH);
  howfar = howfar * 0.01657; //how far away is the object in cm
  Serial.println((int)howfar);
  return round(howfar);
}
/*****detect IR code******/
void do_IR_Tick()
{
  if(IR.decode(&IRresults))
  {
    if(IRresults.value==IR_AVOIDANCE)
    {
      Drive_Num=START_AVOIDANCE;
    }
    else if(IRresults.value==IR_STOP)
    {
       Drive_Num=STOP_AVOIDANCE;
    }
    IRresults.value = 0;
    IR.resume();
  }
}

void auto_avoidance() {
  head.write(90);
  delay(100);
  centerscanval = watch();
  if (centerscanval >= distancelimit) {
    set_motorspeed(LSPEED, RSPEED);
    go_ahead();
  }
  else {
    go_stop();
    alarm();
    head.write(120);
    delay(150);
    ldiagonalscanval = watch();

    head.write(180);
    delay(150);
    leftscanval = watch();

    head.write(90);
    delay(150);

    head.write(60);
    delay(150);
    rdiagonalscanval = watch();

    head.write(0);
    delay(150);
    rightscanval = watch();

    head.write(90);
    if (ldiagonalscanval >= sidedistancelimit && leftscanval >= sidedistancelimit) {
      set_motorspeed(LSPEED, RSPEED);
      go_back();
      delay(200);
      turn_left();
      delay(600);
    }
    else if (rdiagonalscanval >= sidedistancelimit && rightscanval >= sidedistancelimit) {
      set_motorspeed(LSPEED, RSPEED);
      go_back();
      delay(200);
      turn_right();
      delay(600);
    }
  }
}

void setup() {
  /*L298N*/
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  /*LED*/
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  close_led(1),close_led(2);//close led
  /init HC-SR04/
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  digitalWrite(trig, LOW);
  /init buzzer/
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);
  buzz_off();
  /init servo/
  head.attach(SERVO);
  head.write(90);
  pinMode(IRPIN, INPUT);
  digitalWrite(IRPIN, HIGH);
  IR.enableIRIn();
  /baud rate/
  Serial.begin(9600);
  go_stop();
}

void loop() {
   do_IR_Tick();
  if(Drive_Num == START_AVOIDANCE)//remote controller press "OK",robot will avoid obstacles
    auto_avoidance();
  else if(Drive_Num == STOP_AVOIDANCE)//remote controller press "0",robot will stop move
    go_stop();
}