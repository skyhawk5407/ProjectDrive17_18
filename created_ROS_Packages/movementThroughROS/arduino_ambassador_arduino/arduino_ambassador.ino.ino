#include <Servo.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>

#define SERVO_PIN 2
#define ESC_PIN 3

const float NEUTRAL_THROTTLE = 90; // 0 to 180
const float NEUTRAL_STEERING_ANGLE = 90; // 60 to 130

ros::NodeHandle nh;
std_msgs::Int32 servoValue;
ros::Publisher pub("arduino_msg", &servoValue);

const int STEERING_SERVO_PIN = 2;
const int ESC_SERVO_PIN = 3;// was 10
const int PIN_LED = 13;
const int PIN_STATE = 6;
const int TX = 4;
const int RX = 5;

const int SDA_PIN = 18;
const int SCL_PIN = 19;

const float NEUTRAL_THROTTLE = 90; // 60 to 102
const float NEUTRAL_STEERING_ANGLE = 90; // 50 to 130
const boolean if_Bluetooth; // true if using bluetooth, false if using ROS

Servo steeringServo;
Servo electronicSpeedController;
SoftwareSerial BTserial(TX, RX); // Pin4 to TX  | Pin5 to RX

void updateMotor( const geometry_msgs::Vector3& command) {
  if (if_Bluetooth)
  {
    int servo = command.x;
    int throttle = command.y;

    // constrain values into safe zone
    servo = min(servo, 130);
    servo = max(servo, 60);

    throttle = min(throttle, 180);
    throttle = max(throttle, 0);

    steeringServo.write(servo);
    electronicSpeedController.write(throttle);
  }
  else
  {
    //connect steering servo and esc
    steeringServo.attach( SERVO_PIN );
    electronicSpeedController.attach( ESC_PIN );

    //set speeds to neutral
    steeringServo.write( NEUTRAL_STEERING_ANGLE );
    electronicSpeedController.write( NEUTRAL_THROTTLE );

    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub);
  }
}

ros::Subscriber<geometry_msgs::Vector3> sub("arduino_talker", &updateMotor );

void setup()
{
  //connect steering servo and esc
  steeringServo.attach( STEERING_SERVO_PIN );
  electronicSpeedController.attach( ESC_SERVO_PIN );

  //set speeds to neutral
  steeringServo.write( NEUTRAL_STEERING_ANGLE );
  electronicSpeedController.write( NEUTRAL_THROTTLE );

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_STATE, INPUT);
  Serial.begin(9600);
  BTserial.begin(38400);

  delay(1000);
  BTserial.println("Neutral is 90");
  BTserial.println("S = speed, R = rotate, G = go");
  BTserial.println("Rotate values: 50 to 130 and Speed values 60 to 102");
  BTserial.println("E.g: S100G.");
}

int incomingByte = 0;
int number = 0;
boolean setCarSpeed = false; // true = speed, false = rotate
boolean pause = false;
boolean alreadyPaused;

void loop()
{
  if (if_Bluetooth)
  {
    boolean bt_Connected = digitalRead(PIN_STATE);

  
    if (bt_Connected == false) // if BT disconnected
    {
      while (BTserial.available() > 0) //clear any buffer leftovers when BT disconnected
        // Serial.print("Buffer extras: "), Serial.print(BTserial.read()); //serial debugging...
        digitalWrite(PIN_LED, LOW);
      pause = false;
      electronicSpeedController.write(90);
      steeringServo.write(90);
    }
    else if (bt_Connected == true)
    {
      digitalWrite(PIN_LED, HIGH); // connected and on

      /**************************
         Loop: speed/turn
       *************************/

      if ( BTserial.available() > 0 )
      {

        incomingByte = BTserial.read();

        if (incomingByte == 80) // P to stop
        {
          if (pause == true) // if already paused, 2nd pause turns it on
          { pause = false;
            BTserial.println("pause off.");
          }

          else if (pause == false) // if not paused, pause on
          {
            pause = true; BTserial.println("pause on.");
            electronicSpeedController.write(90);
            steeringServo.write(90);
          }
        }

        else if (pause == true) // not paused (normal function of S90G applies)
        {

          if (incomingByte == 83) {
            setCarSpeed = true;
            number = 0;
          }
          else if (incomingByte == 82) {
            setCarSpeed = false;
            number = 0;
          }
          else if (incomingByte == 71) {
            BTserial.print("-> ");

            if (setCarSpeed == true) {
              BTserial.print("Setting car speed to ");
              electronicSpeedController.write(number);
            }

            else {
              BTserial.print("Setting rotation to ");
              steeringServo.write(number);
            }
            BTserial.println(number);
          }

          else {
            number = number * 10 + (incomingByte - '0'); // setting the numerical speed    //  for numbers 90 its 9 then * 10.
          }
          delay(100);
        }
      }

    } // bt connected
  }

  else
  { nh.spinOnce();
    delay(1);
  }

}
