//Includeing Libaries
#include <ax12.h> //arbotix libary
#include <dynamixel_mx.h> //actuator libary
//**********************************

//Giving the functions of the dynamixel libary the function of ax12 functions
#define EnableTorqueMode(id) (ax12SetRegister(id, 70, 1))                 //Defines the function enabling the torque of joints
#define DisableTorqueMode(id) (ax12SetRegister(id, 70, 0))                //Defines the function disabling the torque of joints
#define GetTorqueMode(id) (ax12GetRegister(id, 70, 1))                    //Defines the function getting the torque of joints
#define SetSpeed(id, velocity) (ax12SetRegister2(id, 32, velocity))       //Defines the function setting the speed of joints
//*****************************

//Global Variable Declaration 

//Variables for Gripper 
int Speed = 0;                                              //Variable holding the Speed of the Joints 
//Position of gripper fingers  
int pos5close = 2080;                                   //The position of joint 5 when the gripper is closed
int pos4close = 2019;                                   //The position of joint 4 when the gripper is closed
int pos4open = 3050;                                    //The position of joint 4 when the gripper is open
int pos5open = 1010;                                    //The position of joint 5 when the gripper is open
int pos4current;                                        //The current position of joint 4
int pos5current;                                        //The current position of joint 5
int pos5min = 752;                                      //The value of the joint 5 in the most open position possible  \_
int pos4max = 3334;                                     //The value of the joint 4 in the most open position possible   _/
i
//
float speedMaxGripper = 200;                                //Variable holding the maximal gripper speed
float speedMinGripper = 30;                                 //Variable holding the minimal gripper speed
float speedMax = 40;                                        //Variable holding the maximal speed of the joints 1->3
float speedMin = 10;                                        //Variable holding the minimal speed of the joints 1->3
float highLimCCW = 300;                                     //Variable holding the reference value of the upper-limit of the muscle input for the counterclockwise direction  
float lowLimCCW = 300;                                      //Variable holding the reference value of the lower-limit of the muscle input for the counterclockwise direction
float lowLimCW = 300;                                       //Variable holding the reference value of the lower-limit of the muscle input for the clockwise direction
float highLimCW = 300;                                      //Variable holding the reference value of the upper-limit of the muscle input for the clockwise direction

//Controller 
int changeID = 0;                                           //Boolean coded as integer, 0 if joint switching is enabled, 1 if disabled
int TorqueModeState = 0;                                    //Variable holding the state of the TorqueMode
int id = 1;                                                 //Joint ID
float currentSpeed = 0;                                     //Variable holding the current speed
float error = 0;                                            //Variable holding the value of the feedback Path
float torque = 0;                                           //Variable holding the value of the torques 
float kp = 20;                                              //Variable holding the value of the proportional 

//Variables for DataPackages
unsigned int incomingByte = 0;                              //For incoming serial data
unsigned int package[24];                                   //Array of size 24 for the data of the sensorbox
int packageDone = 0;
int checkvalue = 0;                                         //Checksum
int envelopeCounter = 0;                                    //Variable counting at which position the envelope value has to be saved
const int envelopeFactor = 5;                               //Defining the size of one dimention of the 2 dimensional array
int resSignalEnvelope[envelopeFactor][5];                   //Stores the last data values from the sensor to find the average
int fresh = 1, res;                                         //Status of package finding process 
int resSignal[5];                                           //Array which holds the values of the senors 
int msbz, lsbz;                                             //Variable for most and least significant byte of ACC in Z-direction
int msby, lsby;                                             //Variable for most and least significant byte of ACC in Y-direction
int msbx, lsbx;                                             //Variable for most and least significant byte of ACC in X-direction
int msbemg1, lsbemg1;                                       //Variable for most and least significant byte of EMG1 channel 
int msbemg2, lsbemg2;                                       //Variable for most and least significant byte of EMG2 channel
int res1, res2, res3, res4, res5;                           //Variables of the sum of the most and least significant bytes 

//**********************************                            
//Function Declartaion                                          

//Setup Functions                                                       

//Gets the torque mode from the joint which is currently selected to move.
void checkTorqueMode(void)
{                                                               
  Serial.print(id);
  Serial.print(" ");                                              
  TorqueModeState = GetTorqueMode(id);                    //GetTorqueMode(id) is a predefined function from ax.h
  delay(2);
}

//initialises the envelope 2 dimensional array and puts every entry to 0.
void initEnvelop(void)
{ 
  for(int i = 0; i < envelopeFactor; i++)
  {
    for(int j = 0; j < 5; j++)
    {
      resSignalEnvelope[i][j] = 0;
    }  
  }
}

//*******************************
//PACKAGE FUNCTION

//reads incomming bytes
int readIncoming(void)
{
  while(Serial.available()<=0)
  {
  }
  incomingByte = Serial.read();
  return incomingByte;
}

/**searches for the first 5 bytes of the data package and collects the rest of the package  returns res = 0 when the package is complete and the checksum is okay
*/
int getPackage(int fresh, unsigned int d[])
{
  int c,i; //initialization of the varable c and and the vector index i

    //FIRST Try
  if(fresh == 1)
  {
    //search for 7E  
    do
    {  
      c = readIncoming();
    }
    while(c != 126);
    d[0] = 126;                                           //Put the number 126 (=7E) in the 1st position of the vector
    //******************************************************
    c=readIncoming();                                     //Read the next byte 
    //if next byte is not 0 check if it is 7E 
    if(c != 0)                                            
    {
      if(c == 126)                                        //But if 7E goto second try
      {
        return -2;
      }
      else                                                //If not 7E start over at try 1
      {
        return -1; 
      }
    }
    d[1] = c;                                             //Puts the number 0 into the 2nd position of the vector
    //******************************************************
    c = readIncoming();                                   //Read the next byte
    //if next byte is not 20 check if it is 7E
    if(c != 20)    
    {
      if(c == 126)                                        //But if 7E goto second try
      {
        return -2;
      }
      else                                                //If not 7E start over at try 1
      {
        return -1;
      }
    }
    d[2]=c;                                               //Puts the number 20 into the 3rd position of the vector
    //******************************************************
    c=readIncoming();                                     //Read the next byte
    //if next byte is not 131
    if(c != 131)
    {
      if(c == 126)                                        //But if 7E goto second try
      {
        return -2;
      }
      else                                                //If not 7E start over at try 1
      {
        return -1; 
      }
    }
    d[3] = c;                                             //Puts the number 131 into the 4th position of the vector
    //******************************************************
    c=readIncoming();                                     //Read the next byte
    //if next byte is not 86 
    if(c != 86)
    {
      if(c == 126)                                        //But if 7E goto second try
      {
        return -2;
      }
      else                                                //If not 7E start over at try 1
      {
        return -1; 
      }
    }
    d[4] = c;                                             //Puts the number 86 into the 5th position of the vector
    //******************************************************
    c=readIncoming();                                     //Read the next byte
    //if next byte is not 120
    if(c != 120)
    {
      if(c == 126)                                        //But if 7E goto second try
      {
        return -2;
      }
      else                                                //If not 7E start over at try 1
      {
        return -1; 
      }
    }
    d[5]=c;                                               //Puts the number 120 into the 6th position of the vector
    //******************************************************
    //If the first 5 bytes are correct fill the rest of the package in the vector
    for(i=6; i<24;i++)
    {
      d[i] = readIncoming();
    }
    //******************************************************
    checkvalue = 0;                                    

    for(int j = 3; j < 24; j++)                           //Start at the 4th position and add all following numbers together 
    {
      checkvalue = checkvalue+d[j];
    }
    checkvalue = checkvalue%256;                         //Divide by FF

    //if checkvalue is ok return 0 
    if(checkvalue == 255)
    {
      return 0;
    }
    else                                                  //Start over
    {
      return -1;
    }
  } 

  //SECOND Try
  else if(fresh == 0)
  {
    c=readIncoming();                                     //Read the next byte
    //if next byte is not 0 
    if(c != 0)
    { 
      if(c == 126)                                        //But if 7E goto second try
      {
        return -2;
      }
      else                                                //If not 7E start over at try 1
      {
        return -1; 
      }
    }
    d[1]=c;                                               //Puts the number 7E into the 2st position of the vector
    //******************************************************
    c=readIncoming();                                     //Read the next byte
    //if next byte is not 20
    if(c != 20)
    {
      if(c == 126)                                        //But if 7E goto second try
      {
        return -2;
      }
      else                                                //If not 7E start over at try 1
      {
        return -1;
      }
    }
    d[2] = c;                                             //Puts the number 20 into the 3nd position of the vector
    //******************************************************
    c=readIncoming();                                     //Read the next byte
    //if next byte is not 131
    if(c != 131)
    {
      if(c == 126)                                        //But if 7E goto second try
      {
        return -2;
      }
      else                                                //If not 7E start over at try 1
      {
        return -1;
         
      }
    }
    d[3]=c;                                               //Puts the number 131 into the 4rd position of the vector
    //******************************************************
    c=readIncoming();                                     //Read the next byte
    //if next byte is not 86
    if(c != 86)
    {
      if(c == 126)                                        //But if 7E goto second try
      {
        return -2; 
      }
      else                                                //If not 7E start over at try 1
      {
        return -1; 
      }
    }
    d[4] = c;                                             //Puts the number 86 into the 5th position of the vector
    //******************************************************
    c=readIncoming();                                     //Read the next byte
    //if next byte is not 120
    if(c != 120)
    { 
      if(c == 126)                                        //But if 7E goto second try
      {
        return -2; 
      }
      else                                                  //If not 7E start over at try 1
      {
        return -1; 
      }
    }
    d[5]=c;                                               //Puts the number 86 into the 6th position of the vector
    //******************************************************
    //If the first 5 bytes are correct fill the rest of the package in the vector
    for(i = 6; i < 24; i++)                               
    {
      d[i] = readIncoming();
    }
    checkvalue = 0;
    for(int j = 3; j < 24; j++)                           //Start at the 4th position and add all following numbers together
    {
      checkvalue = checkvalue + d[j];
    }

    checkvalue = checkvalue%256;                     //Divide by FF

    if(checkvalue == 255)
    {
      return 0;                                           //checkvalue is OK 
    }
    else 
    {
      return -1;                                          //Start over
    }
  }
}
//puts the low and high bytes from the sensor data together and puts them into an array.
void assemblePackage(void)
{ 
  //ACC Z-Direction
  msbz = package[13];                                     //Store the 14th position of the package array in the variabel 
  lsbz = package[14];                                     //Store the 15th position of the package array in the variabel
  msbz = msbz<<8;                                         //assembling the byte
  res1 = msbz|lsbz;                                       //Summing the most and least significant bits with a bitwise OR operator
  resSignal[0] = res1;                                    //Puts the value into the 1st of the resSignal array

  //ACC Y-Direction
  msby = package[15];                                     //Store the 16th position of the package array in the variabel
  lsby = package[16];                                     //Store the 17th position of the package array in the variabel
  msby = msby<<8;                                         //assembling the bytes
  res2 = msby|lsby;                                       //Summing the most and least significant bits with a bitwise OR operator
  resSignal[1] = res2;                                    //Puts the value into the 1st of the resSignal array

  //ACC X-Direction
  msbx = package[17];                                     //Store the 18th position of the package array in the variabel
  lsbx = package[18];                                     //Store the 19th position of the package array in the variabel
  msbx = msbx<<8;                                         //assembling the bytes
  res3 = msbx|lsbx;                                       //Summing the most and least significant bits with a bitwise OR operator
  resSignal[2] = res3;                                    //Puts the value into the 1st of the resSignal array

  //EMG1 channel
  msbemg1 = package[19];                                  //Store the 20th position of the package array in the variabel
  lsbemg1 = package[20];                                  //Store the 21th position of the package array in the variabel
  msbemg1 = msbemg1<<8;                                   //assembling the bytes
  res4 = msbemg1|lsbemg1;                                 //Summing the most and least significant bits with a bitwise OR operator
  resSignal[3] = res4;                                    //Puts the value into the 1st of the resSignal array

  //EMG2 channel
  msbemg2 = package[21];                                  //Store the 22th position of the package array in the variabel
  lsbemg2 = package[22];                                  //Store the 23th position of the package array in the variabel
  msbemg2 = msbemg2<<8;                                   //assembling the bytes
  res5 = msbemg2|lsbemg2;                                 //Summing the most and least significant bits with a bitwise OR operator
  resSignal[4] = res5;                                    //Puts the value into the 1st of the resSignal array
}
//Puts the sensors data array into a two dimentional array and calculates an average for each sensor input consisting of the last (envelopefactor) values.
void envelopeSensorData(void)
{
  for(int i = 0; i < 5 ; i++)
  {
    resSignalEnvelope[envelopeCounter][i] = resSignal[i];
    int tempValue = 0;
    for(int j = 0; j < envelopeFactor; j++)
    {
      tempValue = tempValue + resSignalEnvelope[j][i];
    }
    resSignal[i] = tempValue / envelopeFactor;
  } 

  envelopeCounter++;                                              //Increase Counter

  if(envelopeCounter == envelopeFactor)
  {
    envelopeCounter = 0;                                          //Reset the counter
  }
  fresh = 1;                                                      //Reset Tries in getPackage
}

//resets the search for the first 5 bytes [getPackag()]
void resetPackage(void) 
{
  if(res == -2)                                                   //if getpackage returns -2 goto second try
  {
    fresh = 0;
  }
  else                                                            //if getpackage returns 0 or -1 start over
  {
    fresh = 1;  
  }
}

//*******************************
//MOTION FUNCTION

//Switches through the Joints
void SwitchID()
{
  //Checks in which state the sensorbox is and whether it got moved in positive direction along the X-Axis
  if(changeID == 0 && resSignal[0]>650 && resSignal[0]<750 && resSignal[1]>550 && resSignal[1]<650 && resSignal[2]>400 && resSignal[2]<500)
  {
    id++;                                                   //Changes one Joint Up
    changeID = 1;       
    if(id == 5)                                             //If the ID is equal to Joint number 5 set it to Joint 1
    {
      id = 1;         
    }
    DisableTorqueMode(3);
    delay(2);
    DisableTorqueMode(2);
    delay(2);
    DisableTorqueMode(1);
    delay(2);
    TorqueOn(2);
    delay(2);
    TorqueOn(1);
    delay(2);  
    TorqueOn(3);
    delay(2);
  }

  //Checks in which state the sensorbox is and whether it got moved in negative direction along the X-Axis
  if(changeID==0 && resSignal[0]>200 && resSignal[0]<250 && resSignal[1]>450 && resSignal[1]<550 && resSignal[2]>400 && resSignal[2]<500)
  {
    id--;                                                   //Change one Joint down
    changeID=1;
    if(id==0)                                               //If the ID is equal to Joint number 5 set it to Joint 1
    {
      id=4;         
    }
    DisableTorqueMode(3);
    delay(2);
    DisableTorqueMode(2);
    delay(2);
    DisableTorqueMode(1);
    delay(2);
    TorqueOn(2);
    delay(2);
    TorqueOn(1);
    delay(2);  
    TorqueOn(3);
    delay(2);
  }

  //Checks in which state the sensorbox is and whether it got moved back to the start orientation.   
  if(changeID==1  && resSignal[0]>450 && resSignal[0]<550  && resSignal[1]>700  && resSignal[1]<800 && resSignal[2]>400 && resSignal[2]<500)
  {
    changeID=0;
  }
}

//Movement of the secelcted Joint for differnet inputs from the muscles
void moveJoints()
{
  if(resSignal[4]<(highLimCCW-10) && resSignal[3]<(highLimCW-10))
  {
    if(TorqueModeState == 1)
    {
      DisableTorqueMode(id);
      delay(2);
    }
    TorqueOn(id);
    delay(2);
  }

  else if(TorqueModeState == 0)
  {
    EnableTorqueMode(id);
    delay(2);
  }

  if(resSignal[3]<lowLimCCW && resSignal[4]>highLimCCW && TorqueModeState==1)
  {
    RotateJoint1((((speedMax - speedMin)/(1023 - highLimCCW))*(resSignal[4] - highLimCCW)) + speedMin);                        //setting up the wanted speed
    delay(2);
  }

  if(resSignal[4]<lowLimCW && resSignal[3]>highLimCW && TorqueModeState==1)  
  {  
    RotateJoint1(-1*((((speedMax - speedMin)/(1023 - highLimCW))*(resSignal[3] - highLimCCW)) + speedMin));                    //setting up the wanted speed
    delay(2);  
  }
}

//Movement of the gripper for differnet inputs from the muscles
void moveGripper()
{
  if(resSignal[3]<lowLimCCW && resSignal[4]>highLimCCW) 
  {
    closeGripper((((speedMaxGripper - speedMinGripper)/(1023 - highLimCCW))*(resSignal[4] - highLimCCW)) + speedMinGripper);    //setting up the wanted speed
    delay(2);
  }
  if(resSignal[4]<lowLimCW && resSignal[3]>highLimCW)
  {
    openGripper((((speedMaxGripper - speedMinGripper)/(1023 - highLimCW))*(resSignal[3] - highLimCW)) + speedMinGripper);       //setting up the wanted speed
    delay(2);
  }
}

//Makes the joints hold their position until they get told something different.
void stiffJoints()        
{
  DisableTorqueMode(1);             
  delay(2);
  TorqueOn(1);
  delay(2);
  DisableTorqueMode(2);
  delay(2);
  TorqueOn(2);
  delay(2);
  DisableTorqueMode(3);
  delay(2);
  TorqueOn(3);
}


//Functions which are used to do the motion in the loop fuctions

//Open the gripper syncronized
void openGripper(float wantedSpeed)                                         
{ 
  pos5current = GetPosition(5);                                //Reading and storing the current position of the 5th joint  
  if(pos5current > pos5open)
  {
    pos5current-=100;                                          //Setting opening limit
  }
  //Mirroring the 4 joint
  pos4current = pos4max - (pos5current - pos5min);         
  Speed = wantedSpeed;                                         //Setting the opening speed
  SetSpeed(4,Speed);
  delay(2);
  SetSpeed(5,Speed);
  delay(2);
  SetPosition(5,pos5current);
  delay(2);
  SetPosition(4,pos4current);
  delay(2);
}

//Close the gripper syncronized
void closeGripper(float wantedSpeed)
{
  Speed = wantedSpeed;                                      //Setting the closing speed
  pos5current = GetPosition(5);                             //Reading and storing the current position of the 5th joint
  if(pos5current < pos5close)
  {
    pos5current+=100;                                       //Setting closing limit
  }
  //Mirrowing the 4 joint
  pos4current = pos4max - (pos5current - pos5min);
  SetSpeed(4,Speed);
  delay(2);
  SetSpeed(5,Speed);
  delay(2);
  SetPosition(5,pos5current);
  delay(2);
  SetPosition(4,pos4current);
  delay(2);
}

//Controller 
void RotateJoint1(float wantedSpeed)
{
  for(int i = 0; i < 1; i++)
  {
    currentSpeed = GetSpeed(id);                      //Stores the current speed in a variable
    delay(2);
    error = wantedSpeed - currentSpeed;               //Calculating the difference between the wanted speed and the current speed
    torque = error *kp;                               //Calculating the torque needed to reach the wanted speed
    SetTorque(id,torque);                             //Setting the needed torque
    delay(2);
  }
}

//***********************
//MAIN PROGRAM

void setup() 
{
  Serial.begin(115200);                                     //Opens serial port, sets data rate to 115200 bps.
  ax12Init(1000000);                                        //Opens communication to the robot arm.
  stiffJoints();                                            //Calls stiffJoints function
  initEnvelop();                                            //Initializes the envelope 2 dimantional array and puts every entry to 0.
}

void loop() 
{
  checkTorqueMode();                                         //Gets the torque mode from the joint which is currently selected to move.
  res = getPackage(fresh, package);                          //Trys to get a full package and returns res = 0 when the package is complete and the checksum is okay.
  //***************************************************
  //if a full package is returned by the getPackage function.
  if(res == 0)
  {
    assemblePackage();                                       //puts the low and high bytes from the sensor data together and puts them into an array.
    envelopeSensorData();                                    //Puts the senros data array into a two dimentional array and calculates an average for each sensor input based on the last (envelopefactor) values.
    //************************************************
    //Motions based on the EMG an ACC signal
    SwitchID();                                            //Calls the SwitchID function
    if(id < 4)                                             //Movements for the joints with id 1, 2 and 3
    {
      moveJoints();                                        //Calls the moveJoints function
    }
    if(id == 4)                                            //Movements for the gripper  
    {
      moveGripper();                                       //Calls the moveGripper function
    }
  } 
  //************************************************
  // if the package is not done yet and the getPackage function returnes not 0 this resetPackage function says what the getpackage function has to do
  // it can return fresh=0 or fresh=1 where 0 means continue looking for the package and 1 means there is a new start of a package countinue with that one.
  // this ensures that no package is lost.
  else
  {
    resetPackage();
  }
}
























