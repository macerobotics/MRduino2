/**
  ******************************************************************************
  * @file    mrduino2.cpp
  * @author  Mace Robotics (www.macerobotics.com)
  * @Licence MIT Licence
  * @version 0.4
  * @date    05/02/2019
  * @brief   lib for MRduino2 robot
  *
 *******************************************************************************/


#include <Arduino.h>
#include <math.h>
#include <assert.h>
#include "mrduino2.h"

static void forwardControl(int speed, int distance);
static void backControl(int speed, int distance);
static void turnRightControl(int speed, int angle);
static void turnLeftControl(int speed, int angle);
static boolean  state_control = false;
static void controlEnable();
static void controlDisable();
static void forwardC(int speed, int distance);
static void backC(int speed, int distance);
static void turnRightC(int speed, int distance);
static void turnLeftC(int speed, int distance);
static float readFloatData();
static int readData();

static bool s_ledRight = false;
static bool s_ledLeft = false;

void initRobot()
{
  Serial.begin(115200);

  delay(0.5);
}


/**********************************************************
 * @brief  firmwareVersion
 * @param  None
 * @retval None
**********************************************************/
float firmwareVersion()
{
String  commande;
 
  commande = "#FV!";
  Serial.println(commande); 
  
  return(readFloatData());

}


/**********************************************************
 * @brief  battery
 * @param  None
 * @retval None
**********************************************************/
float battery()
{
String  commande;
 
  commande = "#BAT!";
  Serial.println(commande); 
  
  return(readFloatData());
}



/**********************************************************
 * @brief  readSwitch
 * @param  None
 * @retval None
**********************************************************/
int readSwitch()
{
String  commande;
 
  commande = "#SW!";
  Serial.println(commande); 
  
  return(readData());

}



/**********************************************************
 * @brief  motorRight
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
void motorRight(int speed, int direction)
{
String  commande;

  controlDisable();
  commande = "#MOTR," + String(direction) + "," + String(speed) + "!";
  Serial.println(commande); 
}


/**********************************************************
 * @brief  motorRight
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
void motorLeft(int speed, int direction)
{
String  commande;

  controlDisable();
  commande = "#MOTL," + String(direction) + "," + String(speed) + "!";
  Serial.println(commande); 
}


/**********************************************************
 * @brief  motorsDisable
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
void motorsDisable(void)
{
String  commande;

  commande = "#MDI!";
  Serial.println(commande);
}


/**********************************************************
 * @brief  motorsEnable
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
void motorsEnable(void)
{
String  commande;

  commande = "#MEN!";
  Serial.println(commande);
}


/**********************************************************
 * @brief  motorsEnable
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
void motorsBrake(void)
{
String  commande;

    
  commande = "#BRK!";
  Serial.println(commande);
}


/**********************************************************
 * @brief  pinMode
 * @param  
 * @retval None
**********************************************************/
void MRpinMode(int pin, int mode)
{
String  commande;

  commande = "#PIN," + String(pin) + "," + String(mode) + "!";
  Serial.println(commande);
}

void MRpinWrite(int pin, int state)
{

}


/**********************************************************
 * @brief  resetUc
 * @param  
 * @retval None
**********************************************************/
void resetUc()
{
String  commande;

  commande = "#RST!";
  Serial.println(commande);
  delay(1000);
}


/**********************************************************
 * @brief  forward
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
void forward(int speed)
{
String  commande;

  controlDisable();
  
  commande = "#MF," + String(speed) + "!";
  Serial.println(commande); 
  
 }


/**********************************************************
 * @brief  forward_mm (forward with control, pulling function)
 * @param  speed ( 0 to 100 ), distance millimeter
 * @retval None
**********************************************************/
void forward_mm(int speed, int distance)
{
  controlEnable();
  forwardC(speed, distance*4);
  controlDisable();
}


/**********************************************************
 * @brief  forwardmm (forward with control, no pulling function
 * @param  speed ( 0 to 100 ), distance millimeter
 * @retval None
**********************************************************/
void forwardmm(int speed, int distance)
{
String  commande;
int  state = 0;
static bool fmm = false;
 

    controlEnable();
    commande = "#MFC," + String(distance*4) + "," + String(speed) + "!";
    Serial.println(commande);



  
}

/**********************************************************
 * @brief  back_mm (forward with control) - pulling function
 * @param  speed ( 0 to 100 ), distance millimeter
 * @retval None
**********************************************************/
void back_mm(int speed, int distance)
{
  controlEnable();
  backC(speed, distance*4);
  controlDisable();
}


/**********************************************************
 * @brief  backmm (back with control, no pulling function
 * @param  speed ( 0 to 100 ), distance millimeter
 * @retval None
**********************************************************/
void backmm(int speed, int distance)
{
String  commande;
int  state = 0;
 
  controlEnable();
  commande = "#MBC," + String(distance*4) + "," + String(speed) + "!";
  Serial.println(commande); 

  
}


/**********************************************************
 * @brief  back
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
void back(int speed)
{
String  commande;

  controlDisable();
  commande = "#MB," + String(speed) + "!";
  Serial.println(commande); 
  
}

/**********************************************************
 * @brief  turnLeft
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
void turnLeft(int speed)
{  
String  commande;

  controlDisable();
  commande = "#TL," + String(speed) + "!";
  Serial.println(commande); 
 }


/**********************************************************
 * @brief  turnRight_degree
 * @param  speed ( 0 to 100 ), angle (0 to 360)
 * @retval None
**********************************************************/
void turnRight_degree(int speed, unsigned int angle)
{
float angle_degree = 0;
int angle_turn;

  controlEnable();

  if((angle >= 0)&&(angle <= 360))
  {
    angle_degree = (float)(angle*546.0);
    angle_degree = (float)(angle_degree/90.0);
    angle_turn = (int)(angle_degree);
 
    turnRightC(speed, angle_turn); 
  }
  else
  {
    // error angle value
  }
  
  controlDisable();
}


/**********************************************************
 * @brief  turnLeft_degree
 * @param  speed ( 0 to 100 ), angle (0 to 360)
 * @retval None
**********************************************************/
void turnLeft_degree(int speed, unsigned int angle)
{
float angle_degree = 0;
int angle_turn;
  
  controlEnable();
  
  if((angle >= 0)&&(angle <= 360))
  {
    angle_degree = (float)(angle*546.0);
    angle_degree = (float)(angle_degree/90.0);
    angle_turn = (int)(angle_degree);
 
    turnLeftC(speed, angle_turn);
  }
  else
  {
    // error angle value
  }
  
  controlDisable();
}


/**********************************************************
 * @brief  turnRight
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
void turnRight(int speed)
{
String  commande;
 
  controlDisable();
  commande = "#TR," + String(speed) + "!";
  Serial.println(commande); 

}


/**********************************************************
 * @brief  proxSensor
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
int proxSensor()
{
String  commande;
int data;
 
  commande = "#PROX,3!";
  Serial.println(commande); 

  data = readData();
  
  // pour resoudre bug au demarrage du capteur, il envoie des valeurs de 0 ou -1.
  if(data < 5)
  {
    data  = 255;
  }
  
  delay(100);
  
  return data;
}


/**********************************************************
 * @brief  stop
 * @param  
 * @retval None
**********************************************************/
void stop()
{
  
  if(state_control == true)
  {
    controlDisable();  
  }
  Serial.println("#STP!"); 
}


/**********************************************************
 * @brief read encoder left
 * @param  
 * @retval None
**********************************************************/
int encoderLeft()
{
String  commande;
int data;
 
  commande = "#EDL!";
  Serial.println(commande); 

  data = readData();
  
  return data;
}


/**********************************************************
 * @brief read encoder right
 * @param  
 * @retval None
**********************************************************/
int encoderRight()
{
String  commande;
int data;
 
  commande = "#EDR!";
  Serial.println(commande); 

  data = readData();
  
  return data;
}










/**********************************************************
 * @brief  turn_degree
 *                   90°
 *                 |
 * @param  180°<--- ----> 0° (0 to 360°)
 * @retval None
**********************************************************/
void turn_degree(int speed, unsigned int angle)
{
float angle_Robot;
float angle_Consigne;

  // read actual robot position
  angle_Robot = robotPositionOrientation();
  
  // conversion en degree
  angle_Robot = (angle_Robot*180.0)/(3.14);
  
  angle_Consigne = angle - angle_Robot;
  
  if(angle >= angle_Robot)
  {
	   turnLeft_degree(10,abs(angle_Consigne)); 
  }
  else
  {
	   turnRight_degree(10,abs(angle_Consigne));  
  }
  

}


/**********************************************************
 * @brief  robotPositionX
 * @param  
 * @retval None
**********************************************************/
float robotPositionX()
{
String  commande;
 
  commande = "#POX!";
  Serial.println(commande); 
  
  return(readFloatData());

}

/**********************************************************
 * @brief  robotPositionY
 * @param  None
 * @retval None
**********************************************************/
float robotPositionY()
{
String  commande;
 
  commande = "#POY!";
  Serial.println(commande); 
  
  return(-readFloatData());

}


/**********************************************************
 * @brief  robotPositionOrientation
 * @param  None
 * @retval None
**********************************************************/
float robotPositionOrientation()
{
String  commande;
 
  commande = "#POO!";
  Serial.println(commande); 
  
  return(readFloatData());
}


/**********************************************************
 * @brief  robotGo
 * @param coordonner X and coordonner Y
 * @retval None
**********************************************************/
/*void robotGo(int speed, int coord_X, int coord_Y)
{
int distance;
float temp;
float angle, angle_Robot;
float coord_X_Robot, coord_Y_Robot;
float coord_X_Goal, coord_Y_Goal;

  controlEnable();


  // read actual robot position
  coord_X_Robot = robotPositionX();
  coord_Y_Robot = robotPositionY();
  
  // read actual robot position
  angle_Robot = robotPositionOrientation();
  
  // conversion en degree
  angle_Robot = (angle_Robot*180.0)/(3.14);
  
  // calcul goal coordonner
  coord_X_Goal = (float)(coord_X - coord_X_Robot);
  coord_Y_Goal = (float)(coord_Y - coord_Y_Robot);

  if(coord_Y_Goal > 0)
  {

    distance = sqrt(coord_X_Goal*coord_X_Goal + coord_Y_Goal*coord_Y_Goal);
	temp = (float)((float)coord_X_Goal/(float)distance);
	
	if(temp > 1.0)
	{
	  temp = 1.0;
	}
	   
	if(temp < -1.0)
	{
	  temp = -1.0;
	}
	   
    angle = acos(temp);
  }
  else
  {
    if(coord_X_Goal > 0)
    {

      distance = sqrt(coord_X_Goal*coord_X_Goal + coord_Y_Goal*coord_Y_Goal);
	  temp = (float)((float)coord_Y_Goal/(float)distance);
	  
	  	if(temp > 1.0)
	   {
	     temp = 1.0;
	   }
	   
	   if(temp < -1.0)
	   {
	     temp = -1.0;
	   }
	   
	   
      angle = asin(temp);
    }
    else
    {

      distance = sqrt(coord_X_Goal*coord_X_Goal + coord_Y_Goal*coord_Y_Goal);
	  temp = (float)((float)coord_X_Goal/(float)distance);
	  
	   if(temp > 1.0)
	   {
	     temp = 1.0;
	   }
	   
	   if(temp < -1.0)
	   {
	     temp = -1.0;
	   }
	      
	  temp = asin(temp);

      angle = -(1.5707 - temp);
    }

  }

  // conversion en degree
  angle = (angle*180.0)/(3.14);

  if(angle < 0)
  {
	  angle = angle + 360;
  }

  turn_degree(speed,angle); 
	
  forward_mm(speed, distance);
  
  //controlDisable();
  
}*/


/**********************************************************
 * @brief  ledLeft
 * @param  
 * @retval None
**********************************************************/
void ledLeft(bool on_off)
{
String  commande;

  if(on_off > 0)
  {
    commande = "#LEL,1!";
    Serial.println(commande); 
	s_ledLeft = true;
  }
  else
  {
    commande = "#LEL,0!";
    Serial.println(commande); 
	s_ledLeft = false;
  }
}


/**********************************************************
 * @brief  ledRight
 * @param  
 * @retval None
**********************************************************/
void ledRight(bool on_off)
{
String  commande;

  if(on_off > 0)
  {
    commande = "#LER,1!";
    Serial.println(commande); 
	s_ledRight = true;
  }
  else
  {
    commande = "#LER,0!";
    Serial.println(commande); 
	s_ledRight = false;
  }
}

//*********************************************************************/
//*********************************************************************/
//*********************************************************************/
//*****************PRIVATE FUNCTIONS***********************************/
//*********************************************************************/
//*********************************************************************/
//*********************************************************************/


/**********************************************************
 * @brief  turnRightControl
 * @param  
 * @retval None
**********************************************************/
static void turnLeftControl(int speed, int angle)
{
String  commande;

  controlEnable();
  
  if(state_control == true)
  {
    commande = "#TLC," + String(angle) + "," + String(speed) + "!";
    Serial.println(commande); 
  }
}


/**********************************************************
 * @brief  turnRightControl
 * @param  
 * @retval None
**********************************************************/
static void turnRightControl(int speed, int angle)
{
String  commande;

  controlEnable();
  
  if(state_control == true)
  {
    commande = "#TRC," + String(angle) + "," + String(speed) + "!";
    Serial.println(commande); 
  }
}


/**********************************************************
 * @brief  forwardC (forward with control)
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
static void forwardC(int speed, int distance)
{
String  commande;
int  state = 0;
 
  commande = "#MFC," + String(distance) + "," + String(speed) + "!";
  Serial.println(commande); 
  
  while(state != 3)
  {
    Serial.println("#TGS,1!"); 
    Serial.println(state);
    state = readData();
    
  }
  
}


/**********************************************************
 * @brief  backControl
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
static void backControl(int speed, int distance)
{
String  commande;

  controlEnable();
  
  if(state_control == true)
  {
    commande = "#MBC," + String(distance) + "," + String(speed) + "!";
    Serial.println(commande); 
  }

}


/**********************************************************
 * @brief  forwardControl
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
static void forwardControl(int speed, int distance)
{
String  commande;

  controlEnable();
  
  if(state_control == true)
  {
    commande = "#MFC," + String(distance) + "," + String(speed) + "!";
    Serial.println(commande); 
  }

}


/**********************************************************
 * @brief  controlEnable
 * @param  
 * @retval None
**********************************************************/
static void controlEnable()
{
  if(state_control == false)
  {
	resetUc();
    Serial.println("#CRE!"); 
    state_control = true;
  }
  else
  {
    // error
  }
  
}


/**********************************************************
 * @brief  controlDisable
 * @param  
 * @retval None
**********************************************************/
static void controlDisable()
{
  if(state_control == true)
  {
    Serial.println("#CRD!"); 
    state_control = false;
	//resetUc();
	
	if(s_ledRight == true)
		ledRight(1);
	
	if(s_ledLeft == true)
		ledLeft(1);
  }
  else
  {
    // error
  }
}


/**********************************************************
 * @brief  backC (back with control)
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
static void backC(int speed, int distance)
{
String  commande;
int  state = 0;
 
  commande = "#MBC," + String(distance) + "," + String(speed) + "!";
  Serial.println(commande); 
  
  while(state != 3)
  {
    Serial.println("#TGS,1!"); 
    Serial.println(state);
    state = readData();
    
  }
  
}


/**********************************************************
 * @brief  readFloatData
 * @param  
 * @retval None
**********************************************************/
static float readFloatData()
{ 
char c=0;        
String readString;

  while (c != '\n')
  {
    if (Serial.available() >0)
    {
      c = Serial.read();  //gets one byte from serial buffer
      if ( c != '$')
        readString += c; //makes the string readString
    } 
  }    

  return readString.toFloat();
}


/**********************************************************
 * @brief  turnLeftC
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
static void turnLeftC(int speed, int distance)
{
String  commande;
int  state = 0;
 
  commande = "#TLC," + String(distance) + "," + String(speed) + "!";
  Serial.println(commande); 
  
  while(state != 3)
  {
    Serial.println("#TGS,2!"); 
    Serial.println(state);
    state = readData();
  }
}


/**********************************************************
 * @brief  turnRightC
 * @param  speed ( 0 to 100 )
 * @retval None
**********************************************************/
static void turnRightC(int speed, int distance)
{
String  commande;
int  state = 0;
 
  commande = "#TRC," + String(distance) + "," + String(speed) + "!";
  Serial.println(commande); 
  
  while(state != 3)
  {
    Serial.println("#TGS,2!"); 
    Serial.println(state);
    state = readData();
  }
  
}


/**********************************************************
 * @brief  readData
 * @param  
 * @retval None
**********************************************************/
static int readData()
{ 
char c=0;        
String readString;

  while (c != '\n')
  {
    if (Serial.available() >0)
    {
      c = Serial.read();  //gets one byte from serial buffer
      if ( c != '$')
        readString += c; //makes the string readString
    } 
  }    

  return readString.toInt();
}

// end file