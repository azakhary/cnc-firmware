/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotorX = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *myMotorY = AFMS.getStepper(200, 2);

Servo servo;

int lineIndex = 0;
#define LINE_BUFFER_LENGTH 256
char line[ LINE_BUFFER_LENGTH ];
int c = 0; // byte received on the serial port
uint8_t mode = INTERLEAVE;
int speedDelay = 0;


void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps
    Serial.println("START");
    Serial.println("IDLE");
    
    AFMS.begin();  // create with the default frequency 1.6KHz
    //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

    myMotorX->release();
    myMotorY->release();

    servo.attach(9); // 9 or 10
    penUp();
}

void loop() {
  processSerialInput();
  //executeStep();
}

void processSerialInput() {
  if (Serial.available() > 0) {
    c = Serial.read(); 
    if(( c == '\n') || (c == '\r')) { // command receiving complete
      if ( lineIndex > 0 ) {                        // Line is complete. Then execute!       
          line[ lineIndex ] = '\0';                   // Terminate string
          processIncomingLine( line, lineIndex );
          lineIndex = 0;
        } 
        else { 
          // Empty or comment line. Skip block.
        }
    } else {
      //accumulating byte data in line
      if ( lineIndex >= LINE_BUFFER_LENGTH-1 ) {
        Serial.println("ERROR:1");
        lineIndex = 0;
      } 
      else if ( c >= 'a' && c <= 'z' ) {        // Upcase lowercase (making this uppercase)
        line[ lineIndex++ ] = c-'a'+'A';
      } 
      else {
        line[ lineIndex++ ] = c;
      }
    }
  }
}

void processIncomingLine( char* line, int charNB ) {
  int currentIndex = 0;
  char buffer[ 64 ];
  float params[10];
  int paramIndex = 0;
  int bufferIndex = 0;
  char command = '\0';

  while( currentIndex < charNB ) {
    char c = line[currentIndex++];
    if(currentIndex == 1) {
      command = c;
    } else {
      if(c == ':') { // storing prev param, preparing for next param
        if(bufferIndex > 0) {
          // store buffer
          buffer[bufferIndex] = '\0';
          params[paramIndex++] = atof(buffer);
        }
        bufferIndex = 0;
      } else { //reading current param
        buffer[bufferIndex++] = c;
      }
    }
  }
  if(bufferIndex > 0) {
    buffer[bufferIndex] = '\0';
    params[paramIndex++] = atof(buffer);
  }

  processCommand(command, params);
}

void processCommand( char command, float* params ) {
   switch(command) {
    case 'M':
      {
        float xSteps = params[0];
        float ySteps = params[1];
        Serial.println("BUSY");  
        lineBy(xSteps, ySteps);
        lineDebug(xSteps, ySteps);
        Serial.println("IDLE");
      }
      break;
    case 'A':
      {
        float centerX = params[0];
        float centerY = params[1];
        float endX = params[2];
        float endY = params[3];
        boolean dirCW = true;
        if(params[4] == 0) dirCW = false;        
        Serial.println("BUSY");  
        arc(centerX, centerY, endX, endY, dirCW);
        Serial.println("IDLE");
      }
      break;      
    case 'U':
      // servo up      
      Serial.println("BUSY");  
      penUp();
      delay(200);
      Serial.println("IDLE");
      //Serial.println("P");
      break;
    case 'D':
      // servo down
      Serial.println("BUSY");  
      penDown();
      Serial.println("IDLE");
      //Serial.println("P");
      break;
    case 'R':
      myMotorX->release();
      myMotorY->release();
      delay(200);
      break;
    default:
      Serial.println("ERROR:2");
      Serial.println("IDLE");
   }   
}

void penUp() {
  servo.write(90);
  delay(200);
  speedDelay = 0;
  mode = DOUBLE;
}

void penDown() {
  servo.write(76);
  delay(200);
  speedDelay = 10;
  mode = DOUBLE;
}

void lineDebug(float x, float y) {
  /*
  Serial.print("D:");
  Serial.print(x);
  Serial.print(":");
  Serial.println(y);*/
}

void lineByMicro(float xSteps, float ySteps) {   
  lineAlgorythm(xSteps*16.0, ySteps*16.0, MICROSTEP);
}

void lineByDouble(int xSteps, float ySteps) {
  lineAlgorythm(xSteps, ySteps, DOUBLE);
}

void lineAlgorythm(int xSteps, int ySteps, uint8_t mode_override) {  
  if(xSteps == 0 && ySteps == 0) return;
  
  uint8_t dirX = BACKWARD;
  uint8_t dirY = FORWARD;

  int dirMulX = 1;
  int dirMulY = 1;  

  if(xSteps < 0) {
    xSteps = -xSteps;
    dirX = FORWARD;
    dirMulX = -1;
  }
  if(ySteps < 0) {
    ySteps = -ySteps;
    dirY = BACKWARD;
    dirMulY = -1;
  }


  byte stepCommand = 0;
  
  if(xSteps > ySteps) {
    float minStep = (float)ySteps/(float)xSteps; 
    float dr = 0;
    for(int i = 0; i < xSteps; i++) { 
      dr+=minStep;      
      if(dr >= 1) {
        myMotorY->onestep(dirY, mode_override);
        dr = dr - 1.0;
      } else {
        
      }
      myMotorX->onestep(dirX, mode_override);
      if(mode == DOUBLE) {
        delay(speedDelay);
      }
    }
  } else {
    float minStep = (float)xSteps/(float)ySteps; 
    float dr = 0;
    for(int i = 0; i < ySteps; i++) { 
      dr+=minStep;      
      if(dr >= 1) {
        myMotorX->onestep(dirX*FORWARD, mode_override);
        dr = dr - 1.0;
      } else {
        
      }
      myMotorY->onestep(dirY, mode_override);
      if(mode == DOUBLE) {
        delay(speedDelay);
      }
    }
  }
}

void lineByInterleave(float xSteps, float ySteps) { 
  lineAlgorythm(xSteps * 2.0, ySteps * 2.0, INTERLEAVE);
}

void lineBy(float xSteps, float ySteps) { 
  int intX = (int)xSteps;
  int intY = (int)ySteps;
  
  float floatX = xSteps-intX;
  float floatY = ySteps-intY;
  
  if(intX != 0 || intY != 0) {
    lineByDouble(intX, intY);
  }

  if(floatX != 0 || floatY != 0) {
    lineByMicro(floatX, floatY);
  }
}


/**
 * centerX and centerY are relative to current position
 * endX and endY are relative to current position
 */
void arc(float centerX, float centerY, float endX, float endY, boolean CW) {
  float CM_PER_SEGMENT = 5.0;

  int oldDelay = speedDelay;

  // get radius
  float radius=sqrt(centerX*centerX+centerY*centerY);

  // find the sweep of the arc
  float angle1 = atan3(-centerX, -centerY);
  float angle2 = atan3(endX-centerX, endY-centerY);
  
  float sweep = angle2-angle1;

  if(!CW && sweep < 0) sweep = 2*PI + sweep;
  if(CW && sweep > 0) sweep = sweep - 2*PI;
  
  speedDelay = 0;
  
  float len = abs(sweep) * radius;
  int i, num_segments = floor( len / CM_PER_SEGMENT );

  float nx, ny, nz, angle3, fraction;
  nx = 0;
  ny = 0;
  float currX, currY = 0;

  for(i=0;i<num_segments;++i) {
    // interpolate around the arc
    fraction = ((float)i)/((float)num_segments);
    angle3 = ( sweep * fraction ) + angle1;

    // find the intermediate position
    nx = (centerX + cos(angle3) * radius) - currX;
    ny = (centerY + sin(angle3) * radius) - currY;
    // make a line to that intermediate position
    currX=(centerX + cos(angle3) * radius);

    currY=(centerY + sin(angle3) * radius);

    lineByMicro(nx,ny);
    lineDebug(nx, ny);
  }

  // one last line hit the end
  lineByMicro(endX-currX,endY-currY);
  lineDebug(endX-currX,endY-currY);

  speedDelay = oldDelay;
}

// returns angle of dy/dx as a value from 0...2PI
float atan3(float dx,float dy) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}

