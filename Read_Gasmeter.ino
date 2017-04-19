/*This version is different in several ways.
 * first is is using some memory (Adafruit I2C FRAM) memory to store the top and bottom. This is used to keep the top and bottom
 * values after a restart. I guess we could have put them on the controller but can't remember why we didn't.
 * second it will only calculate the top and bottom if the values are not in memory.
 * third it auto calculates the top and bottom values as it is running. 
 * the only settings that should be changed are vpp, SEND_FREQUENCY, and metric. However I am currently using
 * Domoticz as my controller and it doesn't seem to accept English units well so for not I'm using Metric.
 */

#define MY_DEBUG
//#define MY_DEBUG_VERBOSE
#define MY_RADIO_NRF24

#include <MySensors.h>                  
#include <Wire.h>                       //I2C communications library
//#include <Adafruit_FRAM_I2C.h>          //Adafruit FRAM memory library

#define CHILD_ID 1                      //ID of the sensor child
#define SLEEP_MODE false                //prevent sensor from sleeping
#define address 0x1E                    //0011110b, I2C 7bit address of HMC5883 magnetometer

int top = 0;                            //highest magnetic field registered from meter (Ga)Initialize low if using autoDetectMaxMin
int bottom = 0;                         //lowest magnetic field registered from meter (Ga) Initialize high if using autoDetectMaxMin
int tol = 50;
int initstep = 25;                      // Init stepsize to measure difference in Y movement
int rise_count = 0;
unsigned long SEND_FREQUENCY = 30000;   // Minimum time between send (in milliseconds). We don't want to spam the gateway.

bool metric = true;                     //sets units to Metric or English TODO: move to void setup()
bool pcReceived = false;                //whether or not the gw has sent us a pulse count
bool autoDetect = false;                //true if the program is auto detecting Top and Bottom
bool rising = true;                     //whether or not a pulse has been triggered
bool oldrising = true;                  //old status of rising and falling
bool safe = false;                      //whether or not it is safe to switch directions
unsigned long pulsecount = 0;           //total number of pulses measured ever
unsigned long oldPulseCount = 0;        //old total
//double vpp = metric ? 0.160891193181 : 0.00568124219857;//Volume of gas per pulse
double vpp = metric ? 0.00005 : 0.00005; //Volume of gas per pulse
unsigned long lastSend = 0;             //time since last transmission - msec
double volume = 0;                      //Cumulative amount of gas measured
const int len = 3;                      //number of flow rate measurements to save
double flow [len];                      //array of previous gas flow rate measurements
double avgFlow = 0;                     //average of all elements in flow array
double oldAvgFlow = 0;                  //previous average flow
int y = 0;                              //magnetic field reading
int oldy = 0;                           //previous magnetic field reading
int spikey = 0;
int ystep = 0;
int newTop = -9000;                         //potential new Top
int newBottom = 9000;                  //potential new Bottom
int puls_index = 0;
int totDividers = 10;                   //Number of dividers
int pulses [9];           //Array of puls triggers
int increment = 0;                      //space b/w dividers
int counter = 0;                        //used to count pulses over periods longer than SEND_FREQUENCY
int topAddr = 0;                        //address of TOP in FRAM
int bottomAddr = topAddr + 2;           //address of BOTTOM in FRAM

MyMessage flowMsg(CHILD_ID,V_FLOW);
MyMessage volumeMsg(CHILD_ID,V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID,V_VAR1);

//Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();

void setup(){
  //Initialize Serial, I2C, and FRAM communications
  Serial.begin(115200);
  Wire.begin();
  //fram.begin();

  // Fetch last known pulse count value from gw
  request(CHILD_ID, V_VAR1);
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  //get Top and Bottom from FRAM. addresses are hard-coded seeing as there are only 2
  //newTop = readInt(topAddr);
  //newBottom = readInt(bottomAddr);
  //updateBounds();
  
  //WARNING: IT IS PREFERABLE THAT GAS IS RUNNING ON FIRST RUNNING OF THIS PROGRAM!!!
  init_top_bottom();

  y = readMag();
  oldy = readMag();
  while(abs(y - oldy) < increment / 2){ //wait until difference b/w y and oldy is greater than half an increment
    y = readMag();
  }
  rising = (y > oldy);
  Serial.println(rising ? "Magnetic field is rising" : "Magnetic field is falling");
}

void presentation()
{
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo("Gas Meter", "0.6 (2/24/17)");

    // Register this device as Gas sensor
    present(CHILD_ID, S_GAS);
}

void loop(){
  if (!pcReceived) {
    //Last Pulsecount not yet received from controller, request it again
    request(CHILD_ID, V_VAR1);
    return;
  }
  //detecting magnetic pulses - variable boundary method
  Read_magnetic_pulse();
  Calculate_flow();
  //send updated cumulative pulse count and volume data, if necessary
  Pulse_Volume();
}

void receive(const MyMessage &message)
{
  if (message.type==V_VAR1) {
    unsigned long gwPulseCount=message.getULong();
    pulsecount = gwPulseCount;
    oldPulseCount = pulsecount;
    Serial.print("Received last pulse count from gw:");
    Serial.println(pulsecount);
    pcReceived = true;
    lastSend = millis();
    //set magnetic field starting point
    oldy = readMag();
    y = readMag();
  }
}

void updateBounds(){
    int i = 0;
    //recalculate increment to match new top and bottom
    increment = (newTop - newBottom) / totDividers;
    for i = 0 to totDividers {
      array[i] = newBottom + increment*i
    }
    
    //display new bounds
    Serial.println("NEW BOUNDARIES SET:");
    Serial.print("Top = ");
    Serial.println(top);
    Serial.print("Bottom = ");
    Serial.println(bottom);
    Serial.print("Increment = ");
    Serial.println(increment);
  }
}

void updateBounds_old(){
  if(((top + tol) != newTop) && ((bottom - tol) != newBottom)){   //check if anything has actually changed
    //lock in Top and Bottom
    top = newTop - tol;
    bottom = newBottom + tol;
    
    //recalculate increment to match new top and bottom
    increment = (top - bottom) / totDividers;
  
    //reset newTop and newBottom
    //newTop = -9000;
    //newBottom = 9000;
  
    //store updated Top and Bottom in FRAM
//    writeInt(topAddr,top);
//    writeInt(bottomAddr,bottom);

    //reset newTop and newBottom
    newTop = -9000;
    newBottom = 9000;
  
    //display new bounds
    Serial.println("NEW BOUNDARIES SET:");
    Serial.print("Top = ");
    Serial.println(top);
    Serial.print("Bottom = ");
    Serial.println(bottom);
    Serial.print("Increment = ");
    Serial.println(increment);
  }
}

void detectMaxMin(){
  if(y > newTop){
        newTop = y;                    //update newTop if new max has been detected
      }
  else if(y < newBottom){
    newBottom = y;                     //update newBottom if new min has been detected
  }
}

//void writeInt(int addr, int val){       //write an int value to memory
//  byte b = highByte(val);
//  fram.write8(addr,b);
//  b = lowByte(val);
//  fram.write8(addr + 1,b);
//}

//int readInt(int addr){                  //read an int value from memory
//  int result = 0;
//  result += (int)fram.read8(addr);
 // result = result << 8;
 // result += (int)fram.read8(addr + 1);
 // return result;
//}

int readMag(){
  int x = 0, z = 0;
  
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register - was called Wire.send but the compiler had an error and said to rename to to Wire.write
  Wire.endTransmission();

  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }


  if(!autoDetect){
    //show real-time magnetic field, pulse count, and pulse count total
    Serial.print("y: ");
    Serial.print(y);
    Serial.print(rising ? "  Rising, " : "  Falling, ");
    Serial.print("next pulse at: ");
    Serial.print(rising ? oldy + increment : oldy - increment);
    Serial.print("  Current Number of Pulses: ");
    Serial.print(pulsecount - oldPulseCount);
    Serial.print("  Last Total Pulse Count Sent to GW: ");
    Serial.println(oldPulseCount);
  }
    if(abs(y - spikey) < 800){    // filter out sudden spikes in the measurements
      spikey = y;
      return y;
    }
    else{
      return spikey;
    }
    
}

void init_top_bottom (){
  y = readMag();
  oldy = readMag();
  int i = 0;
  int changes = 0;
  
  while(abs(y - oldy) < initstep){  // Wait until difference b/w y and oldy is greater than the initstep size
    y = readMag();
  }
  oldrising = (y > oldy);           // Detect whether magnetic field is rising or falling
  oldy = y;
  
  while (changes < 4) {               // Stop the loop after 4 changes of rising and falling
    while(abs(y - oldy) < initstep){  // Wait until difference b/w y and oldy is greater than the initstep size
      y = readMag();
    }
    rising = (y > oldy);
    Serial.println(rising ? "Magnetic field is rising" : "Magnetic field is falling");
    if (rising != oldrising) {        // If there is a change in direction from rising to falling start the counter
      i += 1;
      Serial.println ("First change detected!");  
    } 
    else {                            // No change in direction reset the counter
      i = 0;                          // Reset values direction is the same
      oldrising = rising;
    }
    if (i == 2) {                       // If the change has been detected two times in a row then it must be a true change
      changes += 1;                   // Increase the number of changes
      Serial.print ("Second change detected! Total number of changes in amplitude is now: ");
      Serial.print(changes);
    }
    detectMaxMin();
    oldy = y;
    
    //display details
    Serial.print("y: ");
    Serial.print(y);
    Serial.print("  Top: ");
    Serial.print(newTop);
    Serial.print("  Bottom: ");
    Serial.print(newBottom);
    Serial.print("  Number of changes detected: ");
    Serial.print(changes);
    Serial.print(":");
  }
  oldy = y;               // exit the init fase and update the boundery_table
  oldrising = rising;
  updateBounds();
  autoDetect = false;
}

void Read_magnetic_pulse(){
    y = readMag();      // Read new value for y
    if(abs(oldy - y) > increment/4){        // See if there has been sufficient movement of Y compared to the old Y value
      rising = (y > oldy);
      oldy = y;
      Serial.println(rising ? "Magnetic field is rising" : "Magnetic field is falling");
      if (rising != oldrising) {        // If there is a change in direction from rising to falling start the counter
        rise_count += 1;  
        Serial.println ("First change detected!");
      } 
      else {                            // No change in direction reset the counter
        rise_count = 0;                 // Reset values direction is the same
        oldrising = rising;
        check_puls_step();
      }
      if (rise_count == 2) {            // If the change has been detected two times in a row then it must be a true change
        Serial.println ("Second change detected!");
        //update newTop and newBottom
        if ((!rising && abs(y-newTop) >= tol) || (rising && abs(y-newBottom) >= tol)){ 
          detectMaxMin();        
        }
        if (rising) {
          puls_index = 1;
        }
        else{
          puls_index = totDividers -2;
        }
      }
    }
}

void check_puls_step(){
  if (rising && ystep <= y) {
    if (puls_index < totDividers-1) {
      puls_index += 1;
      pulsecount ++;
    } else puls_index = totDividers -1;
  } 
  else if (!rising && y <= ystep) {
    if (puls_index > 0){
      puls_index += -1;
      pulsecount ++; 
    } else puls_index = 0;
  }
  ystep = pulses[puls_index];
}

void Read_magnetic_pulse_old(){
  while(millis() - lastSend < SEND_FREQUENCY){
    //check if the signal has significantly increased/decreased
    if(abs(oldy - y) > increment){
      pulsecount ++;
      //increment or decrement oldy by one increment based on direction
      oldy += rising ? increment : -1 * increment;     
      safe = false;             //reset safe now that oldy has updated     
    }
    //check if the signal has recently switched directions
    else if(safe){                  //first make sure y has moved a significant distance from oldy
      if((rising && y <= oldy) || (!rising && y >= oldy)){
        pulsecount ++;              //add one extra pulse
        rising = !rising;           //update direction
        safe = false;
       }
    }
    
    //take another reading
    y = readMag();
    //check if y has moved a significant distance from oldy
    if(abs(y - oldy) > tol / 2){
      safe = true;
    }
    
    //update newTop and newBottom
    detectMaxMin();                
  }
  lastSend = millis();
}


void Calculate_flow(){
  //shift all flow array elements to the right by 1, ignore last element
  for(int idx = len - 1; idx > 0; idx--){
    flow[idx] = flow[idx - 1];
  }
  //calculate newest flow reading and store it as first element in flow array
  flow[0] = (double)(pulsecount - oldPulseCount) * (double)vpp * 60000.0 / (double)SEND_FREQUENCY;
  //display flow array state
  Serial.print("Flow Array State: [");
  for(int idx = 0; idx < len - 1; idx++){
    Serial.print(flow[idx]);
    Serial.print("|");
  }
  Serial.print(flow[len - 1]);
  Serial.println("]");
  //calculate average flow
  avgFlow = 0;                                //reset avgFlow
  for(int idx = 0; idx < len; idx++){         //calculate weighted sum of all elements in flow array
    avgFlow += (flow[idx] * (len - idx));
  }
  avgFlow /= (len * (len + 1) / 2);           //divide by triangle number of elements to get linear weighted average
  Serial.print("Average flow: ");             //display average flow
  Serial.println(avgFlow);
  //send flow message if avgFlow has changed
  if(avgFlow != oldAvgFlow){
    oldAvgFlow = avgFlow;
    send(flowMsg.set(avgFlow, 2));
  }
}

void Pulse_Volume(){
  if(pulsecount != oldPulseCount){    
    //calculate volume
    volume = (double)pulsecount * (double)vpp / (metric ? 1000.0 : 1);

    //send pulse count and volume data to gw
    send(lastCounterMsg.set(pulsecount));
    send(volumeMsg.set(volume, 3));

    counter += (pulsecount - oldPulseCount);      //update counter
    if(counter >= ((totDividers + 1) * 2)){
      updateBounds();                 //update bounds if at least 1 cycle has been read
      counter = 0;                    //reset counter
    }

    oldPulseCount = pulsecount;              //update old total
  }
}

