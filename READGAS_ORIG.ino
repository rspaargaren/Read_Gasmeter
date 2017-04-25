/*
 * 
 * 
 * 
 * 
 * Currently the autoDetectMaxMin in set to true which will find the TOP and BOTTOM of the wave, however if you want 
 * to use it the gas must be flowing.
 */



#define MY_DEBUG
#define MY_RADIO_NRF24

#include <MySensors.h>                  
#include <Wire.h>                       //I2C Arduino Library

#define CHILD_ID 1                      //ID of the sensor child
#define SLEEP_MODE false                //prevent sensor from sleeping
#define address 0x1E                    //0011110b, I2C 7bit address of HMC5883

int TOP = 0;                            //highest magnetic field registered from meter (Ga)Initialize low if using AutoDetectMaxMin
int BOTTOM = 0;                         //lowest magnetic field registered from meter (Ga) Initialize high if using AutoDetectMaxMin
int NewTop=-9000;
int NewBottom=9000;
int tol = 50;
unsigned long SEND_FREQUENCY = 30000;   // Minimum time between send (in milliseconds). We don't want to spam the gateway.

bool metric = true;                     //sets units to Metric or English
bool autoDetectMaxMin = false;           //lets Arduino decide the values for TOP and BOTTOM
bool pcReceived = false;                //whether or not the gw has sent us a pulse count
bool rising = true;                     //whether or not a pulse has been triggered
bool inside = true;                     //whether the magnetic field is within TOP and BOTTOM limits
unsigned long pulsecount = 0;           //total number of pulses measured ever
unsigned long oldPulseCount = 0;        //old total
double vpp = 0.12;                      //Volume of gas per pulse
unsigned long lastSend = 0;             //time since last transmission - msec
double volume = 0;                      //Cumulative amount of gas measured
const int len = 3;                      //number of flow rate measurements to save
double flow [len];                      //array of previous gas flow rate measurements
double avgFlow = 0;                     //average of all elements in flow array
double oldAvgFlow = 0;                  //previous average flow
int divider = 1;                        //Current divider
int totDividers = 10;                    //Number of dividers
int increment = (TOP - BOTTOM) / totDividers;   //space b/w dividers
int newTop = -9000;                     //potential new Top
int newBottom = 9000;                   //potential new Bottom
int counter=0;

MyMessage flowMsg(CHILD_ID,V_FLOW);
MyMessage volumeMsg(CHILD_ID,V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID,V_VAR1);
MyMessage lastTopMsg(CHILD_ID,V_VAR2);
MyMessage lastBottomMsg(CHILD_ID,V_VAR3);

void setup(){
  //Initialize Serial and I2C communications
  Serial.begin(115200);
  Wire.begin();

  // Fetch last known pulse count , TOP and BOTTOM value from gw
  request(CHILD_ID, V_VAR1);
  request(CHILD_ID, V_VAR2);
  request(CHILD_ID, V_VAR3);
  
  // Wait until timeout of 2 seconds for message from gw
  wait(2000, 2, V_VAR1);
  wait(2000, 2, V_VAR2);
  wait(2000, 2, V_VAR3);

  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  
  int y = 0;
  int oldy = 0;

  //WARNING: MAKE SURE GAS IS RUNNING IF USING THIS OPTION!!!
  if(TOP==0 && BOTTOM==0){
  autoDetectMaxMin = true;
    //determine max and min magnetic field strength over a few minutes
    lastSend = millis();
    
    while(millis() - lastSend < 120000){
      y = readMag();
      if(y > TOP){
        TOP = y;                        //update TOP if new max has been detected
      }
      else if(y < BOTTOM){
        BOTTOM = y;                     //update BOTTOM if new min has been detected
      }
    }
    
    TOP -= tol;                         //nudge TOP and BOTTOM so that they have a chance of being triggered
    BOTTOM += tol;

    increment = (TOP - BOTTOM) / totDividers;    //recalculate increment to match new TOP and BOTTOM
    autoDetectMaxMin = false;           //finished determining TOP and BOTTOM
   
  Serial.println("Store on Controller TOP and BOTTOM found");
  send(lastTopMsg.set(TOP));
  send(lastBottomMsg.set(BOTTOM));
  
  }
  increment = (TOP - BOTTOM) / totDividers;    //recalculate increment to match new TOP and BOTTOM
  Serial.print("Increment = ");
  Serial.println(increment);
  
  oldy = readMag();
  y = readMag();
  while(abs(y - oldy) < increment / 2){ //wait until difference b/w y and oldy is greater than half an increment
    y = readMag();
  }
  rising = (y > oldy);
  Serial.println(rising ? "Magnetic field is rising" : "Magnetic field is falling");
}

void presentation()
{
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo("Gas Meter", "0.4");

    // Register this device as Gas sensor
    present(CHILD_ID, S_GAS);
}

void loop(){
  if (!pcReceived) {
    //Last Pulsecount not yet received from controller, request it again
    request(CHILD_ID, V_VAR1);
    return;
  }
  //detecting magnetic pulses - Fractional Simple Method
  while(millis() - lastSend < SEND_FREQUENCY){
    int y = readMag();
  
  if(y >NewTop ){
        NewTop = y;                        //update TOP if new max has been detected
      }
      else if(y < NewBottom){
        NewBottom = y;                     //update BOTTOM if new min has been detected
      }

    if(inside && rising && y > BOTTOM + divider * increment && divider < totDividers+1){
      divider++;
      pulsecount++;
    }
    else if(inside && !rising && y < TOP - divider * increment &&  divider < totDividers+1){
      divider++;
      pulsecount++;
    }

    if(inside && (y > TOP || y < BOTTOM )){        //switch directions once TOP or BOTTOM divider has been reached
      inside = false;                 //keep this from happening multiple times once signal exceeds TOP or BOTTOM
      Serial.println("OUTSIDE");
    }
    else if(!inside && (y < TOP - increment / 2 && y > BOTTOM + increment / 2)){
      rising = !rising;
      divider = 1;
      inside = true;
      Serial.println("INSIDE");
    } 
  }
  
counter += (pulsecount - oldPulseCount);      //update counter
    if(counter >= ((totDividers + 1) * 2)){
    if ( (abs(TOP-NewTop)) > tol || (abs(BOTTOM-NewBottom)) >tol){
      TOP=NewTop-tol;
      BOTTOM=NewBottom+tol;
      increment = (TOP - BOTTOM) / totDividers;    //recalculate increment to match new TOP and BOTTOM

      //Send Top and Bottom to gateway
      send(lastTopMsg.set(TOP));                  
      send(lastBottomMsg.set(BOTTOM));
      
      //reset newTop and newBottom
      newTop = -9000;
      newBottom = 9000;
      counter = 0;
      //display new bounds
      Serial.println("NEW BOUNDARIES SET:");
      Serial.print("Top = ");
      Serial.println(TOP);
      Serial.print("Bottom = ");
      Serial.println(BOTTOM);
      Serial.print("Increment = ");
      Serial.println(increment);
    }
  }
  

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

  //send updated cumulative pulse count and volume data, if necessary
  if(pulsecount != oldPulseCount){
    oldPulseCount = pulsecount;              //update old total
    
    //calculate volume
    volume = (double)oldPulseCount * (double)vpp / 1000.0;

    //send pulse count and volume data to gw
    send(lastCounterMsg.set(pulsecount));
    send(volumeMsg.set(volume, 3));
  }

  lastSend = millis();
  
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
  }
  if (message.type==V_VAR2) {
    int gwStoredTOP=message.getInt();
    TOP = gwStoredTOP;
    Serial.print("Received stored TOP value from gw:");
    Serial.println(TOP);
  }
  if (message.type==V_VAR3) {
    int gwStoredBOTTOM=message.getInt();
    BOTTOM = gwStoredBOTTOM;
    Serial.print("Received stored BOTTOM value from gw:");
    Serial.println(BOTTOM);
  }
}
int readMag(){
  int x = 0, y = 0, z = 0;
  
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

  if(!autoDetectMaxMin){
    //show real-time magnetic field, pulse count, and pulse count total
    Serial.print("y: ");
    Serial.print(y);
    Serial.print(rising ? "  Rising, " : "  Falling, ");
    Serial.print("next pulse at: ");
    Serial.print(rising ? BOTTOM + divider * increment : TOP - divider * increment);
    Serial.print("  Current Number of Pulses: ");
    Serial.print(pulsecount - oldPulseCount);
    Serial.print("  Last Total Pulse Count Sent to GW: ");
    Serial.println(oldPulseCount);
  }
  else{
    //show real-time magnetic field, TOP, BOTTOM, and time left in auto-detect mode
    Serial.print("y: ");
    Serial.print(y);
    Serial.print("  TOP: ");
    Serial.print(TOP);
    Serial.print("  BOTTOM: ");
    Serial.print(BOTTOM);
    unsigned long remainingTime = 120000 + lastSend - millis();
    Serial.print("  Time remaining: ");
    Serial.print(remainingTime / 60000);
    Serial.print(":");
    remainingTime = (remainingTime % 60000) / 1000;
    if(remainingTime >= 10){
      Serial.println(remainingTime);
    }
    else{
      Serial.print("0");
      Serial.println(remainingTime);
    }
    
  }
  
  return y;
 
}


