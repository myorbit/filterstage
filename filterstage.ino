/*

filterstage - Optical bandpass controller for two TMC223 stepper drivers
  
JSON client is: https://github.com/interactive-matter/aJson

To drive one filter to a position

  {"fpos":{"type":<pass>,"pos":<x>}}

with <pass> = "short" or "long"
and <x> = [-32768:32767]
  
e.g. {"fpos":{"type":"short","pos":230}}

*/


#include <Wire.h>
#include <String.h>
#include <aJSON.h>


#define _DEBUG    // for debugging purposes; uncomment if not used

// I2C address of the filter drivers
#define DRV_ALL   0x00  // address works for ALL drivers (broadcast address)
#define DRV_SHORT 0x60  // address of short-pass filter driver
#define DRV_LONG  0x61  // address of long-pass filter driver


/**
 * A structure to represent TMC223 status information
 * Use together with the GetFullStatus1() function
 * Length: 5 bytes
 */
typedef struct tmc223status
{
  unsigned char IHold: 4;
  unsigned char IRun: 4;
  unsigned char VMin: 4;
  unsigned char VMax: 4;
  unsigned char Acc: 4;
  unsigned char Shaft: 1;
  unsigned char StepMode: 2;
  unsigned char AccShape: 1;
  unsigned char TInfo: 2;
  unsigned char TW: 1;
  unsigned char TSD: 1;
  unsigned char UV2: 1;
  unsigned char ElDef: 1;
  unsigned char SteppLoss: 1;
  unsigned char VddReset: 1;
  unsigned char CPFail: 1;
  unsigned char NA: 1;
  unsigned char OVC2: 1;
  unsigned char OVC1: 1;
  unsigned char ESW: 1;
  unsigned char Motion: 3;
} tmc223status;


/**
 * A structure to represent TMC223 motor parameter
 * Use variables of this type together with the setMotorParam()
 * and getMotorParam() functions.
 * Length: 5 bytes
 */
typedef struct tmc223param
{
  unsigned char IHold: 4;
  unsigned char IRun: 4;
  unsigned char VMin: 4;
  unsigned char VMax: 4;
  unsigned char Acc: 4;
  unsigned char Shaft: 1;
  unsigned char SecPosHi: 3;
  unsigned char SecPosLo: 8;	
  unsigned char NA1: 2;
  unsigned char StepMode: 2;
  unsigned char AccShape: 1;
  unsigned char NA2: 3;
} tmc223param;


/**
 * A structure to represent TMC223 Stall Detection Status values
 * Use together with GetFullStatus2() function
 * Length: 3 bytes
 */
typedef struct tmc223stall
{
  unsigned char NA1: 3;
  unsigned char DC100: 1;
  unsigned char NA2: 1;
  unsigned char FS2StallEn: 3;
  unsigned char PWMJEn: 1;
  unsigned char DC100StEn: 1;
  unsigned char MinSamples: 3;
  unsigned char DelStallHi: 1;
  unsigned char DelStallLo: 1;
  unsigned char AbsStall: 1;
  unsigned char AbsThr: 4;
  unsigned char DelThr: 4;
} tmc223stall;


/**
 * A structure to represent TMC223 Stall Detection Paramater
 * Use variables of this type together with the setStallParam()
 * Length: 5 bytes
 */
typedef struct tmc223stallparam
{
  unsigned char IHold: 4;
  unsigned char IRun: 4;
  unsigned char VMin: 4;
  unsigned char VMax: 4;
  unsigned char Acc: 4;
  unsigned char Shaft: 1;
  unsigned char MinSamples: 3;
  unsigned char DelThr: 4;	
  unsigned char AbsThr: 4;
  unsigned char PWMJEn: 1;
  unsigned char DC100En: 1;
  unsigned char StepMode: 2;
  unsigned char AccShape: 1;
  unsigned char FS2StallEN: 3;
} tmc223stallparam;




// Declare TMC223 status structs
tmc223status fstatShort;  // Short-pass
tmc223status fstatLong;  // Long-pass

// Declare TMC223 motor parameter structs
tmc223param fparamShort;
tmc223param fparamLong;

// Declare TMC223 stall detection parameter structs
tmc223stallparam fstallparamShort;
tmc223stallparam fstallparamLong;

// Declare TMC223 stall detection status structs
tmc223stall fstallShort;
tmc223stall fstallLong;


static volatile unsigned char rxBuffer[9];  // I2C receive buffer


// Declare TMC223 position variables (for short & long-pass filters)
int targetPosShort;
int targetPosLong;
int securePosShort;
int securePosLong;
int actualPosShort;
int actualPosLong;


// JSON Streamer
unsigned long last_print = 0;
aJsonStream serial_stream(&Serial);


/**
 * @brief Arduino setup() function for initialization
 */
void setup()
{
  byte drv_count = 0;  // Counts if drivers are ready
  
  Wire.begin();    // Join I2C bus
  delay(2000);    // Delay, so we don't flood serial line
  Serial.begin(115200);    // Set up Serial library at 9600 bps


  // Scan I2C bus for both filter shifter and init them
  if (scanDriver(DRV_SHORT)) {
    Serial.println("{\"fshift\":{\"type\":\"short\",\"ready\":\"yes\"}}");
    drv_count++;
  } else
  {
    Serial.println("{\"fshift\":{\"type\":\"short\",\"ready\":\"no\"}}");
  }
  if (scanDriver(DRV_LONG)) {
    Serial.println("{\"fshift\":{\"type\":\"long\",\"ready\":\"yes\"}}");
    drv_count++;
  } else
  {
    Serial.println("{\"fshift\":{\"type\":\"long\",\"ready\":\"no\"}}");
  }
 
  if (drv_count == 2) {
    initDriver();
    Serial.println("{\"status\":\"ready\"}");
  }
}


/**
 * @brief Loops endlessly after setup() is done
 */
void loop()
{
  /* JSON parser */
  if (serial_stream.available()) {
    /* First, skip any accidental whitespace like newlines. */
    serial_stream.skip();
  }

  if (serial_stream.available()) {
    /* Something real on input, let's take a look. */
    aJsonObject *msg = aJson.parse(&serial_stream);
    processMessage(msg);
    aJson.deleteItem(msg);
  }
}


/**
 * @brief Process JSON message
 *
 * @params msg Pointer to message of JSON object
 */
void processMessage(aJsonObject *msg)
{
  
  // Positioning of filters; e.g. {"fpos":{"type":"short","pos":200}}
  aJsonObject *fpos = aJson.getObjectItem(msg, "fpos");
  if (fpos != NULL)
  {
    String filter_type;
    int pos_value, all_ok = 0;
    aJsonObject *fpositem = aJson.getObjectItem(fpos, "type");
    if (fpositem->type == aJson_String)
    {
      filter_type = fpositem->valuestring;
      all_ok++;
    }
    fpositem = aJson.getObjectItem(fpos, "pos");
    if (fpositem->type == aJson_Int)
    {
      pos_value = fpositem->valueint;
      all_ok++;
    }
    if (all_ok == 2)
    {
      if (filter_type == "short")
      {
        setPosition(DRV_SHORT, pos_value);
      }
      else if (filter_type == "long")
      {
        setPosition(DRV_LONG, pos_value);
      }
    }
  }
  
  // Drive and Stop filter; e.g. {"fdrive":{"type":"short","action":"softstop"}}
  aJsonObject *fdrive = aJson.getObjectItem(msg, "fdrive");
  if (fdrive != NULL)
  {
    String filter_type;
    aJsonObject *fdriveitem = aJson.getObjectItem(fdrive, "type");
    if (fdriveitem->type == aJson_String)
      filter_type = fdriveitem->valuestring;
        
    fdriveitem = aJson.getObjectItem(fdrive, "action");
    if (fdriveitem->valuestring == "hardstop")
    {
      hardStop(DRV_ALL);  // both filters
    }
    else if (fdriveitem->valuestring == "softstop")
    {
      softStop(DRV_ALL);  // both filters
    }
    else if (fdriveitem->valuestring == "resetpos")
    {
      if (filter_type == "short")
        resetPosition(DRV_SHORT);
      else if (filter_type == "long")
        resetPosition(DRV_LONG);
    }
    else if (fdriveitem->valuestring == "resetdefault")
    {
      if (filter_type == "short")
        resetToDefault(DRV_SHORT);
      else if (filter_type == "long")
        resetToDefault(DRV_LONG);
    }
    else if (fdriveitem->valuestring == "gotosecure")
    {
      if (filter_type == "short")
        gotoSecurePosition(DRV_SHORT);
      else if (filter_type == "long")
        gotoSecurePosition(DRV_LONG);
    }
  }
  
  // Retreive and set motor parameter
  aJsonObject *fmotor = aJson.getObjectItem(msg, "fmotor");
  if (fmotor != NULL)
  {
    String filter_type;
    aJsonObject *fmotoritem = aJson.getObjectItem(fmotor, "type");
    if (fmotoritem->type == aJson_String)
      filter_type = fmotoritem->valuestring;
    
    if (filter_type == "short")
    {
      fmotoritem = aJson.getObjectItem(fmotor, "ihold");
      fparamShort.IHold = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "irun");
      fparamShort.IRun = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "vmin");
      fparamShort.VMin = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "vmax");
      fparamShort.VMax = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "acc");
      fparamShort.Acc = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "shaft");
      fparamShort.Shaft = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "sepos");
      fparamShort.SecPosHi = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "stepmode");
      fparamShort.StepMode = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "accshape");
      fparamShort.AccShape = fmotoritem->valueint;

      setMotorParam(DRV_SHORT, &fparamShort);
    }
    else if (filter_type == "long")
    {
      fmotoritem = aJson.getObjectItem(fmotor, "ihold");
      fparamLong.IHold = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "irun");
      fparamLong.IRun = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "vmin");
      fparamLong.VMin = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "vmax");
      fparamLong.VMax = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "acc");
      fparamLong.Acc = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "shaft");
      fparamLong.Shaft = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "sepos");
      fparamLong.SecPosHi = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "stepmode");
      fparamLong.StepMode = fmotoritem->valueint;
      fmotoritem = aJson.getObjectItem(fmotor, "accshape");
      fparamLong.AccShape = fmotoritem->valueint;

      setMotorParam(DRV_LONG, &fparamLong);
    }
  }

  
  // Retreive and set stall detection parameter
  aJsonObject *fstall = aJson.getObjectItem(msg, "fstall");
  if (fstall != NULL)
  {
    String filter_type;
    aJsonObject *fstallitem = aJson.getObjectItem(fstall, "type");
    if (fstallitem->type == aJson_String)
      filter_type = fstallitem->valuestring;
    
    if (filter_type == "short")
    {  
      fstallitem = aJson.getObjectItem(fstall, "dc100");
      fstallparamShort.DC100En = fstallitem->valueint;
      fstallitem = aJson.getObjectItem(fstall, "fs2stallen");
      fstallparamShort.FS2StallEN = fstallitem->valueint;
      fstallitem = aJson.getObjectItem(fstall, "minsamples");
      fstallparamShort.MinSamples = fstallitem->valueint;
      fstallitem = aJson.getObjectItem(fstall, "delthr");
      fstallparamShort.DelThr = fstallitem->valueint;
      fstallitem = aJson.getObjectItem(fstall, "absthr");
      fstallparamShort.AbsThr = fstallitem->valueint;
      fstallitem = aJson.getObjectItem(fstall, "pwmjen");
      fstallparamShort.PWMJEn = fstallitem->valueint;
      
      setStallParam(DRV_SHORT, &fstallparamShort);
    }
    else if (filter_type == "long")
    {
      fstallitem = aJson.getObjectItem(fstall, "dc100");
      fstallparamLong.DC100En = fstallitem->valueint;
      fstallitem = aJson.getObjectItem(fstall, "fs2stallen");
      fstallparamLong.FS2StallEN = fstallitem->valueint;
      fstallitem = aJson.getObjectItem(fstall, "minsamples");
      fstallparamLong.MinSamples = fstallitem->valueint;
      fstallitem = aJson.getObjectItem(fstall, "delthr");
      fstallparamLong.DelThr = fstallitem->valueint;
      fstallitem = aJson.getObjectItem(fstall, "absthr");
      fstallparamLong.AbsThr = fstallitem->valueint;
      fstallitem = aJson.getObjectItem(fstall, "pwmjen");
      fstallparamLong.PWMJEn = fstallitem->valueint;
      
      setStallParam(DRV_LONG, &fstallparamLong);
    }
  }
}


/**
 * @brief Generate JSON message to send motor status information
 *
 * @param filter_type Type of optical filter: "short" / "long"
 */
aJsonObject *createMsgMotorStatus(char* filter_type)
{
  aJsonObject *root, *msg;
  root = aJson.createObject();
  tmc223status *pfstat;  // Pointer to status parameter

  if (filter_type == "short")
  {
    pfstat = &fstatShort;
    getFullStatus1(DRV_SHORT, pfstat, NULL);
  }
  else if (filter_type == "long")
  {
    pfstat = &fstatLong;
    getFullStatus1(DRV_LONG, pfstat, NULL);
  }
    
  aJson.addItemToObject(root, "fstat", msg = aJson.createObject());
  aJson.addStringToObject(msg, "type", filter_type);
  aJson.addNumberToObject(msg, "irun", pfstat->IRun);
  aJson.addNumberToObject(msg, "ihold", pfstat->IHold);
  aJson.addNumberToObject(msg, "vmax", pfstat->VMax);
  aJson.addNumberToObject(msg, "vmin", pfstat->VMin);
  aJson.addNumberToObject(msg, "acc", pfstat->Acc);
  aJson.addNumberToObject(msg, "shaft", pfstat->Shaft);
  aJson.addNumberToObject(msg, "stepmode", pfstat->StepMode);
  aJson.addNumberToObject(msg, "accshape", pfstat->AccShape);
  aJson.addNumberToObject(msg, "tinfo", pfstat->TInfo);
  aJson.addNumberToObject(msg, "tw", pfstat->TW);
  aJson.addNumberToObject(msg, "tsd", pfstat->TSD);
  aJson.addNumberToObject(msg, "uv2", pfstat->UV2);
  aJson.addNumberToObject(msg, "eldef", pfstat->ElDef);
  aJson.addNumberToObject(msg, "steploss", pfstat->SteppLoss);
  aJson.addNumberToObject(msg, "vddreset", pfstat->VddReset);
  aJson.addNumberToObject(msg, "cpfail", pfstat->CPFail);
  aJson.addNumberToObject(msg, "ovc2", pfstat->OVC2);
  aJson.addNumberToObject(msg, "ovc1", pfstat->OVC1);
  aJson.addNumberToObject(msg, "esw", pfstat->ESW);
  aJson.addNumberToObject(msg, "motion", pfstat->Motion);

  return root;
}


/**
 * @brief Generate JSON message to send stall detection status
 *
 * @param filter_type Type of filter: "short" / "long"
 */
aJsonObject *createMsgStall(char* filter_type)
{
  aJsonObject *root, *sub;
  root = aJson.createObject();
  tmc223stall *pfstall;
  
  if (filter_type == "short")
  {
    pfstall = &fstallShort;
    getFullStatus1(DRV_SHORT, NULL, pfstall);  // Get AbsThr and DelThr
    getFullStatus2(DRV_SHORT, NULL, NULL, pfstall);
  }
  else if (filter_type == "long")
  {
    pfstall = &fstallLong;
    getFullStatus1(DRV_LONG, NULL, pfstall);  // Get AbsThr and DelThr
    getFullStatus2(DRV_LONG, NULL, NULL, pfstall);
  }

  aJson.addItemToObject(root, "fstall", sub = aJson.createObject());
  aJson.addStringToObject(sub, "type", filter_type);
  aJson.addNumberToObject(sub, "dc100", pfstall->DC100);
  aJson.addNumberToObject(sub, "fs2stallen", pfstall->FS2StallEn);
  aJson.addNumberToObject(sub, "pwmjen", pfstall->PWMJEn);
  aJson.addNumberToObject(sub, "dc100sten", pfstall->DC100StEn);
  aJson.addNumberToObject(sub, "minsamples", pfstall->MinSamples);
  aJson.addNumberToObject(sub, "delstallhi", pfstall->DelStallHi);
  aJson.addNumberToObject(sub, "delstalllo", pfstall->DelStallLo);
  aJson.addNumberToObject(sub, "absstall", pfstall->AbsStall);
  aJson.addNumberToObject(sub, "absthr", pfstall->AbsThr);
  aJson.addNumberToObject(sub, "delthr", pfstall->DelThr);
  
  return root;
}


/**
 * @brief Init the TMC223 drivers
 */
void initDriver( void )
{
  // Bring filter shifter out of shutdown state and reate status
  getFullStatus1(DRV_SHORT, &fstatShort, NULL);
  getFullStatus1(DRV_LONG, &fstatLong, NULL);
  
  // Get actual position to prevent deadlock state
  actualPosShort = getFullStatus2(DRV_SHORT, NULL, NULL, NULL);
  actualPosLong = getFullStatus2(DRV_LONG, NULL, NULL, NULL);
  
  #ifdef _DEBUG
  Serial.print("initDriver() -> acutalPosShort = ");
  Serial.println(actualPosShort, DEC);
  Serial.print("initDriver() -> acutalPosLong = ");
  Serial.println(actualPosLong, DEC);
  #endif
  
  // Define motor parameter...
  fparamShort.IHold = 0x0;
  fparamShort.IRun =  0xE;
  fparamShort.VMin =  0x1;
  fparamShort.VMax =  0x9;
  fparamShort.Acc =   0x9;
  fparamShort.Shaft = 0x0;  // clockwise
  fparamShort.SecPosHi = 0x4;  // value 0x400 ignores secure position!
  fparamShort.SecPosLo = 0x00;
  fparamShort.StepMode = 0x03;  // 1/16 micro-stepping
  fparamShort.AccShape = 0;     // acceleration with Acc parameter
  
  fparamLong.IHold = 0x7;
  fparamLong.IRun =  0xF;
  fparamLong.VMin =  0x1;
  fparamLong.VMax =  0x9;
  fparamLong.Acc =   0x5;
  fparamLong.Shaft = 0x1;  // counter-clockwise
  fparamLong.SecPosHi = 0x4;  // value 0x400 ignores secure position!
  fparamLong.SecPosLo = 0x00;
  fparamLong.StepMode = 0x03;  // 1/16 micro-stepping
  fparamLong.AccShape = 0;     // acceleration with Acc parameter
  
  // ... and set them
  setMotorParam(DRV_SHORT, &fparamShort);
  setMotorParam(DRV_LONG, &fparamLong);
  
  // Actual position is unknown. Hence, do init drive
  // Drive not with maximum velocity!
  runInit(DRV_SHORT, 0x0, 0x1, actualPosShort-5500, actualPosShort-231);
  runInit(DRV_LONG, 0x0, 0x1, actualPosLong-5500, actualPosLong-231);

  // Send initial status information via JSON
  delay(3000);  // Wait until init is completed
  
  aJsonObject *msg = createMsgMotorStatus("short");
  aJson.print(msg, &serial_stream);
  Serial.println(); // Add newline.
  msg = createMsgMotorStatus("long");
  aJson.print(msg, &serial_stream);
  Serial.println();

  msg = createMsgStall("short");
  aJson.print(msg, &serial_stream);
  Serial.println();
  msg = createMsgStall("long");
  aJson.print(msg, &serial_stream);
  Serial.println();
  aJson.deleteItem(msg);
}


 
/**
 * @brief Scan for filter driver attached to the I2C bus
 *
 * @param i2c_adr Address of I2C slave
 *
 * @return found
 */
int scanDriver( byte i2c_adr )
{
  byte error;
  int found;    // device found variable
 
  #ifdef _DEBUG
  Serial.println("Scanning I2C for filter drivers...");
  #endif
  
  // uses return value of the Write.endTransmission
  // to see if a device did acknoledge to the address.
  Wire.beginTransmission(i2c_adr);
  error = Wire.endTransmission();
  
  if (error == 0)
  {
    #ifdef _DEBUG
    Serial.print("I2C device found at address 0x");
    if (i2c_adr<16)
      Serial.print("0");
    Serial.print(i2c_adr, HEX);
    Serial.println(" !");
    #endif
    
    found = 1;  // device found
  }
  else if (error == 4) 
  {
    #ifdef _DEBUG
    Serial.print("Unknow error at address 0x");
    if (i2c_adr<16) 
      Serial.print("0");
    Serial.println(i2c_adr,HEX);
    #endif
      
    found = -1;  // device error
  }
  else
  {
    #ifdef _DEBUG
    Serial.print("No device found at address 0x");
    if (i2c_adr<16)
      Serial.print("0");
    Serial.println(i2c_adr, HEX);
    #endif
    
    found = 0;  // device not found
  }
  
  return found;
}


/**
 * @brief Returns complete status of the TMC223 chip
 * @details With parameter set to NULL error flags can be reset without the need to know the status
 *
 * @param i2c_adr Address of I2C slave
 * @param tmc223status Pointer to struct that holds TMC223 status information (or NULL)
 * @param tmc223stall Pointer to struct that holds TMC223 stall status information (or NULL)
 */
void getFullStatus1(byte i2c_adr, struct tmc223status *tmc223status, struct tmc223stall *tmc223stall)
{
  unsigned int i;    // dummy (counter)
  unsigned int numBytes = 8;  // number of bytes to read
  
  // Send command byte to TMC223
  Wire.beginTransmission(i2c_adr);  // write device address
  Wire.write(0x81);  // GetFullStatus1 command byte
  Wire.endTransmission();
  // Read 8 bytes from TMC223
  Wire.requestFrom(i2c_adr, numBytes);  // read 8 bytes
  for(i=0; i<numBytes; i++)
  {
    rxBuffer[i] = Wire.read();  // no Wire.endTransmission() needed!
  }
  
  // Copy received data to TMC223 status struct (if pointer is not NULL)
  if(tmc223status != NULL)
  {
    for(i=0; i<5; i++) {
      *(((unsigned char *) tmc223status)+i)=rxBuffer[i+1];
    }
  }
  // Copy last received byte to TMC223 stall struct (if pointer is not NULL)
  if(tmc223stall != NULL)
  {
    *(((unsigned char *)tmc223stall)+2) = rxBuffer[7];
  }
  
  #ifdef _DEBUG
  Serial.print("getFullStatus1: ");
  for(i=0; i<numBytes; i++)
  {
    if (rxBuffer[i] < 16)
      Serial.print("0");
    Serial.print(rxBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\n");
  #endif
}


/**
 * @brief Returns actual, target and secure position.
 * @details Reads TMC223 position registers. NULL for values that are not used.
 *
 * @param i2c_adr Address of I2C slave
 * @param targetPosition Pointer to variable holding target position (or NULL)
 * @param securePosition Pointer to variable holding secure position (or NULL)
 * @param tmc223stall Pointer to struct that holds stall status (or NULL)
 *
 * @return Actual position of stepper motor.
 */
int getFullStatus2(byte i2c_adr, int *targetPosition, int *securePosition, struct tmc223stall *tmc223stall)
{
  unsigned int i;    // dummy (counter)
  unsigned int numBytes = 8;  // number of bytes to read
  
  // Send command byte to TMC223
  Wire.beginTransmission(i2c_adr);  // write device address
  Wire.write(0xFC);  // GetFullStatus2 command byte
  Wire.endTransmission();
  // Read 8 bytes from TMC223
  Wire.requestFrom(i2c_adr, numBytes);  // read 8 bytes
  for(i=0; i<numBytes; i++)
  {
    rxBuffer[i] = Wire.read();  // no Wire.endTransmission() needed!
  }
  
  // Copy received data to position values
  if(targetPosition != NULL)
    *targetPosition = (rxBuffer[3]<<8) | rxBuffer[4];  // combine Hi and Lo Byte
  if(securePosition != NULL)
    *securePosition = ((rxBuffer[6]&0x07)<<8) | rxBuffer[5];

  // Copy received data to TMC223 stall detection status struct (if pointer is not NULL)
  if(tmc223stall != NULL)
  {
    for(i=0; i<2; i++) {
      *(((unsigned char *) tmc223stall)+i)=rxBuffer[i+6];
    }
  }
    
  return (rxBuffer[1]<<8) | rxBuffer[2];

  
  #ifdef _DEBUG
  for(i=0; i<numBytes; i++)
  {
    Serial.print("getFullStatus2: ");
    if (rxBuffer[i] < 16)
      Serial.print("0");
    Serial.print(rxBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\n");
  #endif
}


/**
 * @brief Set the TMC223 motor parameters in RAM.
 *
 * @param i2c_adr Address of I2C slave
 * @param tmc223param Pointer to struct that holds TMC223 motor parameter
 */
void setMotorParam(byte i2c_adr, struct tmc223param *tmc223param)
{
  unsigned int i;    // dummy (counter)
  
  // Send command byte to TMC223
  Wire.beginTransmission(i2c_adr);  // write device address
  Wire.write(0x89);  // SetMotorParam command byte
  Wire.write(0xFF);  // N/A
  Wire.write(0xFF);  // N/A
  // Send 5 bytes of TMC223 parameter struct to device
  for(i=0; i<5; i++) {
    Wire.write(*(((unsigned char *) tmc223param)+i));
  }
  Wire.endTransmission();
  
  #ifdef _DEBUG
  Serial.print("setMotorParam: ");
  for(i=0; i<5; i++) {      
    Serial.print(*(((unsigned char *) tmc223param)+i), HEX);
  }
  Serial.print("\n");
  #endif
}


/**
 * @brief Set the TMC223 parameter for sensorless stall detection
 *
 * @param i2c_adr Address of I2C slave
 * @param tmc223stallparam Pointer to struct that holds TMC223 stall detection parameter
 */
void setStallParam(byte i2c_adr, struct tmc223stallparam *tmc223stallparam)
{
  unsigned int i;    // dummy (counter)
  
  // Send command byte to TMC223
  Wire.beginTransmission(i2c_adr);  // write device address
  Wire.write(0x96);  // SetMotorParam command byte
  Wire.write(0xFF);  // N/A
  Wire.write(0xFF);  // N/A
  // Send 5 bytes of TMC223 stall detection parameter struct to device
  for(i=0; i<5; i++) {
    Wire.write(*(((unsigned char *) tmc223stallparam)+i));
  }
  Wire.endTransmission();
  
  #ifdef _DEBUG
  Serial.print("setMotorStallParam: ");
  for(i=0; i<5; i++) {      
    Serial.print(*(((unsigned char *) tmc223stallparam)+i), HEX);
  }
  Serial.print("\n");
  #endif
}


/**
 * @brief Pre-programmed motion sequence to move to a mechanical limit
 * @details Initialize positioning of the motor by seeking the zero
 *          (or reference) position. Once it is started it can NOT be
 *          interrupted except when a HardStop is received.
 *          If the SecPos value equals 0x400 the final travel to the
 *          secure position is omitted
 *
 * @param i2c_adr Address of I2C slave
 * @param VMin Minimum velocity of stepper motor for reference search
 * @param Vmax Maximum velocity of stepper motor for reference search
 * @param Position1 First motion that _must_ drive into a stop for sure
 * @param Position2 Second motion: drive out of stall or into to compensate for bouncing
 */
void runInit(byte i2c_adr, unsigned char Vmin, unsigned char Vmax, int Position1, int Position2)
{
  // Send command byte and data to TMC223
  Wire.beginTransmission(i2c_adr);  // write device address
  Wire.write(0x88);  // RunInit command byte
  Wire.write(0xFF);  // N/A
  Wire.write(0xFF);  // N/A
  Wire.write(Vmax<<4) | (Vmin<<4);
  Wire.write(highByte(Position1));
  Wire.write(lowByte(Position1));
  Wire.write(highByte(Position2));
  Wire.write(lowByte(Position2));
  Wire.endTransmission();
  
  #ifdef _DEBUG
  Serial.println("runInit(): DONE");
  #endif
}


/**
 * @brief Drive te motor to a given position (target position).
 * @details The motor is driven relative to the zero position, defined in number of half or micro steps, according to the StepMode value
 *
 * @param i2c_adr Address of I2C slave
 * @param Position Target Position
 */
void setPosition(byte i2c_adr, int Position)
{
  // Send command byte to TMC223
  Wire.beginTransmission(i2c_adr);  // write device address
  Wire.write(0x8B);  // SetPosition command byte
  Wire.write(0xFF);  // N/A
  Wire.write(0xFF);  // N/A
  Wire.write(highByte(Position));  // TagPos(15:8)
  Wire.write(lowByte(Position));  // TagPos(7:0)
  Wire.endTransmission();
  
  #ifdef _DEBUG
  Serial.print("setPosition(): ");
  Serial.print(Position);
  Serial.print("\n");
  #endif
}


/**
 * @brief Set actual position to zero (without moving the motor)
 *
 * @param i2c_adr Address of I2C slave
 */
void resetPosition(byte i2c_adr)
{
  // Send command byte to TMC223
  Wire.beginTransmission(i2c_adr);  // write device address
  Wire.write(0x86);  // ResetPosition command byte
  Wire.endTransmission();
  
  #ifdef _DEBUG
  Serial.print("resetPosition(): DONE\n");
  #endif
}


/**
 * @brief Reset all values to the default values (overwrite RAM with OTP contents)
 * @details ActPos is not modified. It's value is copied into TagPos register in order to avoid an attempt to position the motor to zero
 *
 * @param i2c_adr Address of I2C slave
 */
void resetToDefault(byte i2c_adr)
{
  // Send command byte to TMC223
  Wire.beginTransmission(i2c_adr);  // write device address
  Wire.write(0x87);  // ResetToDefault command byte
  Wire.endTransmission();
  
  #ifdef _DEBUG
  Serial.print("resetToDefault(): DONE\n");
  #endif
}


/**
 * @brief Drives motor to secure position
 * @details If secure position value equals 0x400 it is disabled and command is ignored.
 *
 * @param i2c_adr Address of I2C slave
 */
void gotoSecurePosition(byte i2c_adr)
{
  // Send command byte to TMC223
  Wire.beginTransmission(i2c_adr);  // write device address
  Wire.write(0x87);  // GotoSecurePosition command byte
  Wire.endTransmission();
  
  #ifdef _DEBUG
  // if SecPos = 0x400 send ignore!
  Serial.print("gotoSecurePosition(): DONE\n");
  #endif
}


/**
 * @brief Immediate full stop
 *
 * @param i2c_adr Address of I2C slave
 */
void hardStop(byte i2c_adr)
{
  // Send command byte to TMC223
  Wire.beginTransmission(i2c_adr);  // write device address
  Wire.write(0x85);  // HardStop command byte
  Wire.endTransmission();
  
  #ifdef _DEBUG
  Serial.print("hardStop(): DONE\n");
  #endif
}


/**
 * @brief Motor stopping with deceleration (to Vmin) phase
 *
 * @param i2c_adr Address of I2C slave
 */
void softStop(byte i2c_adr )
{
  // Send command byte to TMC223
  Wire.beginTransmission(i2c_adr);  // write device address
  Wire.write(0x8F);  // SoftStop command byte
  Wire.endTransmission();
  
  #ifdef _DEBUG
  Serial.print("softStop(): DONE\n");
  #endif
}


/**
 * @brief Get motor parameter with getFullStatus1() and getFullStatus2()
 *
 * @param i2c_adr Address of I2C slave
 * @param tmc223param Pointer to struct that holds TMC223 motor parameter
 */
void getMotorParam(byte i2c_adr, struct tmc223param *tmc223param)
{
  int tmpSecurePosition;
  tmc223status tmpStatus;

  // Get status for specified filtershifter
  getFullStatus1(i2c_adr, &tmpStatus, NULL);

  // Assign status values
  tmc223param->IRun = tmpStatus.IRun;
  tmc223param->IHold = tmpStatus.IHold;
  tmc223param->VMin = tmpStatus.VMin;
  tmc223param->VMax = tmpStatus.VMax;
  tmc223param->Acc = tmpStatus.Acc;
  tmc223param->AccShape = tmpStatus.AccShape;
  tmc223param->Shaft = tmpStatus.Shaft;
  tmc223param->StepMode = tmpStatus.StepMode;

  // Now read secure position only
  getFullStatus2(i2c_adr, NULL, &tmpSecurePosition, NULL);
  tmc223param->SecPosHi = highByte(tmpSecurePosition);
  tmc223param->SecPosLo = lowByte(tmpSecurePosition);
  
  #ifdef _DEBUG
  Serial.print("getMotorParam(): DONE\n");
  // print param-values
  #endif
}

