# Proyecto-PI2
Código:

#include <Keypad.h>
#include <Wire.h>      // libreria de comunicacion por I2C
#include <LCD.h>      // libreria para funciones de LCD
#include <LiquidCrystal_I2C.h>    // libreria para LCD por I2C
#include "Adafruit_Thermal.h"
#include "SoftwareSerial.h"
#include <EEPROM.h>
#define TX_PIN 6 // Arduino transmit  YELLOW WIRE  labeled RX on printer
#define RX_PIN 5 // Arduino receive   GREEN WIRE   labeled TX on printer
//
//Para ESP8266
#define DEBUG true
LiquidCrystal_I2C lcd (0x27, 2, 1, 0, 4, 5, 6, 7); // DIR, E, RW, RS, D4, D5, D6, D7

SoftwareSerial mySerial(RX_PIN, TX_PIN); // Declare SoftwareSerial obj first
Adafruit_Thermal printer(&mySerial);     // Pass addr to printer constructor
//==========================================================
                      //PARA ACELEROMETRO
//==========================================================
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

//Para teclado////////////////////////////
const byte Filas=4;
const byte Columnas=4;
char Tecla;
int  contador = 1;    // cuenta el nivel del menu en el que se esta
int pasajeros = 0;
char pfinal = ' ';
char paraderos[5] = {'1','2','3','4','5'};

/////////// CANTIDAD DE BAJADORES EN PARADEROS//
int paradero1=0;
int paradero2=0;
int paradero3=0;
int paradero4=0;
int paradero5=0;
int proximo_paradero=0;
int TOKEN_NUMBER = 0;
int paradero_inicial = 0;

/////////// DISTANCIA DE PARADEROS //

int distp1 = 40;
int distp2 = 80;
int distp3 = 120;
int distp4 = 160;
int distp5 = 200;

////////////////////////////
String Direccion_IP = " "; 
int distancia_cm = 0;
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
///////////////////////////////////////////////////////////////////

char keys[Filas][Columnas]={
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte pinesFilas[Filas]={47,45,43,41};
byte pinesColumnas[Columnas]={39,37,35,33};
Keypad teclado = Keypad(makeKeymap(keys), pinesFilas, pinesColumnas, Filas, Columnas);
     

//////////////////////////////////////////////////


///Fuciones para la pantalla (Interfaz)
void Pantalla_paradero_final(){
    lcd.setCursor(0, 0);  
    lcd.print("P.final:");
    lcd.setCursor(8, 0);
    lcd.print(pfinal);
    lcd.setCursor(12,0);
    lcd.print("NP");
    lcd.setCursor(14,0);
    lcd.print(pasajeros);
    lcd.setCursor(0, 1);  
    lcd.print("Sig:A");
    lcd.setCursor(10, 1);    
    lcd.print("Canc:B");

}

void Pantalla_tipo(){
    lcd.setCursor(0, 0);  
    lcd.print("Uni/Esc: A");
    lcd.setCursor(0, 1);  
    lcd.print("General: B");
}
//========================================================================
////////////////////////IMPRESORA
//=====================================================================
void Imprime_vauchergeneral() {
  printer.write(10);
  printer.justify('C');
  printer.setSize('M');
  printer.println("*** TU RUTEC ***");
  printer.justify('C');
  printer.setSize('L');        // Set type size, accepts 'S', 'M', 'L'
  printer.println("GENERAL");
  printer.justify('C');
  printer.setSize('L'); 
  printer.println("Nro.TICKET ");// Set type size, accepts 'S', 'M', 'L'
  printer.println(TOKEN_NUMBER);
  printer.boldOn();
  printer.setSize('M');
  printer.println("RUC:20283510204");
  printer.boldOff();
  printer.justify('C');
  printer.print("Paradero inicial: ");
  printer.println(paradero_inicial);
  printer.justify('C');
  printer.print("Paradero final: "); 
  printer.println(pfinal); 
  printer.justify('C');
  printer.setSize('S');
  printer.println("Exija y conserve su ticket");
  printer.justify('C');
  printer.setSize('S');
  printer.println("***  TEN UN GRAN DIA  ***");
  printer.write(10);
  printer.write(10);
}

void Imprime_vaucherunivesitario() {
  printer.write(10);
  printer.justify('C');
  printer.setSize('M');
  printer.println("*** TU RUTEC ***");
  printer.justify('C');
  printer.setSize('L');        // Set type size, accepts 'S', 'M', 'L'
  printer.println("ESC/UNIV");
  printer.justify('C');
  printer.setSize('L'); 
  printer.println("Nro.TICKET ");// Set type size, accepts 'S', 'M', 'L'
  printer.println(TOKEN_NUMBER);
  printer.boldOn();
  printer.setSize('M');
  printer.println("RUC:20283510204");
  printer.boldOff();
  printer.justify('C');
  printer.print("Paradero inicial: ");
  printer.println(paradero_inicial);
  printer.justify('C');
  printer.print("Paradero final: "); 
  printer.println(pfinal); 
  printer.justify('C');
  printer.setSize('S');
  printer.println("Exija y conserve su ticket");
  printer.justify('C');
  printer.setSize('S');
  printer.println("***  TEN UN GRAN DIA  ***");
  printer.write(10);
  printer.write(10);
}

///
//=====================================================================


void setup()
{
    Serial.begin(9600); 
    Serial3.begin(9600);
    mySerial.begin(9600); 
    printer.begin();
  //Para aceleromtro
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    /////////////////
    lcd.setBacklightPin(3,POSITIVE);  // puerto P3 de PCF8574 como positivo
    lcd.begin(16, 2);     // 16 columnas por 2 lineas para LCD 1602A
    lcd.clear();      // limpia pantalla
    lcd.setCursor (0,0);
    lcd.print("==> TU RUTEC <==");
    lcd.setCursor (4,1);
    lcd.print("Load....");
    Serial.print("Capacidad de la memoria EEPROM: " );
    Serial.println(EEPROM.length() );
    Serial.print(" "); 
    Serial.print("Valor almacenado en direccion 0: ");  // imprime texto
    Serial.println( EEPROM.read(0) );         // lee direccion cero y muestra
  
   //=============================== 
   //Para inicializar el acelerómetro
    
    while (!Serial); 

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));



    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(134);
    mpu.setYGyroOffset(66);
    mpu.setZGyroOffset(7);
    mpu.setXAccelOffset(940); 
    mpu.setYAccelOffset(-4989); 
    mpu.setZAccelOffset(1944); 
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    //===============
    //Para el ESP88..
    delay(500); 
  
  //sendData("AT+RST\r\n",2000,DEBUG); // reset del módulo
  //delay(10000);
  sendData("AT+CWMODE=3\r\n",1000,DEBUG); // configuración punto de acceso
  sendData("AT+CIPMUX=1\r\n",1000,DEBUG); // multiples conexiones
  sendData("AT+CIPSERVER=1,80\r\n",1000,DEBUG); // encender servidor en puerto 80
  delay(10000);
  sendData("AT+CWJAP=\"HUAWEI Y7\",\"12345678\"\r\n",1000,DEBUG);//red wifi, usuario y clave
  delay(10000);
  Direccion_IP = sendData("AT+CIFSR\r\n",1000,DEBUG); // se obtienen direcciones IP 
  delay(10000);
  printer.println("Direcciones");
  printer.justify('C');
  printer.setSize('S');        // Set type size, accepts 'S', 'M', 'L'
  printer.println(Direccion_IP);
  printer.write(10);
  printer.write(10);
 

    lcd.clear(); 
}

void loop()
{
Tecla = teclado.getKey();  

if (Tecla != 0) {      
    lcd.clear();
    delay(100);
 }

 printer.begin();
 //pasajeros = EEPROM.read(0);
   if(distancia_cm > 0){
    if(distancia_cm > 0 && distancia_cm < 39){
      paradero_inicial = 0;
      proximo_paradero = 1;
    }
    if(distancia_cm == distp1){
      paradero_inicial = 1;
      proximo_paradero = 2;
      pasajeros = pasajeros - paradero1;
      paradero1 = 0;
      Serial.print("pasajeros: ");
      Serial.println(pasajeros);
      lcd.clear();
    }
     if(distancia_cm == distp2){
      paradero_inicial = 2;
      proximo_paradero = 3;
      pasajeros = pasajeros - paradero2;
      paradero2 = 0;
      Serial.print("pasajeros: ");
      Serial.println(pasajeros);
      lcd.clear();
    }
     if(distancia_cm == distp3){
      paradero_inicial = 3;
      proximo_paradero = 4;
      pasajeros = pasajeros - paradero3;
      paradero3 = 0;
      Serial.print("pasajeros: ");
      Serial.println(pasajeros);
      lcd.clear();
    }
    if(distancia_cm == distp4){
      paradero_inicial = 4;
      proximo_paradero = 5;
      pasajeros = pasajeros - paradero4;
      paradero4 = 0;
      Serial.print("pasajeros: ");
      Serial.println(pasajeros);
      lcd.clear();
    }
    if(distancia_cm == distp5){
      paradero_inicial = 5;
      proximo_paradero = 6;
      pasajeros = pasajeros - paradero5;
      paradero5 = 0;
      Serial.print("pasajeros: ");
      Serial.println(pasajeros);
      lcd.clear();
    }
   }

 //===========================================================================
 //Para acelerometro
     if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  

    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(ypr[2] * 180/M_PI);
            distancia_cm  = sqrt(pow((ypr[0] * 180/M_PI),2)+pow((ypr[1] * 180/M_PI),2)+pow((ypr[2] * 180/M_PI),2));
            //Serial.print("Distancia en cm: ");
            //Serial.println(distancia_cm);

        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

    //////////Final de acelerómetro

 if(contador == 2){ Pantalla_tipo();accion_4();}
 if(contador == 1){ Pantalla_paradero_final();accion_2();accion_3();}
 
}



/////////////////////Accion 2 //////////////////////////////
void accion_2(){
  for(int i =0;i < 5; i++){
    if(Tecla == paraderos[i])pfinal= Tecla;
  }
}
  /////////////////////Accion 3 //////////////////////////////
void accion_3(){
  if(Tecla == 'A' && pfinal != ' ') {
  if(pfinal == '1') { paradero1++;Serial.print("paradero1: ");Serial.println(paradero1);}
  if(pfinal == '2') { paradero2++;Serial.print("paradero2: ");Serial.println(paradero2);}
  if(pfinal == '3') { paradero3++;Serial.print("paradero3: ");Serial.println(paradero3);}
  if(pfinal == '4') { paradero4++;Serial.print("paradero4: ");Serial.println(paradero4);}
  if(pfinal == '5') { paradero5++;Serial.print("paradero5: ");Serial.println(paradero5);}
  lcd.clear();contador=2;}
  else{contador == 1;}
  if(Tecla == 'B') {lcd.clear();pfinal = ' ';contador=1;}
  if(Tecla == '*') {lcd.clear();pfinal = ' ';contador=1;}
}

/////////////////////accion 4  //////////////////////////////////
void accion_4(){
  if(Tecla == 'A') {
    pasajeros++;
    TOKEN_NUMBER++;
    Imprime_vaucherunivesitario();
    lcd.clear();
    contador=1;
    pfinal = ' ';
    }
    
  if(Tecla == 'B') {
    pasajeros++;
    TOKEN_NUMBER++;
  Imprime_vauchergeneral();
  lcd.clear();
  contador=1;
  pfinal = ' ';
  }
  if(Tecla == '*') {lcd.clear();pfinal = ' ';contador=1;}

   //==================
  //Para que el ESP88.. mande los datos
  //=================
  if(Serial3.available()) // Comprueba si el ESP esta enviando mensaje
  {
    if(Serial3.find("+IPD,")){
     delay(1000);    
   
     int connectionId = Serial3.read()-48; // Restar 48 porque la función read () devuelve 
     String webpage = "<head><meta http-equiv=""refresh"" content=""5""></head>"; //refresh de 5 segundos
      webpage+="<h1>Empresa TU RUTEC</h1><h3></h3>";
     webpage+="<h3>Placa de carro: V1H-302</h3>";
     webpage+="<h3>----------------------<h3>";
     webpage+="<h4>Pasajeros: ";
     webpage+=(pasajeros);
     webpage+="<h4>Siguiente paradero: ";
     webpage+=(proximo_paradero);
     webpage+="<h4>Cantidad de personas que bajaran: ";
     if(proximo_paradero == 1){
      webpage+=(paradero1);
     }
     if(proximo_paradero == 2){
      webpage+=(paradero2);
     }
     if(proximo_paradero == 3){
      webpage+=(paradero3);
     }
     if(proximo_paradero == 4){
      webpage+=(paradero4);
     }
     if(proximo_paradero == 5){
      webpage+=(paradero5);
     }
     webpage+="<h5>===============================================================</h5>";
     sendData("AT+CIPSEND=" + String(connectionId) + "," + webpage.length() + "\r\n", 500, true);
     sendData(webpage, 1000, true); // 
     sendData("AT+CIPCLOSE=" + String(connectionId) + "\r\n", 1000, true); 
     }
  }
}

String sendData(String command, const int timeout, boolean debug)
{
    String response = "";
    Serial3.print(command); // Se envía el carácter de lectura a la esp8266
    long int time = millis();
    while( (time+timeout) > millis())
    {while(Serial3.available())
      { 
        char c = Serial3.read(); // Lee el siguiente carácter.
        response+=c;
      }  
    }
    
    if(debug)
    {
      Serial.print(response);
    } 
       
    return response;
    
}
