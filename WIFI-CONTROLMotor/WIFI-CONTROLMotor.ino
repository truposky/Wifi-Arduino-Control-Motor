#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "common.h"
#include <math.h>
#include "MeanFilterLib.h"
#include <Arduino_LSM6DS3.h>
#include <string>
#include <vector>


using namespace std;
//WiFi variable setup
//char ssid[] = "Robotarium";     // your network SSID (name)
//char pass[] = "robotarium";    // your network password (use for WPA, or use as key for WEP)
char ssid[] = "MiFibra-4300";     // your network SSID (name)
char pass[] = "SzreaH22";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
//UDP variables setup
/*-----------ip setup-----------------*/
IPAddress ip_arduino1(192,168,1,5);
IPAddress ip_server(192,168,1,2);
WiFiUDP Udp;
unsigned int localPort = 4243;      // local port to listen on
char packetBuffer[256]; //buffer to hold incoming packet

  
//--------------------------------------------filtro de media movil simple para estabilizar la lectura de rad/s---------------------------------------
MeanFilter<long> meanFilterD(6);
MeanFilter<long> meanFilterI(6);

//Time variables
unsigned long previous_timer;
unsigned long timer=10000;


//operation variables
struct appdata operation_send;
struct appdata *server_operation;


void setup() {
 
  motorSetup();
  //interrupciones para contar pulsos de encoder
  pinMode(encoderD,INPUT_PULLUP);
  pinMode(encoderI,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderI), isrI, RISING);//prepara la entrada del encoder como interrupcion
  attachInterrupt(digitalPinToInterrupt(encoderD), isrD, RISING);
  //se prepara la IMU para poder ser leida
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }
  //se empieza con los motores parados
  fullStop( pinMotorD);
  fullStop( pinMotorI);

  //comunicacion por puerto serie
  Serial.begin(9600);
 /*while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }*/
  //---se prepara la comunicacion UDP------//
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  WiFi.config(ip_arduino1);
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to WiFi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);//se prepara el puerto para escuchar

}
  
 

void loop() {
    int error=0;
    double fD,fI;
    float x, y, z;//variables para el gyroscope
    //se define un contador de tiempo para comprobar que las reudas estan paradas
    timeStopD=millis();
    timeStopI=millis();
    deltaTimeStopD=timeStopD-timeAfterDebounceD;
    deltaTimeStopI=timeStopI-timeAfterDebounceI;
     // start the UDP on port 4243
     //clean udp buffer
      memset (packetBuffer,'\0',MAXDATASIZE);
     
    // if there's data available, read a packet
    
    int packetSize = Udp.parsePacket();
    if (packetSize) 
    {//se comprueba si se recibe un paquete
      
      IPAddress remoteIp = Udp.remoteIP();//proviene del servidor u ordenador central
      int numbytes = Udp.read((byte*)packetBuffer, MAXDATASIZE); //se guardan los datos en el buffer
      server_operation= (struct appdata*)&packetBuffer;
      //se comprueba longitud
      if((numbytes <  HEADER_LEN) || (numbytes != server_operation->len + HEADER_LEN))
      {
          Serial.print("(arduino1) unidad de datos recibida de manera incompleta \n");
          error=1;//error
      }
      else
      {
           Serial.print("\nFrom ");
          IPAddress remoteIp = Udp.remoteIP();
          Serial.print(remoteIp);
          Serial.print(", port ");
          Serial.print(Udp.remotePort());
          Serial.print("\n(arduino1) operacion solicitada ");
          Serial.print(server_operation->op);
          Serial.print("\n");
          Serial.println(server_operation->data);
          switch (server_operation->op)
          {
              case OP_SALUDO:
                  op_saludo();
                  break;
              case OP_MOVE_WHEEL:
                  op_moveWheel();
                  break;
              case OP_STOP_WHEEL:
      
                   break;
              case OP_VEL_ROBOT:
                  op_vel_robot();
                  break;
              default:
      
                   break;
            
          }
      }
    }
    //condition for know when the wheel is stoped
    if(deltaTimeStopD>=200){
      fD=0;
    }
    else{
      fD=(double)1/(deltaTimeD*N)*1000000;
    }
    if(deltaTimeStopI>=200){
      fI=0;
    }
    else{
      fI=(double)1/(deltaTimeI*N)*1000000;
    }
    //condicion para que no supere linealidad y se sature.
    if(fD<31/2/3.14)
    {
      wD=2*3.14*fD;
     
    }
    if(fI<31/2/3.14 )
    {
       wI=2*3.14*fI;
    }
    int ajusteD=pidD(wD,setpointWD);
    int ajusteI=pidI(wI,setpointWI); 
    
  
    
    PWM_D=PWM_D+ajusteD;
    PWM_D=PWM_D*(PWM_D>=MINPWM)+(PWM_D<MINPWM)*MINPWM;
    PWM_I=PWM_I+ajusteI;
    PWM_I=PWM_I*(PWM_I>=MINPWM)+(PWM_I<MINPWM)*MINPWM;
    moveWheel(PWM_I,setpointWI,pinMotorI,backI);
    moveWheel(PWM_D,setpointWD,pinMotorD,backD);
    TimeWait();//se establece 300 milisegundos tiempo suficente para que el PWM cambie sin afectara a los motores
  //Timewait tambien sirve para hacer media movil de los pulsos.
    Serial.print(setpointWD);
    Serial.print(",");
    Serial.print(wD);
    Serial.print(",");
    Serial.print(PWM_D);
    Serial.print(",");
    Serial.print(setpointWI);
    Serial.print(",");
    Serial.print(wI);
    Serial.print(",");
    Serial.println(PWM_I);
  
}

void printWifiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void op_saludo()
{
    operation_send.op=OP_SALUDO;
    operation_send.id=ID;
    strcpy(operation_send.data,"hola Soy arduino1");
    operation_send.len = strlen (operation_send.data);  /* len */
    Udp.beginPacket(ip_server,Udp.remotePort());
    Udp.write((byte*)&operation_send,operation_send.len+HEADER_LEN);
    Udp.endPacket();
    Serial.println("mensaje enviado");
}
void op_message()
{

    
  
}
void op_moveWheel()
{
  
  strcpy(operation_send.data,"mensaje recibido");
  operation_send.op=OP_MESSAGE_RECIVE;
  operation_send.id=ID;
  string data=server_operation->data;
  char del =',';
  vector<string> vel;
  tokenize(data, del,vel);
 

  setpointWD=stod(vel[0]);
  setpointWI=stod(vel[1]);
  if(setpointWD<0)
  {
    setPointGWD=setpointWD;
    setpointWD=setpointWD*(-1);
    backD=true;
  }
  else if(setpointWD>0)
  {
    backD=false;
  }
  if(setpointWI<0)
  {
    setPointGWI=setpointWI;
    setpointWI=setpointWI*(-1);
    backI=true;
  }
  else if(setpointWI>0)
  {
    backI=false;
  }
  //if feedforward detect a problem with discontinuitiy on velocity robot send an alert message
  feedForwardD();
  feedForwardI();
    // send a reply, to the IP address and port that sent us the packet we received
  operation_send.len = strlen (operation_send.data);  /* len */
 
  Udp.beginPacket(ip_server,Udp.remotePort());
  Udp.write((byte*)&operation_send,operation_send.len+HEADER_LEN);
  Udp.endPacket();
  
  moveWheel(PWM_D,setpointWD,pinMotorD,backD);
  moveWheel(PWM_I,setpointWI,pinMotorI,backI);
  TimeWait();
  

   
  
}
void op_StopWheel(){
  
}

void op_vel_robot(){
  operation_send.op=OP_VEL_ROBOT;
  operation_send.id=ID;
  char del=',';
  sprintf(operation_send.data,"%lf",wD);
  sprintf(operation_send.data+sizeof(wD),"%c",del);
  sprintf(operation_send.data+sizeof(wD)+sizeof(del),"%lf",wI);
  Serial.print("vel:");
  Serial.print(operation_send.data);
  operation_send.len = strlen (operation_send.data);  /* len */
  Udp.beginPacket(ip_server,Udp.remotePort());
  Udp.write((byte*)&operation_send,operation_send.len+HEADER_LEN);
  Udp.endPacket();
 }
void TimeWait()
{
   int ahora=millis();
    int resta=0;
    int despues=0;
  while(resta<50){
      despues=millis();
      resta=despues-ahora;
      deltaTimeD=meanFilterD.AddValue(deltaTimeD);
      deltaTimeI=meanFilterI.AddValue(deltaTimeI);
    }
}



 void motorSetup()
 {

   pinMode(pinIN1, OUTPUT);
   pinMode(pinIN2, OUTPUT);
   pinMode(pinENA, OUTPUT);
   pinMode(pinIN3, OUTPUT);
   pinMode(pinIN4, OUTPUT);
   pinMode(pinENB, OUTPUT);
}

void moveForward(const int pinMotor[3], int speed)
{
   digitalWrite(pinMotor[1], HIGH);
   digitalWrite(pinMotor[2], LOW);

   analogWrite(pinMotor[0], speed);
}
void moveBackward(const int pinMotor[3], int speed)
{
   digitalWrite(pinMotor[1], LOW);
   digitalWrite(pinMotor[2], HIGH);

   analogWrite(pinMotor[0], speed);
}
void fullStop(const int pinMotor[3])
{
   digitalWrite(pinMotor[1], LOW);
   digitalWrite(pinMotor[2], LOW);

   analogWrite(pinMotor[0], 0);
}
void moveWheel(int pwm,double w, const int pinMotor[3],bool back)
{
  
  if(pwm==0 || w==0){
    fullStop(pinMotor);
  }
  else{
    if(back){
      moveBackward(pinMotor,pwm);
    }
    else if(!back){
      moveForward(pinMotor,pwm);
    }
  }
  //se espera un tiempo antes de cambiar  PWM
  //no se usa delay opara evitar interferir con las interruociones.
  
}

void isrD()
{
  
  timeBeforeDebounceD=millis();//tiempo para evira rebotes
  deltaDebounceD=timeBeforeDebounceD-timeAfterDebounceD;// tiempo que ha pasdo entre interrupcion he interrupcion
  if(deltaDebounceD>TIMEDEBOUNCE){//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.
    startTimeD=micros();
    deltaTimeD=startTimeD-timeAfterD;
    encoder_countD++;
    timeAfterD=micros();
  }
  timeAfterDebounceD=millis();
  
}

void isrI()
{
    
    timeBeforeDebounceI=millis();//tiempo para evira rebotes
    deltaDebounceI=timeBeforeDebounceI-timeAfterDebounceI;// tiempo que ha pasdo entre interrupcion he interrupcion
    if(deltaDebounceI>TIMEDEBOUNCE){//condicion para evitar rebotes
      //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.
      startTimeI=micros();
      deltaTimeI=startTimeI-timeAfterI;
      encoder_countI++;//se cuenta los pasos de encoder
      timeAfterI=micros();
      
    }
    timeAfterDebounceI=millis();  
  
}
int pidD(double wD,double Setpoint)
{
  int output=0;
  currentTimeD = millis();                               // obtener el tiempo actual
  elapsedTimeD = (double)(currentTimeD - previousTimeD); // calcular el tiempo transcurrido
  if(elapsedTimeD>=k){//se asegura un tiempo de muestreo
    errorD = Setpoint - wD;
    if(errorD>=0.6|| errorD<(-0.6)){
        cumErrorD += errorD * elapsedTimeD;                      // calcular la integral del error
        if(lastErrorD>0 && errorD<0)cumErrorD=errorD;
        if(lastErrorD<0 && errorD>0)cumErrorD=errorD;
        if(cumErrorD>35000 || cumErrorD<-35000) cumErrorD=0;                         //se resetea el error acumulativo
        rateErrorD = (errorD - lastErrorD) / elapsedTimeD*1000;         // calcular la derivada del error
        
      output =(int) round(kp*errorD +Ki*cumErrorD + Kd*rateErrorD );     // calcular la salida del PID     0.0001*cumError  + Kd*rateErrorD
        lastErrorD = errorD;                                      // almacenar error anterior
    }
    previousTimeD = currentTimeD;                             // almacenar el tiempo anterior
  }
  return output;
}


int pidI(double wI,double Setpoint)
{
  int output=0;
  currentTimeI = millis();                               // obtener el tiempo actual
  elapsedTimeI = (double)(currentTimeI - previousTimeI);     // calcular el tiempo transcurrido
  if(elapsedTimeI>=k){//se asegura un tiempo de muestreo
    errorI = Setpoint - wI;   
 
                    
    if(errorI>=0.6|| errorI<-0.6){
      cumErrorI += errorI * elapsedTimeI;  //pasado un tiempo se tiene que borrrar cumerror                    // calcular la integral del error
   
      if(lastErrorI>0 && errorI<0){
        cumErrorI=errorI;
      }
      if(lastErrorI<0 && errorI>0){
        cumErrorI=errorI;
      }
      if(cumErrorI>35000||cumErrorI<-35000) cumErrorI=0;     //se resetea el error acumulativo 
      rateErrorI = (errorI - lastErrorI) /elapsedTimeI;         // calcular la derivada del error
    
      output = (int)round(kp*errorI  +Ki*cumErrorI + Kd*rateErrorI);     // calcular la salida del PID 0.0001*cumError  + Kd*rateErrorI
     
      lastErrorI = errorI; 

    }
    previousTimeI = currentTimeI;                             // almacenar el tiempo anterior
  }
 /* Serial.print("error");
        Serial.print(errorI);
        Serial.print(",");
        Serial.println(output);
        */
  return output;
  
}

//funcion que permite un control mejorado de las velocidades de las ruedas gracias al giroscopo
double ajusteGyroscope(double z)
{//Setpoint en rad/s
  double W=D/2*(setPointGWD-setPointGWI)/L;// se calcula la velocidad angular del centro de masas del robot
   z=z+ERR_GIROSCOPE;//se pone a cero la medida 
   if(z>-0.15 && z<0.20){
    z=0;
   }
   double error=W-z;

   return error;
   
}


void feedForwardD()
{
    
    PWM_D=round((setpointWD-8.658)/0.141);
    if(setpointWD==0){
      PWM_D=0;
    }
    if(PWM_D<76){
      strcpy(operation_send.data,"mensaje recibido, alerta pwm en discontinuidad");
      PWM_D=76;
    }
}
void feedForwardI()
{
    PWM_I=round((setpointWI-14.077)/0.112);
    if(setpointWI==0){
      PWM_I=0;
    }
    if(PWM_I<76){
      strcpy(operation_send.data,"mensaje recibido, alerta pwm en discontinuidad");
      PWM_I=76;
    }  
}

void tokenize(const string s, char c,vector<string>& v)//sirve para separa la entrada string.
{
   string::size_type i = 0;
   string::size_type j = s.find(c);

   while (j != string::npos) 
   {
      v.push_back(s.substr(i, j-i));
      i = ++j;
      j = s.find(c, j);

      if (j == string::npos)
         v.push_back(s.substr(i, s.length()));
   }
}
