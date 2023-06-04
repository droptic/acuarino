/******************************************************************************************
*            Controlador acuarino v 0.2.4            
*                  por droptic                                             
*
*  Release: 29/09/2011
*  FUNCIONES
*  -MUESTRA LA INFORMACION EN UNA PANTALLA LCD 16X2 CONECTADA VIA I2C
*  -SENSA LA TEMPERATURA CON SENSOR DE TEMPERATURA 18B20 CONECTADO AL PIN D2
*  -MODULO RELOJ EN TIEMPO REAL DS1307 CONECTADO AL A4-A5
*  -RELAYs CON CAPACIDAD DE 4 RELAYS USANDO D3-D4-D5-D6 (PERMITE CONTROLAR APARATOS 220 VOLTS)
*  -MEDIANTE RELAY CONTROLA ENCENDIDO Y APAGADO DE LUCES DE LAS 7:00 AM A LAS 18:00 PM
*  -MEDIANTE RELAY CONTROLA ENCENDIDO DE CALEFACTOR CUANDO TEMPERATURA BAJA DE 25 GRADOS
*
*  POR HACER:
*  -INSTALAR MODULO BNC + SONDA DE pH <------------- iniciado
*  -PROGRAMAR CALENDARIO PARA MOONLIGHT VERDADERO (O SEA DE ACUERDO AL CICLO LUNAR) <-------------- LISTO
*  -PROGRAMAR EL 2A MOTOR SHIELD PARA CONTROL DE BOMBA PERISTALTICA <--------------INICIADO
*  -PROGRAMAR SENSOR DE FLUJO EN FILTRO PARA ESTIMACION DE LIMPIEZA DEL MISMO <----------------- INICIADO
*  -MEJORAR LA PANTALLA
*  -AGREGAR MODULO INFRAROJO PARA CONTROL DE FUNCIONES VIA CONTROL REMOTO
*  -AGREGAR FUNCION DE ENFRIAMIENTO DE ACUARIO (CON VENTILADOR) CUANDO TEMP MAYOR A 26 GRADOS
*  -AGREGA FUNCION DE REGISTRO DE DATOS <------------------  INICIADO
*
*  LARGO PLAZO
*  -AGREGAR FUNCIONES PARA REEF
*  -AGREGAR MENU (POSIBLE LIMITACION CODIGO POR TAMAÑO > 32 KB) ¿ARDUINO MEGA?¿Extra EEPROM?
*  
*                                http://code.google.com/p/acuarino
**************************************************************************************************************/

///////////////////////////////  LIBRERIAS ///////////////////////////////////////////

#include <Wire.h>
#include <TM1638.h>
#include <TM1638Fonts.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <DS1307.h>
#include <TimeLord.h>
#include <SD.h>
#include <AikoEvents.h>

/////////////////////////// Ajustes Basicos //////////////////////////////////////

int luz_on = 7;   //   Hora Encendido luces (Formato 1-24)
int luz_off = 18;  //   Hora Apagado Luces (Formato 1-24)
int tiemporegistro = 1; // Tiempo de registro en minutos en tarjeta SD
int tiempoperistaltica = 60; // Tiempo en minutos cada cuanto funciona bomba peristaltica 
int ml_solucion = 1; // cuanto mL de solucion agrega por tiempo anterior

///////////////////////// Ajuste tiempo minutos a milisegundos///////////////////////////////////
using namespace Aiko;
int tiemporegistroseg = (tiemporegistro * 60000); //Convierte minutos a milisegundos
int tiempoperistalticaseg = (tiempoperistaltica * 60000); //Convierte minutos a milisegundos

////////////////////////////// Ajusta Pantalla LCD ///////////////////////////////////

LiquidCrystal_I2C lcd(0x27,16,2);  // Ajusta la pantalla LCD a la direccion 0x27 con 16 caracteres y 2 lineas


//////////////////////////// Funcion peristaltica ///////////////////////////////////
//24 ML X MIN  ////////////////
//11.69 VOLTS  /   value=255  /
//650 mAmp     ////////////////
//0,4 ml = 1 seg
//1 ml = 2,5 seg
//Arduino PWM Speed Control：

int E1 = 6;   
int M1 = 7;
int ml_sol = ml_solucion * 2500;

void peristaltica() 
{ 
  int value = 255;

  { 
    digitalWrite(M1,HIGH);   

    analogWrite(E1, value);   //PWM Speed Control

    delay (ml_sol); // Agrega un ml de solucion X
    
  }  
 
  int value2=0;
   { 
    digitalWrite(M1,HIGH);   

    analogWrite(E1, value2);   //PWM Speed Control

    delay (60000); // cada X Horas
    
  }  
}


///////////////////////////// AJUSTA TARJETA sd ///////////////////////////////
//SD card datalogger
//
// The circuit:
// * SD card attached to SPI bus as follows:
// ** MOSI - pin 11
// ** MISO - pin 12
// ** CLK - pin 13
// ** CS - pin 8
// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.

const int chipSelect = 8;


///////////////////////////// Ajusta Sensor temperatura //////////////////////////////


#define ONE_WIRE_BUS 2 // Sensor temperatura conectado a puerto digital 2
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
DeviceAddress insideThermometer; // arrays to hold device address

//////////////////////////// Ajuste sensor de flujo /////////////////////////////////

volatile int NbTopsFan; //measuring the rising edges of the signal
int Calc;                              
int sensorflujo = 3;    //Sensor de Flujo conectado a D3

void rpm ()     //This is the function that the interupt calls
{
  NbTopsFan++;  //This function measures the rising and falling edge of the hall effect sensors signal
}

void medicionflujo ()
{
  NbTopsFan = 0;   //Set NbTops to 0 ready for calculations
  sei();      //Enables interrupts
  delay(1000);   //Wait 1 second
  cli();      //Disable interrupts <--- apaga tambien la conexion serial con la pantalla LCD
  Calc = (NbTopsFan * 60 / 7.5); //(Pulse frequency x 60) / 7.5Q, = flow rate in L/hour
  sei();      // reinicia la conexion serial con LCD
  lcd.setCursor(7,0);
  lcd.print("    ");
  lcd.setCursor(7,0);
  lcd.print(Calc); //Prints the number calculated above
  lcd.setCursor(12,0);
  lcd.print("L/hr"); //Prints "L/hour" 
}

/////////////////////////////// Funcion Sensores temperatura ////////////////////////

void sensortemp1()
{
  sensors.requestTemperatures(); // Envia comando para obtener temperatura
  lcd.setCursor(0,1);
  lcd.print(sensors.getTempCByIndex(0),1);
  lcd.print((char)223);
  lcd.print("C");
}

//////////////////////////////   AJUSTE SENSOR DE pH //////////////////////////////
// change this to whatever pin you've moved the jumper to

  int ph_pin = 1;
  //int for the averaged readin
  int reading;
  //int for conversion to millivolts
  int millivolts;
  //float for the ph value
  float ph_value;
  int h;
   
  // highly recommended that you hook everything up and check the arduino's voltage with a multimeter. It doesn't make that much of a difference, but
  // if you want it to be highly accurate than do this step
   
  #define ARDUINO_VOLTAGE 5.01
  // PH_GAIN is (4000mv / (59.2 * 7))
  // 4000mv is max output and 59.2 * 7 is the maximum range (in millivolts) for the ph probe.
  #define PH_GAIN 9.6525 

void sonda_pH ()
{
  //take a sample of 50 readings
  reading = 0;
  for(h = 1; h < 50; h++){
    reading += analogRead(ph_pin);
    delay(10);
  }
  //average it out
  reading /= h;
 
  //convert to millivolts. remember for higher accuracy measure your arduino's voltage with a multimeter and change ARDUINO_VOLTAGE
  millivolts = ((reading * ARDUINO_VOLTAGE) / 1024) * 1000;
 
  ph_value = ((millivolts / PH_GAIN) / 59.2) + 7;
  lcd.setCursor(7,1);
  lcd.print("pH=");
  lcd.print(ph_value,1);
  
}



//////////////////////////////  Configura el reloj /////////////////////////////////

#define DS1307_I2C_ADDRESS 0x68
byte second, minute, hour, dayOfMonth, month, year; // variables para la hora
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}
// 1) Sets the date and time on the ds1307
// 2) Starts the clock
// 3) Sets hour mode to 24 hour clock
// Assumes you're passing in valid numbers
void setDateDs1307(byte second,        // 0-59
byte minute,        // 0-59
byte hour,          // 1-23
byte dayOfMonth,    // 1-28/29/30/31
byte month,         // 1-12
byte year)          // 0-99
{
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.send(0);
  Wire.send(decToBcd(second));    // 0 to bit 7 starts the clock
  Wire.send(decToBcd(minute));
  Wire.send(decToBcd(hour));      // If you want 12 hour am/pm you need to set
  // bit 6 (also need to change readDateDs1307)
  Wire.send(decToBcd(dayOfMonth));
  Wire.send(decToBcd(month));
  Wire.send(decToBcd(year));
  Wire.endTransmission();
}
// Obtiene la fecha y hora desde ds1307 (reloj tiempo real)
void getDateDs1307(byte *second,
byte *minute,
byte *hour,
byte *dayOfMonth,
byte *month,
byte *year)
{
  // Resetea el punto de registro
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.send(0);
  Wire.endTransmission();
  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);
  // A few of these need masks because certain bits are control bits
  *second     = bcdToDec(Wire.receive() & 0x7f);
  *minute     = bcdToDec(Wire.receive());
  *hour       = bcdToDec(Wire.receive() & 0x3f);  // Permite cambiar a 12 horas am/pm
  *dayOfMonth = bcdToDec(Wire.receive());
  *month      = bcdToDec(Wire.receive());
  *year       = bcdToDec(Wire.receive());
}

///////////////////////////////////// FUNCION HORA ///////////////////////////////////////////

void onesecond() //Funcion que muestra la hora en la posicion (0,0)
{
  byte second, minute, hour, dayOfMonth, month, year;
  getDateDs1307(&second, &minute, &hour, &dayOfMonth, &month, &year);
  lcd.setCursor(0, 0);
  if (hour<10)
    {
    lcd.print("0");
    lcd.print(hour,DEC);
    }
  else
    {
    lcd.print(hour, DEC);
    }
  lcd.print(":");
  if (minute<10)
    {
    lcd.print("0");
    lcd.print(minute,DEC);
    }
  else
    {
    lcd.print(minute, DEC);
    }
}

////////////////////////////////// Funcion Encendido de luces ////////////////////////////

void luces()
{
  byte second, minute, hour, dayOfMonth, month, year;
  getDateDs1307(&second, &minute, &hour, &dayOfMonth, &month, &year);
  byte relayPin4 = 4;
   // Incializa los pines de los relays
  {
  pinMode(relayPin4,OUTPUT);
  }

//D3 -> COM1  <----------------- Digital 4  LUCES
//D4 -> COM2  <----------------- Digital 5  CALEFACTOR
//D5 -> COM3  <----------------- Digital 9  MOONLIGHT
//D6 -> COM4  <----------------- Digital 10
//cambiar esto si falta puerto libres
//Para recordar sobre el relay shield
//NC Normally Closed - NC1 is connected with COM1 when control port D2 is set low and disconnected when D2 is set high
//NO Normally Open - NO1 is disconnected from COM1 when control port D2 is set low and connected when D2 is set high

  if (hour >= luz_on) // Ajusta que las luces se enciendan
  {
    digitalWrite(relayPin4,HIGH);
    lcd.setCursor(15, 1);
    lcd.write(1);
  }
  else if (hour >= luz_off) // Ajusta que las luces se apaguen
  {
    digitalWrite(relayPin4,LOW);
    lcd.setCursor(15, 1);
    lcd.write(2);
  }
  else
  {
  }
}

////////////////////////////////// Funcion calefactor segun temperatura ////////////////////////////

void calefactor()
{
  byte relayPin5 = 5;
   // Incializa los pines de los relays
  {
  pinMode(relayPin5,OUTPUT);
  }

//D3 -> COM1  <----------------- Digital 4  LUCES
//D4 -> COM2  <----------------- Digital 5  CALEFACTOR
//D5 -> COM3  <----------------- Digital 9  MOONLIGHT
//D6 -> COM4  <----------------- Digital 10
//cambiar esto si falta puerto libres
//Para recordar sobre el relay shield
//NC Normally Closed - NC1 is connected with COM1 when control port D2 is set low and disconnected when D2 is set high
//NO Normally Open - NO1 is disconnected from COM1 when control port D2 is set low and connected when D2 is set high
  sensors.requestTemperatures();
 
  if ((sensors.getTempCByIndex(0))<=25) // Ajusta que la temperatura a 25ºC
  {
    digitalWrite(relayPin5,HIGH);
    lcd.setCursor(14, 1);
    lcd.write(0);
  }
  else
  {
    digitalWrite(relayPin5,LOW);
    lcd.setCursor(14, 1);
    lcd.write(3);
  }
}

///////////////////////////////////// MOONLIGHT ////////////////////////


void moon ()
{
/*
TimeLord::MoonPhase(byte date_time[6]);
This subroutine returns the phase of the moon on the specified date.
The returned value ranges from 0 to (almost) 1
0.0      New moon
0.25     First Quarter
0.5      Full Moon
0.75     Last Quarter
0.99     Almost new

*/  
 
  byte second, minute, hour, dayOfMonth, month, year;
  getDateDs1307(&second, &minute, &hour, &dayOfMonth, &month, &year);
  TimeLord moonlight;
  byte ahora[] = {second, minute, hour, dayOfMonth, month, year};
  float fase;
  fase=moonlight.MoonPhase(ahora);
  byte relayPin9 = 9;
   // Incializa los pines de los relays
  {
  pinMode(relayPin9,OUTPUT);
  }
//D3 -> COM1  <----------------- Digital 4  LUCES
//D4 -> COM2  <----------------- Digital 5  CALEFACTOR
//D5 -> COM3  <----------------- Digital 9  MOONLIGHT
//D6 -> COM4  <----------------- Digital 10
//cambiar esto si falta puerto libres
//Para recordar sobre el relay shield
//NC Normally Closed - NC1 is connected with COM1 when control port D2 is set low and disconnected when D2 is set high
//NO Normally Open - NO1 is disconnected from COM1 when control port D2 is set low and connected when D2 is set high
 
  if (fase>=0.4 && fase<=0.6) // Ajusta que el moonlight se encieda  casi cuando hay luna llena
  {
    int pinrelayluz = 3;
    int prl;
    prl = digitalRead(pinrelayluz);
    if (prl = LOW)           // permite prender el moonlight solo cuando la luz rpincipal este apagada
      {
      digitalWrite(relayPin9,HIGH);
      lcd.setCursor(13, 1);
      lcd.write(4);
      }
    else
      {
       digitalWrite(relayPin9,LOW);
       lcd.setCursor(13, 1);
       lcd.write(5);
      }  
  }
  else
  {
    digitalWrite(relayPin9,LOW);
    lcd.setCursor(13, 1);
    lcd.write(5);
  }
}

///////////////////////////////////// CARACTERES ESPECIALES ////////////////////////

byte newChar[8] = {    //temperatura en alza o para calefactor encendido
    B00100,
    B01110,
    B10101,
    B00100,
    B00000,
    B01110,
    B00100,
    B00100
};
byte newChar1[8] = {    //Luces Encendidas
    B10101,
    B01110,
    B11011,
    B01110,
    B10101,
    B01000,
    B01000,
    B01110
};

byte newChar2[8] = {    //Luces Apagadas
    B00000,
    B01110,
    B01010,
    B01110,
    B00000,
    B01000,
    B01000,
    B01110
};
byte newChar3[8] = {  //Temperatura OK
    B00001,
    B00010,
    B10100,
    B01000,
    B00000,
    B01110,
    B00100,
    B00100
};
byte newChar4[8] = {    //Luz de Luna encendido
    B00100,
    B01110,
    B11111,
    B01110,
    B00100,
    B10010,
    B10010,
    B11011
};
byte newChar5[8] = {    //Luz de Luna apagado
    B00000,
    B00100,
    B01110,
    B00100,
    B00000,
    B10010,
    B10010,
    B11011
};


///////////////////////////////////////// FUNCION DATALOGGER  ///////////////////////////////////////

void datalogger()
{
  byte second, minute, hour, dayOfMonth, month, year;
  getDateDs1307(&second, &minute, &hour, &dayOfMonth, &month, &year);
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("registro.csv", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
   // byte second, minute, hour, dayOfMonth, month, year;
   // getDateDs1307(&second, &minute, &hour, &dayOfMonth, &month, &year);
    if (dayOfMonth<10)
      {
      dataFile.print("0");
      dataFile.print(dayOfMonth,DEC);
      }
    else
      {
      dataFile.print(dayOfMonth, DEC);
      }
    dataFile.print("/");
    if (month<10)
      {
      dataFile.print("0");
      dataFile.print(month,DEC);
      }
    else
      {
      dataFile.print(month, DEC);
      }    
    dataFile.print(";");
    if (hour<10)
      {
      dataFile.print("0");
      dataFile.print(hour,DEC);
      }
    else
      {
      dataFile.print(hour, DEC);
      }
    dataFile.print(":");
    if (minute<10)
      {
      dataFile.print("0");
      dataFile.print(minute,DEC);
      }
    else
      {
      dataFile.print(minute, DEC);
      }    
    
    dataFile.print(";");
    dataFile.print(sensors.getTempCByIndex(0));
    dataFile.print(";");
    dataFile.print(ph_value);
    dataFile.print(";");
    dataFile.println(Calc,DEC);
    dataFile.close();
    
  }  
  // if the file isn't open, pop up an error:
  else {
    lcd.clear();
    lcd.print("error abriendo registro.csv");
    delay(4000);
  }
  
 
}


///////////////////////////////////////// SETUP  ///////////////////////////////////////

void setup() {                
  // IniciaLCD
  lcd.init();
  // Activa luz fondo LCD
  lcd.backlight();
  //Inicia Mensaje Bienvenida
  lcd.print(" Acuarino v0.2.4");
  //Escribe en la segunda linea LCD y se mantiene 2 segundos
  lcd.setCursor(0, 1);
  lcd.print("  por droptic");
  delay(4000);
  //Crea simbolos
  lcd.createChar(0, newChar);
  lcd.createChar(1, newChar1);
  lcd.createChar(2, newChar2);
  lcd.createChar(3, newChar3);
  lcd.createChar(4, newChar4);
  lcd.createChar(5, newChar5);
  lcd.clear();
 
 // Inicia SD
  pinMode(10, OUTPUT);
  if (!SD.begin(chipSelect)) {
    lcd.print("Problema SD");
    // don't do anything more:
    return;
  }
  lcd.print("SD... OK !!!");
  delay(1000);
  lcd.clear();

  Events.addHandler(datalogger,tiemporegistroseg);
 //Inicia Sensor flujo
  pinMode(sensorflujo, INPUT); //initializes digital pin 3 as an input
  attachInterrupt(1, rpm, RISING); //and the interrupt is attached
 
  Events.addHandler(peristaltica,tiempoperistalticaseg);
 //Inicia bomba persitaltica conectada a 2A Motor Shield (DFRobot)
  pinMode(M1, OUTPUT);   
 
}

///////////////////////////////////////// LOOP  ///////////////////////////////////////

void loop() {

//Muestra la Hora en la posicion (0,0)
  onesecond();

//Temperatura 
  sensortemp1();

// Medicion pH
  sonda_pH ();
 
// Sensor de flujo
  medicionflujo();

// Activa el sistema de luces
  luces();

// Activa el sistema de control de temperatura
  calefactor();
 
// activa moonlight
  moon();

// activa registro de datos en SD
  Events.loop();
 
   

}
