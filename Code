/*
 * AQUAPHONIK SenseBox 4 (Each DS18B20 sensor has its own address provided in the script - see TEMPERATURE below)
 * V 0.2 - 01.08.2017
 * 
 * SENSORS:
 * DS18B20 Waterproof Temperature Sensor  - Digital
 * DIY-Conductivity Meter                 - Analog
 * pH-Electrode with pH-Adapter           - Analog
 * ORP-Electrode with pH/ORP-Adapter      - Analog
 * 
 * COMPONENTS: 
 * RTC                                    - I2C
 * SD-Card module                         - SPI
 * LCD-Display with POTI for contrast     - without SDA/SDL
 * SIM800L GSM (not yet implemented)      - UART 
 *  
 * INFRASTRUCTURE: Arduino Sensor Shield V 5.0, 9V-Battery Block, ON/OFF-Switch
 * 
 * Named PINOUTS: Important to know your MISO / MOSI / SCK / SDA / SCL Pins of yur board!
 * 
 * RTC (Real-Time Clock module)
 * DS3231+24C32 I2C 5V
 * SDA PIN - ANALOG 4
 * SCL PIN - ANALOG 5
 * <DS3231.h> library from http://www.rinkydinkelectronics.com/library.php?id=73
 * 
 * SD-CARD
 * Attached to SPI bus as follows: SPI = Serial Peripheral Interface with 5 logic signals:
 * SCLK: Serial Clock (output from master also called CLK).
 * MOSI: Master Output Slave Input, or Master Out Slave In (data output from master).
 * MISO: Master Input Slave Output, or Master In Slave Out (data output from slave).
 * SDIO: Serial Data I/O (bidirectional I/O)
 * SS: Slave Select / Chip select (often active low, output from master).
 * On the NANO board: MISO D12 / MOSI D11 / SCK D13 
 * CS DIGITAL 10 declared below (SD.begin())
 * You need a file called aqua.txt on your sd card to log the data!
 * Because this line of code: File dataFile = SD.open("aqua.txt", FILE_WRITE); 
 * 
 * TEMPERATURE
 * Dallas DS18B20
 * Include libraries Onewire and DallasTemperature
 * Conductivity goes to DIGITAL 3 (D3) for reading the sensor
 * IMPORTANT: Assign the addresses of each of your 1-Wire temp sensors!!
 * See the tutorial on how to obtain these addresses
 * Dallas sensor (eachone has a different one)
 * THE ONE I TESTED ITS CALLED: 0x28, 0xFF, 0xDD, 0xB0, 0x31, 0x17, 0x03, 0x7E
 * WIRING DALLAS! Power and ground OF THE SENSOR goes conected, BUT power OF THE SYSTEM goes after resitor 4k7
 * checkt he image: sensor-de-temperatura-ds18b20-1.png
 * 
 * CONDUCTIVITY Ec
 * The circuit: 
 * Sensor reading attached to analog input 3
 * Center pad of the sensor to the analog pin => A3
 * LEFT side pad to ground = GND = 0v
 * RIGHT side pad to Positive = +5V
 * 
 * pH-SENSOR
 * DFRobot Gravity: Analog pH Sensor/Meter Kit (DF-SEN0161)
 * Sensor reading attached to A0 
 *  
 * ORP-SENSOR
 * Phidgets ORP-Electrode (PHI-3555) with ph/ORP-Adapter (PHI-1130) from noDNA
 * Sensor reading attached to A1
 * 
 * LCD SCREEN 
 * We are not using the SDA/SCL adaptor, because of needed analog inputs (this can change now!)
 * So wiring goes this way:
 * Attach the pins written on the LCD to the pins on the Arduino: 
 * Vss-GND
 * VDD-5v
 * V0: to center pin of potentiometer for screen contrast
 * The other 2 legs to 5v and Gnd
 * Value of pot: 5k-1000k no worries its just a voltage divider
 * RS-D8 Digital 8 on Arduino
 * Rw-GND
 * E-D9 Digital 9 on Arduino
 * D0 to D3 on LCD not connected
 * D4-D5-D6-D7 on LCD Screen to D4-D5-D6-D7 on Arduino
 * Light on the SCREEN HAS 2 pin Anode(+) and Katode(-)
 * A= LED+ to 5v
 * K= GND
 * 
 * CONSUM
 * NANO CONSUMES WITH DATA LOGGER on low power (SD+RTC) 29,02mA
 * Power Bank (4x1,5v 600mA with ideally 600mA) 2400 mAh = +/-3 Days
 * Delete leds light and you will have much more time!
 * 
 * 2Do:
 * Optimization lowpower
 * Plot graphics with processing
 * Solar panel
 * GSM module
 * Make alarms for human action or actuators
 * 
 * FREE PINS:
 * A4/A5 SDA / SCL For I2C serial communication: LCD/RTC/ETC.
 * D2(PWM)
 * Visual or sound trought the PWM pins
 * 
 * THIS PROJECT WAS INSPIRED BY THE WORKS OF:
 * Björn Guntermann (Institut für Geoinformatik, Universität Münster)
 * 
 * THIS SCRIPT WAS DEVELOPED BY:
 * Víctor Mazón Gardoqui (Critical Media Space Leipzig). 07.17 CC 4.0-BY_SA
 * 
 * ALTERATION OF THE SKRIPT WAS CONDUCTED BY: 
 * Roman Gunold (machBar Potsdam / FabLab Leipzig)
 * Friedmann Eppelein (Science Shop Nürnberg)
 */

//Libraries
#include <LiquidCrystal.h>                //LCD library
#include <OneWire.h>                      //converts analog DS18B20 signal into digital one
#include <DallasTemperature.h>            //library for DS18B20 Temperature sensor
#include <SPI.h>                          //SPI library for SD card
#include <SD.h>                           //SD card library https://github.com/adafruit/SD
#include <DS3231.h>                       //http://github.com/JChristensen/DS3232RTC

// Init the DS3231 RTC module using the hardware interface
DS3231  rtc(SDA, SCL);                    //to set date & time choose Script "DS3231_Serial_Easy" from examples folder of <DS3231.h> library

// Measure the voltage of the Arduino (not exactly 5.0 V)
float readVcc()
{ long result; // Read 1.1V reference against AVcc 
ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); 
delay(2); // Wait for Vref to settle 
ADCSRA |= _BV(ADSC); // Convert 
while (bit_is_set(ADCSRA,ADSC));
result = ADCL;
result |= ADCH<<8;
result = 1126400L / result; // Back-calculate AVcc in mV 
return result; }
float Varduino;
float Vdigital;


//The following defines are needed for ph-Sensor
#define phSensor 14                       //pH-Sensor output on Analog Input A0
#define phOffset 0                     //deviation compensate
//#define systemVoltage=5.0       //for both pH and ORP
float ph;                                 //used to hold a floating point number that is the pH.
  static float pHvoltage; 
  
//The following defines are needed for orp-Sensor
#define orpSensor 15                      //orp-Sensor output on Analog Input A1
#define orpOffset 0                    //deviation compensate
float ORP;                                //used to hold a floating point number that is the ORP

//Temperature probe setup
float tempC;
float decPlaces;
int Ccc = (tempC,decPlaces);              //need to optimice this for printing temp on serial
#define ONE_WIRE_BUS 3                    // Data wire is plugged into Digital 3
OneWire oneWire(ONE_WIRE_BUS);            // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);      // Pass our oneWire reference to Dallas Temperature.
DeviceAddress insideThermometer = { 0x28, 0xFF, 0xDD, 0xB0, 0x31, 0x17, 0x03, 0x7E };  // VERY IMPORTANT ITS TO KNOW and ASIGN  
                                                                                       // the addresses of your 1-Wire temp sensors
                                                                                       // Use the sketch "Onewire_Sensor_Address_finder.ino"
// CONDUCTIVITY
//Quellspannung=5;             // We set up the reference for the sensor with 5v from the Arduino Board
int CondPin = 3;                      // The Analog pin we are using on this example its A3
int R1 = 150.0;                       // Wert des bekannten Widerstands (150 Ohm)
long Messwert;
float SpannungR2;                     // Spannung über dem zu messenden Widerstand
float Widerstand;
float Conductivity;
float Ec1000000;                      // Conductivity value *1000000 for the screen print, not used on the SD
char condString[10];
int cases = 1;
String output;

//LCD set up
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // select the pins used on the LCD panel

 
void setup()
{
	Serial.begin(38400);        //enable the hardware serial port
	rtc.begin();                //RTC startup
  
	sensors.begin();            //start up temp probe library
	sensors.setResolution(insideThermometer, 10);       // set the temp probe resolution to 10 bit
	lcd.begin(16, 2);           // start the lcd library
	lcd.print("Aquino/CubeSense");// display a text for welcome (16ch!)
	lcd.setCursor(0,1);                         
	lcd.print("  Version 0.2  ");
	delay(2000);
	lcd.setCursor(0,0);                         
	lcd.print("An aquaphonic   ");  // display a text for welcome (16ch!)
	lcd.setCursor(0,1);                         
	lcd.print(" Analysis System");
	delay(2000);
	lcd.clear();                      //clear screen!
	SD.begin(10);                     //SD card module CS on DIGITAL 10
	pinMode(10, OUTPUT);
  Serial.println("AQUAPHONICS - CUBESENSE - AQUINO - WHATEVER");
	Serial.println("DIY monitoring system for aquatic sampling based on Open Hardware");
	Serial.println(""); // Print blank line
  
//Column headers for Serial Monitor
  //String header = "Datum der\tUhrzeit\t\tTemperatur\tLeitfaehigkeit\tph-Wert\t\tRedoxpotential"; //column headers
  String header2 = "Probenahme\t(MEZ)\t\tT (C)\t\ts (uS/cm)\t(-log a(H3O+))\tE° (mV)\tUarduino (V)"; //column headers
	Serial.println("  ");
	//Serial.println(header);
  Serial.println(header2);
  Serial.println("  ");  							// Now print the columns on the loop
	delay(1000);

//Column headers for SD-Card text file
  File dataFile = SD.open("aqua.txt", FILE_WRITE);         // open the file
  if (dataFile) 
  {                                                        // if the file is available, write to it:
    dataFile.println("-->");                               //leading blank line to distinguish from previous data
    String SDheader = "Datum\tUhrzeit\tTemperatur\tLeitfähigkeit\tph-Wert\tRedoxpotential"; //column headers
    String SDheader2 = "Probenahme\t(MEZ)\tT (°C)\ts (µS/cm)\t(-log a(H3O+))\tE° (mV)\tUarduino (V)"; //column headers
    dataFile.println(SDheader);
    dataFile.println(SDheader2);
    dataFile.close();
}
} 

void loop() 
{
// Measure the voltage of the Arduino (not exactly 5.0 V)
Varduino = readVcc()/1000;
Vdigital = Varduino/1023;

//ph_Code from example (modified, as we do not need the delays etc.):  
 
//we actually don't need the average building	
	pHvoltage = analogRead(phSensor)*Vdigital;
  ph = 3.5*pHvoltage + phOffset;                           //formula from https://www.dfrobot.com/wiki/index.php/PH_meter(SKU:_SEN0161)


//Maybe we need averaging for pH and ORP values, but let's try it first without
	static float orpvoltage;  
	orpvoltage=analogRead(orpSensor)*Vdigital;
	ORP = (((2.5-orpvoltage)/1.037)*1000)+orpOffset;                //formula from https://www.phidgets.com/?tier=3&catid=11&pcid=9&prodid=103

//conductivity calculations for EC sensor
	Messwert = 0;
  for (int i = 0; i < 5; i++) 
  {
    Messwert += analogRead(CondPin);
  }
  Messwert = trunc(Messwert / 5);
  SpannungR2 = Vdigital * Messwert;                                //Spannung berechnen
  Widerstand = R1 * (SpannungR2 / (Varduino - SpannungR2));       //Berechnung: (R2 = R1 * (U2/U1))
  Conductivity = 1 / (Widerstand * 1.6);                          // Abweichung Abstand der Elektroden von 1 cm
  dtostrf(Conductivity, 8, 6, condString);                        // dtostrf(FLOAT,WIDTH,PRECSISION,BUFFER);
                                                                  // Check the precision of your sensor and adjust it here! 
  output += condString;
  output +=";";
  Ec1000000 = Conductivity * 1000000;
  
//Temperature sensor
	sensors.requestTemperatures();         	//read Temp probe          
	printTemperature(insideThermometer);	  //Here, ALL values are printed, so this needs to happen at the end
}


void printTemperature(DeviceAddress deviceAddress)
{
	int decPlaces = 2;             // set temp decimal places to 2
	float tempC = sensors.getTempC(deviceAddress);
	if (tempC == -127.00) 
	{
		lcd.print("Error getting temperature");
	}
	else 
	{
		lcd.setCursor(0,1);          //set position on lcd for pH
		lcd.print("pH:");
		lcd.print(ph, 2);            //send pH to lcd
		lcd.setCursor(7,1);          //set position on lcd for ORP
		lcd.print(" E");
		lcd.print(":");
		lcd.print(ORP, 0);           //send ORP to lcd
		lcd.print("mV");
		lcd.setCursor(0,0);          //set position on lcd for Temp
		//lcd.print("T:");
		lcd.print(tempC,decPlaces);  //display Temp in celsius
		lcd.print((char)223);        // ASCI degree 
		lcd.print("C");              // C character
    lcd.setCursor(7,0);
		lcd.print(" s:");         // Electrical conductivity *1000 to be displayed on screen
		lcd.print(Ec1000000);
    lcd.print((char)228);        // ASCI micro sign 
		lcd.print("S");  
		delay(1000);                 //Take a reading every 1000ms 
								  //HERE YOU CAN CHANGE THE TIME OF THE READINGS
    
	}

	//long currentTime = millis();                            // Get the current time in ms (time since program start)
	File dataFile = SD.open("aqua.txt", FILE_WRITE);        // open the file
	if (dataFile) 
	{                                                       // if the file is available, write to it:
  	//dataFile.print(currentTime);                          // logs the time in milliseconds since the program started
    dataFile.print(rtc.getDateStr());       
    dataFile.print("\t");                                 // prints a tab
    dataFile.print(rtc.getTimeStr());       
    dataFile.print("\t");                                 // inserts a tab
  	dataFile.print(tempC,decPlaces);                      // logs the temperature in degrees C
    dataFile.print("\t");                                 // inserts a tab
    dataFile.print(Ec1000000);                            // logs Ec
    dataFile.print("\t");                                 // inserts a tab
		dataFile.print(ph);                                   // logs the pH
		dataFile.print("\t");                                 // inserts a tab
		dataFile.print(ORP);                                // logs the ORP
    dataFile.print("\t");                                 // inserts a tab
    dataFile.print(Varduino);                             // logs the arduino voltage (readVcc)
		dataFile.close();
	}
// if the file isn't open, pop up an error if needed??
    else {
      //Serial.println("Kann die Datei aqua.txt nicht finden. Steck mal ne SD-Karte rein, oder willste die Werte abschreiben?");
  }
  
// Serial printing
      //Serial.print("Current Time from start up ");
      //Serial.println(currentTime);                  
      Serial.print(rtc.getDateStr());       
      Serial.print("\t");    // prints a tab
      Serial.print(rtc.getTimeStr());       
      Serial.print("\t");    // prints a tab
      Serial.print(tempC,decPlaces);       
      Serial.print("\t");    // prints a tab
      Serial.print("\t");    // prints a tab
      Serial.print(Ec1000000);       
      Serial.print("\t");    // prints a tab
      Serial.print("\t");    // prints a tab
      Serial.print(ph);       
      Serial.print("\t");    // prints a tab
      Serial.print("\t");    // prints a tab
      Serial.print(ORP);       
      Serial.print("\t");    // prints a tab
      Serial.print("\t");    // prints a tab
      Serial.print(Varduino);
      Serial.println("  ");
      delay(1000);
              
}
