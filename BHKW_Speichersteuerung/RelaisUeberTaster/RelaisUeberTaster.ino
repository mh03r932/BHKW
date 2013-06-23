// BHKW Steuerung 24. Juni 2013
// one wire bus 
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>

// Data wire is plugged into port 8 on the Arduino
#define ONE_WIRE_BUS 4
#define TEMPERATURE_PRECISION 9
#define BOUNCE_DURATION 40 

#define IN_1 2 //input 1 und 2 bestimmen modus
#define IN_2 3 //input in Port 3
#define RELAIS 12 //Relais in Port 12
#define RELAIS2 11 //Relais2 in Port 11
#define RELAIS3 10 //Relais3 in Port 10
#define RELAIS4 9 //Relais4 in Port 9
#define RELAIS5 8 //Relais 5 in Port 8
#define RELAIS6 7 //Relais 6 in Port 7
#define ON_OFF_LIGHT 13 //Ein/Aus Anzeige in Port 13

#define RESERVE 5 //Reservekabel in Port 5

volatile unsigned long bounceTime=0; // variable to hold ms count to debounce a pressed switch
/*Steuerung hat 4 Zustaende 
wenn x1 high x2 high -> Restwaerme abfuehren
wenn x1 low  x2 high -> vorwaermen
wenn x1 high x2 low -> betrieb
wenn x1 low  x2 low -> alles aus */

// Setup einen OneWire Bus fuer die Kommunikation mit den Temperaturfuehlern
OneWire oneWire(ONE_WIRE_BUS);
// Die DallasTemparature Lib uebernimtm das handling des one wire bus
DallasTemperature sensors(&oneWire);
const long TEMPR_READ_INTERVAL = 6000; // gibt an wie haeufig Temperatur gelesen wird

// arrays mit den Adressen der Temperaturfuehler 28D3AF3304000081
DeviceAddress temp1 = { 0x28, 0x81, 0x91, 0x33, 0x4, 0x0, 0x0, 0xE7 };
DeviceAddress temp2 = { 0x28, 0xD3, 0xAF, 0x33, 0x4, 0x0, 0x0, 0x81 };
DeviceAddress temp3 = { 0x28, 0xD2, 0xFF, 0x33, 0x4, 0x0, 0x0, 0x23 };
DeviceAddress temp4 = { 0x28, 0x89, 0x7A, 0x33, 0x4, 0x0, 0x0, 0xEC };
double temp1Val = 0,temp2Val = 0,temp3Val =0,temp4Val = 0;

unsigned long lastTempReadTime = 0;  // Wann die Temperatur zuletzt gelesen wurde
const double vorwaermenTemp = 40; //Temperatur bei dem das Relais fuer das vorwaermen der Maschine ausgeschaltet wird
const double abgasWtEinTemp = 35; //Temperatur von der an die Pumpe des Abgaswaermetauschers lauft
double abgastWtPumpStartedTime = -1; //Zeit zu der die Abgaswaermetauschpumpe beim abstellen gestartet wurde, -1 bedeutet das noch keine Zeit gesetzt wurde
LiquidCrystal lcd(A0, A1, A4, A5, A6, A7);

/**
* Initialisierung des Programms (Anschluesse, Sensonren etc ..)
*/
void setup() {
  pinMode(IN_1,INPUT);
  pinMode(IN_2,INPUT);
  pinMode(RELAIS,OUTPUT);
  pinMode(RELAIS2,OUTPUT);
  pinMode(RELAIS3,OUTPUT);
  pinMode(RELAIS4,OUTPUT);
  pinMode(RELAIS5, OUTPUT);
  pinMode(RELAIS6,OUTPUT);
  pinMode(13, OUTPUT); //fuer ein/aus laempchen
  digitalWrite(RELAIS, HIGH );
  digitalWrite(RELAIS2, HIGH );
  digitalWrite(RELAIS3, HIGH );
  digitalWrite(RELAIS4, HIGH );
  digitalWrite(RELAIS5,HIGH);
  digitalWrite(RELAIS6,HIGH);
  digitalWrite(ON_OFF_LIGHT,HIGH);
  
    pinMode(A0,OUTPUT);
    pinMode(A1,OUTPUT);
    pinMode(A4,OUTPUT);
    pinMode(A5,OUTPUT);
    pinMode(A6,OUTPUT);
    pinMode(A7,OUTPUT);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16,2);
  
  
  //Gib Daten der Temperaturfuehler aus
   // start serial port
  Serial.begin(9600);
  Serial.println("Blockheizkraftwerksteuerung Daniel Hofstetter");

  // Start Temperaturfuelherbibliothek
  sensors.begin();

  // locate devices on the bus
  Serial.print("Suche Fuehler...");
  Serial.print("Es wurden  ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" Temperaturfuehler gefunden.");
  if(!sensors.isConnected(temp1)) {Serial.println("FEHLER Konnte Adresse fuer  temp1 nicht finden"); digitalWrite(RELAIS6,LOW);} 
  if(!sensors.isConnected(temp2)) {Serial.println("FEHLER Konnte Adresse fuer  temp2 nicht finden"); digitalWrite(RELAIS6,LOW);} 
  if(!sensors.isConnected(temp3)) {Serial.println("FEHLER Konnte Adresse fuer  temp3 nicht finden"); digitalWrite(RELAIS6,LOW);} 
  if(!sensors.isConnected(temp4)) {Serial.println("FEHLER Konnte Adresse fuer  temp4 nicht finden"); digitalWrite(RELAIS6,LOW);} 
  
  
    // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  
  // Aufloesung der Temperatursensoren auf 9 bit einstellen
  sensors.setResolution(temp1, TEMPERATURE_PRECISION);
  sensors.setResolution(temp2, TEMPERATURE_PRECISION);
  sensors.setResolution(temp3, TEMPERATURE_PRECISION);
  sensors.setResolution(temp4, TEMPERATURE_PRECISION);
  
  
}
/*
* Schlaufe welche dauernd wiederholt wird
*/
void loop() {
  readTemperatures();
  handleNotstop();  
  if(!isEverythingOff()){
    handleVorwaermen(); //vorwaermen handhaben
    handleAbgaswaermetauscher1();
    handleWaermetransportSpeicher();
  }else{
    //kein betrieb -> lampe aus
    digitalWrite(ON_OFF_LIGHT,HIGH);
    digitalWrite(RELAIS, HIGH );
    digitalWrite(RELAIS2, HIGH );
    digitalWrite(RELAIS3,HIGH);
    digitalWrite(RELAIS4,HIGH);
    digitalWrite(RELAIS5,HIGH);
        
  }

}
boolean isEverythingOff(){
   return (digitalRead(IN_1) == LOW && digitalRead(IN_2) == LOW);  
}

/* Startet die Vorwaermpumpe wenn die Temperatur noch tief ist
*/
void handleVorwaermen(){
  if( isVorwaermenActive && temp1Val < vorwaermenTemp ){
    digitalWrite(RELAIS, LOW ); //Ausgang einschalten
    Serial.print("Vorwaermpumpe ein (tmp1 = ");
    Serial.println(temp1Val); 
  }else{
     digitalWrite(RELAIS, HIGH );
  }
}

boolean isVorwaermenActive(){
  return (digitalRead(IN_1) == LOW && digitalRead(IN_2) == HIGH);
}

/*
* handhabt die Steuerung der Pumpe des Abgaswarmetauscher 1
*/
void   handleAbgaswaermetauscher1(){
  double temperature3 = temp3Val;//lies temperatur des fuehelrs 3
  if(isBetriebActive()){
    abgastWtPumpStartedTime = -1;
     if(temperature3 > abgasWtEinTemp){
       digitalWrite(RELAIS2, LOW ); //Ausgang einschalten
  }  else{
     digitalWrite (RELAIS2,HIGH);
    }
  }
  else if(isRestwaermeAbfuhr() == false){
     double temperature1 =  temp1Val;
     if(temperature3 > temperature1){
           //startzeitpunkt zu dem wtPumpe 1 eingeschaltet wurde
            if(abgastWtPumpStartedTime == -1){
               abgastWtPumpStartedTime = millis(); 
            }
            if(abs((millis() - abgastWtPumpStartedTime) < (1000* 60 * 6))) {
                digitalWrite(RELAIS2, LOW ); //Ausgang einschalten   
            } else{
              digitalWrite (RELAIS2,HIGH);   //nach 6 Minuten ausschalten  
            }
              
      } else{ //temparaturdifferenzbedingung ist nicht erfuellt
           digitalWrite (RELAIS2,HIGH);     
     }  
  }
  else{
    //nichts trifft zu
   digitalWrite(RELAIS2,HIGH); 
  }
}

/**
* handhabt den Waermetransport zum Speicher in 3 Stufen
*/
void handleWaermetransportSpeicher(){
    double temperature4 = temp4Val;//lies temperatur des fuehelrs 4  
    double temperature3 = temp3Val;//lies temperatur des fuehelrs 3
    double temperature2 = temp2Val;//lies temperatur des fuehelrs 2
  if(isBetriebActive() && temperature3 >= 78 && temperature3 < 83){
     activateExclusive(RELAIS3); //exklusiv einschalten
  } else if(isBetriebActive() && temperature3 >= 83 && temperature3 < 85){
      activateExclusive(RELAIS4);//4ein alle anderen aus
  } else if(isBetriebActive() && temperature3 >= 85){
       activateExclusive(RELAIS5); //5 ein andere aus
  }else if( isBetriebActive()  && temperature2 >= 105){
       activateExclusive(RELAIS3);//temp3 erfordert keine aktion. gaskuehler abfragen
  } else if(isRestwaermeAbfuhr() && temperature3 > 40  && (temperature3 - temperature4) > 8){ 
       activateExclusive(RELAIS5); //5 ein andere aus
  }
  else{
    //nichts traf zu -> Alles ausschalten
    Serial.println("Keine der Speicherpumpenbedingungen traf zu. Speicherpumpe aus");
      digitalWrite(RELAIS3,HIGH);
      digitalWrite(RELAIS4,HIGH);
      digitalWrite(RELAIS5,HIGH);
  }
}

/* Wenn IN_1 High und  IN_2 low ist dann ist betrieb*/
boolean isBetriebActive(){
  if(digitalRead(IN_1 == HIGH ) && digitalRead(IN_2) == LOW){
      digitalWrite(ON_OFF_LIGHT,LOW);
      return true;
  } 
  return false;
  
}

boolean isRestwaermeAbfuhr(){
  return(digitalRead(IN_1) == HIGH && digitalRead(IN_2) == HIGH);
}
/**
* handhabt den Notstop wenn uebertemperatur vorhanden ist
*/
void handleNotstop(){
    //uebertemperaturtest
    if (temp1Val >= 105 || temp2Val >= 110 || temp3Val >= 112 ){
      digitalWrite(RELAIS6,LOW);//Notstop einschalten
    }
}

void activateExclusive(int pinNumber){
  if(pinNumber != RELAIS3){
    digitalWrite(RELAIS3, HIGH);  
  }
  if(pinNumber != RELAIS4){
    digitalWrite(RELAIS4,HIGH);
  }
  if(pinNumber != RELAIS5){
    digitalWrite(RELAIS5,HIGH);
  }
  if(pinNumber != RELAIS3 && pinNumber != RELAIS4 && pinNumber != RELAIS5 ){
    Serial.println("KRITISCHER FEHLER Pinnummer zum aktivieren muss einer der vorgaben entsprechen");
  } else{
    digitalWrite(pinNumber,LOW) ;//einschalten
    
  }
}


void readTemperatures(){
    unsigned long currentMillis = millis();
  if(abs(currentMillis - lastTempReadTime) > TEMPR_READ_INTERVAL){
    // call sensors.requestTemperatures() to issue a global temperature 
    // request to all devices on the bus
    Serial.print("Frage Temperaturfuehler ab... ");
    sensors.requestTemperatures();
    Serial.println("FERTIG");
    double newTemp1 = sensors.getTempC(temp1);
    double newTemp2 = sensors.getTempC(temp2);
    double newTemp3 = sensors.getTempC(temp3);
    double newTemp4 = sensors.getTempC(temp4);
    //if the value that was read equals -127 then there was a problem reading it. ignore value and hope the next read will correct the problem

 if(newTemp1 != -127){
      temp1Val = newTemp1;
    }else{
      Serial.println("read Value was -127; ignored it");
    }
    
 if(newTemp2 != -127){
      temp2Val = newTemp2;
    }else{
      Serial.println("read Value was -127; ignored it");
    }
 if(newTemp3 != -127){
      temp3Val = newTemp3;
    }else{
      Serial.println("read Value was -127; ignored it");
    }
 if(newTemp4 != -127){
      temp4Val = newTemp4;
    }else{
      Serial.println("read Value was -127; ignored it");
    }
    // Zeige Temperatur auf Serieller Schnittstelle
    printTemperature("temp1",temp1Val); 
    printTemperature("temp2",temp2Val);
    printTemperature("temp3",temp3Val);
    printTemperature("temp4",temp4Val);
    printTemperatureLCD();
    lastTempReadTime = currentMillis;
  
  }
}



// function to print the temperature for a device
void printTemperature(String name,double tempC)
{
  Serial.print("Temperatur C    ");
  Serial.print(name);
  Serial.print(": ");
  Serial.println(tempC);

}

// function to print the temperature for a device
void printTemperatureLCD(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(temp1Val);
  lcd.print(" ");
  lcd.print(temp2Val);
  lcd.setCursor(0,1);
  lcd.print(temp3Val);
  lcd.print(" ");
  lcd.print(temp4Val);
    
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

