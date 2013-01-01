// Deiser Code steuert die Relais ueber 2 Taster (einer für EIN und einer für AUS)
// Die Steuerung ermittelt ausserdem die Temperatur von 2 ds18b20 fuehlern an einem
// one wire bus
#include <OneWire.h>
#include <DallasTemperature.h>


// Data wire is plugged into port 8 on the Arduino
#define ONE_WIRE_BUS 4
#define TEMPERATURE_PRECISION 9
#define BOUNCE_DURATION 40 

#define BUTTON_ON 2 //Knopf in port 2
#define BUTTON_OFF 3 //Knopf in Port 3
#define RELAIS 12 //Relais in Port 12
#define RELAIS2 11 //Relais2 in Port 11
#define RELAIS3 10 //Relais3 in Port 10
#define RELAIS4 9 //Relais4 in Port 9
#define RELAIS5 8 //Relais 5 in Port 8
#define RELAIS6 7 //Relais 6 in Port 7


// Instantiert zwei Entpreller fuer die Knoepfe. Einen fuer ein und einen fuer aus

volatile unsigned long bounceTime=0; // variable to hold ms count to debounce a pressed switch
volatile boolean start = false; // Steuerung hat 2 Zustaende start = ja oder start = nein (-> stopp)

// Setup einen OneWire Bus fuer die Kommunikation mit den Temperaturfuehlern
OneWire oneWire(ONE_WIRE_BUS);
// Die DallasTemparature Lib uebernimtm das handling des one wire bus
DallasTemperature sensors(&oneWire);
const long TEMPR_READ_INTERVAL = 3000; // gibt an wie haeufig Temperatur gelesen wird

// arrays mit den Adressen der Temperaturfuehler 28D3AF3304000081
DeviceAddress temp1 = { 0x28, 0x81, 0x91, 0x33, 0x4, 0x0, 0x0, 0xE7 };
DeviceAddress temp2 = { 0x28, 0xD3, 0xAF, 0x33, 0x4, 0x0, 0x0, 0x81 };
DeviceAddress temp3 = { 0x28, 0xD2, 0xFF, 0x33, 0x4, 0x0, 0x0, 0x23 };
DeviceAddress temp4 = { 0x28, 0x89, 0x7A, 0x33, 0x4, 0x0, 0x0, 0xEC };

unsigned long lastTempReadTime = 0;  // Wann die Temperatur zuletzt gelesen wurde
const double vorwaermenTemp = 45; //Temperatur bei dem das Relais fuer das vorwaermen der Maschine ausgeschaltet wird
const double abgasWtEinTemp = 35; //Temperatur von der an die Pumpe des Abgaswaermetauschers lauft
double abgastWtPumpStartedTime = -1; //Zeit zu der die Abgaswaermetauschpumpe beim abstellen gestartet wurde, -1 bedeutet das noch keine Zeit gesetzt wurde

/**
* Initialisierung des Programms (Anschluesse, Sensonren etc ..)
*/
void setup() {
  pinMode(BUTTON_ON,INPUT);
  pinMode(BUTTON_OFF,INPUT);
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
  
 //button interrupt handler
 attachInterrupt(0, buttonInterruptHandler, LOW );
 attachInterrupt(1, buttonInterruptHandler, LOW );
  
}
/*
* Schlaufe welche dauernd wiederholt wird
*/
void loop() {
  readTemperatures();
  handleVorwaermen(); //vorwaermen handhaben
  handleAbgaswaermetauscher1();
  handleWaermetransportSpeicher();
  handleNotstop();

}
/* Startet die Vorwaermpumpe wenn die Temperatur noch tief ist
*/
void handleVorwaermen(){
  double temperature1 =  sensors.getTempC(temp1);
  if(start == true && temperature1 < vorwaermenTemp ){
    digitalWrite(RELAIS, LOW ); //Ausgang einschalten
  }else{
     digitalWrite(RELAIS, HIGH );
  }
}

/*
* handhabt die Steuerung der Pumpe des Abgaswarmetauscher 1
*/
void   handleAbgaswaermetauscher1(){
  double temperature3 = sensors.getTempC(temp3);//lies temperatur des fuehelrs 3
  if(start == true){
    abgastWtPumpStartedTime = -1;
     if(temperature3 > abgasWtEinTemp){
       digitalWrite(RELAIS2, LOW ); //Ausgang einschalten
  }  else{
     digitalWrite (RELAIS2,HIGH);
    }
  }
  else if(start == false){
     double temperature1 =  sensors.getTempC(temp1); 
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
}

/**
* handhabt den Waermetransport zum Speicher in 3 Stufen
*/
void handleWaermetransportSpeicher(){
    double temperature4 = sensors.getTempC(temp4);//lies temperatur des fuehelrs 4  
    double temperature3 = sensors.getTempC(temp3);//lies temperatur des fuehelrs 3
    double temperature2 = sensors.getTempC(temp2);//lies temperatur des fuehelrs 2
  if(start == true && temperature3 >= 70 && temperature3 < 80){
     activateExclusive(RELAIS3); //exklusiv einschalten
  } else if(start == true && temperature2 >= 95){
      activateExclusive(RELAIS3); //exklusiv einschalten
  }else if(start == true && temperature2 < 94.5){
      digitalWrite(RELAIS3,HIGH); //Wenn gekuehlt Pumpe wieder abschalten
  } else if(start == true && temperature3 >= 80 && temperature3 < 85){
      activateExclusive(RELAIS4);//4ein alle anderen aus
  } else if(start == true && temperature3 >= 85){
       activateExclusive(RELAIS5); //5 ein andere aus
  } else if(start == false && temperature3 > 40  && (temperature3 - temperature4) > 4){ 
       activateExclusive(RELAIS5); //5 ein andere aus
  }
  else{
    //nichts traf zu -> Alles ausschalten
      digitalWrite(RELAIS3,HIGH);
      digitalWrite(RELAIS4,HIGH);
      digitalWrite(RELAIS5,HIGH);
  }
}

/**
* handhabt den Notstop wenn uebertemperatur vorhanden ist
*/
void handleNotstop(){
    double temperature1 = sensors.getTempC(temp1);//lies temperatur des fuehelrs 1
    double temperature2 = sensors.getTempC(temp2);//lies temperatur des fuehelrs 2
    double temperature3 = sensors.getTempC(temp3);//lies temperatur des fuehelrs 3
    //uebertemperaturtest
    if (temperature1 >= 98 || temperature2 >= 108 || temperature3 >= 110 ){
      digitalWrite(RELAIS6,LOW);//Notstop einschalten
    }
}

void activateExclusive(int pinNumber){
  digitalWrite(RELAIS3,HIGH);
  digitalWrite(RELAIS4,HIGH);
  digitalWrite(RELAIS5,HIGH);
  if(pinNumber != RELAIS3 && pinNumber != RELAIS4 && pinNumber != RELAIS5 ){
    Serial.println("KRITISCHER FEHLER Pinnummer zum aktivieren muss einer der vorgaben entsprechen");
  } else{
    digitalWrite(pinNumber,LOW) ;//einschalten
  }
}


void buttonInterruptHandler(){  
// this is the interrupt handler for button presses
// it ignores presses that occur in intervals less then the bounce time
   if (abs(millis() - bounceTime) > BOUNCE_DURATION){
      int valueON = digitalRead(BUTTON_ON);
      int valueOFF = digitalRead(BUTTON_OFF);
       // Wenn AUS Schalter aktivier (auf low) dann wird das Relais ausgeschaltet
     if ( valueOFF == LOW) {
         start = false;
         digitalWrite(13,LOW);
     } else  if ( valueON == LOW) {
          //EIN Knopf wird nur geprueft wenn AUS Knopf nicht geschaltet
           start = true;
           digitalWrite(13,HIGH);
           digitalWrite(RELAIS, HIGH );
           digitalWrite(RELAIS2, HIGH );
           digitalWrite(RELAIS3, HIGH );
           digitalWrite(RELAIS4, HIGH );
           digitalWrite(RELAIS5,HIGH);
           digitalWrite(RELAIS6,HIGH); //allfaelliger Notstp wird bei start wieder geloescht
    } 
    bounceTime = millis();
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
    // Zeige Temperatur auf Serieller Schnittstelle
    printTemperature("temp1",temp1); 
    printTemperature("temp2",temp2);
    printTemperature("temp3",temp3);
    printTemperature("temp4",temp4);
    lastTempReadTime = currentMillis;
  
  }
}

// function to print the temperature for a device
void printTemperature(String name,DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temperatur C    ");
  Serial.print(name);
  Serial.print(": ");
  Serial.println(tempC);

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
