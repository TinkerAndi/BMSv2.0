#include <SoftwareSerial.h>
#include "CellToMasterCom.h"
#include <avr/sleep.h>

const byte rxPin = 13;
const byte txPin = 12;
const byte slaR_Pin = 11;
const uint8_t red = 5;
const uint8_t orange = 6;
const uint8_t green = 9;
const uint8_t Bal_active = 2;
const uint8_t bal_passive = 3;

bool	Gb_bal_Status = false;
bool	Gb_OverTemp = false;
bool	Gb_ExtBalControl = false; // true wenn der Ballancer von außen eingeschaltet wurde
bool	Gb_BlinkOr = false;

uint32_t	Gu32_Timestamp001 = 0;
uint32_t	Gu32_Timestamp002 = 0;
uint16_t	Gu16_BalThreshold = 4100; // Spannung ab der der pasive Ballancer auf jeden Fall aktiv werden muss
uint8_t		Gu8_BalHysteresis = 100; // Differenz zu Gu16_BalThreshold ab der der pasive Ballancer wieder aus geht
uint16_t	Gu16_BalUndervoltThreshold = 2800; // Spannung ab der der pasive Ballancer auf jeden Fall inaktiv werden muss
uint8_t		Gu8_OvertempThreshold = 60; // Abschalttemperatur(Temperature Range AtMega328p: -40?C to 85?C)
uint8_t		Gu8_OvertempHysteresis = 5;
uint8_t		Gu8_SlaveAdress = 255;

Crc16 crc;// Objekt zuz Bildung einer Pruefsumme
SoftwareSerial console (rxPin, txPin); // Debug Schnittstelle ueber SPI-Pins MISO=TX, MOSI=RX
CellToMasterCom Communication(&Serial, &console, &Gu8_SlaveAdress);

void setup() {
	// initialize the digital pin as an output.
	pinMode(slaR_Pin, INPUT_PULLUP);//Pin zum resetten der Slave Adresse
	pinMode(red, OUTPUT);// rote LED
	pinMode(orange, OUTPUT);// orange LED
	pinMode(green, OUTPUT);// gruene LED
	pinMode(Bal_active, OUTPUT);// Relais
	pinMode(bal_passive, OUTPUT);// Widerstandsarray
		
	Communication.m_USART->begin(1200);// wuerde auch bis 4800 laufen
	UCSR0B &= ~(1<<TXEN0);//TX auf Low - Die UART haellt den TX-Pin im idle auf High das darf so nicht sein, weil andere Slaves dann nicht senden koennen
	Communication.m_Console->begin(9600);// Debug Schnittstelle ueber SPI-Pins MISO=TX, MOSI=RX
	Communication.m_Console->println();	
	Communication.CheckSLA(digitalRead(slaR_Pin) == LOW);//Handling Slaveadresse (laden, lassen oder loeschen)
	Communication.m_Console->println("Cell Monitor boot completed");
	
	watchdogOn(1); // Watchdog timer einschalten zum aufwachen aus dem sleepmode
	digitalWrite(green, HIGH);
}

void loop() 
{
	uint8_t Instruction = 0;
	uint8_t Data_A4[4];
	uint32_t u32_Temprature = readTemp();
	int8_t retVal = 0;
	
	retVal = Communication.SlaveReceive(&Instruction, &Data_A4[0], &Data_A4[1], &Data_A4[2], &Data_A4[3]);
	Communication.ACK(retVal);
	
	switch(Instruction)
	{
		case ARE_U_THERE:
		{// Der Master fragt einzig die Anwesenheit des Slave ab
			console.println("ARE_U_THERE ping received");
			break;
		}
		case BLINK_ON:
		{// Der Master fragt einzig die Anwesenheit des Slave ab
			Gb_BlinkOr = true;
			console.println("BLINK_ON received");
			break;
		}
		case BLINK_OFF:
		{// Der Master fragt einzig die Anwesenheit des Slave ab
			Gb_BlinkOr = false;
			console.println("BLINK_OFF received");
			break;
		}
		case SET_SLA:
		{// Eine neue Slaveadresse setzen
			Communication.ChangeSLA(Data_A4[0]);
			break;
		}
		case SLEEP:
		{// Sleep Modus
			Sleep(Data_A4[0]);
			break;
		}
		case PASS_BALL_ON:
		{// Passives Ballancing aktivieren
			Gb_ExtBalControl = true;
			break;
		}
		case PASS_BALL_OFF:
		{// Passives Ballancing deaktivieren
			Gb_ExtBalControl = false;
			Ballance(false);
			console.print("Balancing external off! @ Voltage ");
			console.println(readVcc());
			break;
		}
		case ACT_BALL_ON:
		{// aktives Ballancing aktivieren
			digitalWrite(Bal_active, HIGH);
			digitalWrite(orange, HIGH);
			delay(100);
			console.println("Charge on");
			break;
		}
		case ACT_BALL_OFF:
		{// aktives Ballancing deaktivieren
			digitalWrite(Bal_active, LOW);
			digitalWrite(orange, LOW);
			delay(100);
			console.println("Charge off");
			break;
		}
		case SEND_VOLT:
		{// die Zellspannung zum Master senden
			delay(100);
			console.println(readVcc());
			break;
		}
		case SEND_INT_TEMP:
		{// die Chip-Temperatur zum Master senden
			delay(100);
			console.println(u32_Temprature);
			break;
		}
		case SEND_STATUS:
		{// Der Master fragt einzig die Anwesenheit des Slave ab
			console.println("SEND_STATUS received");
			break;
		}
	};
	
	CheckForBallance();
	CheckForTemprature();
	Cyclic();
}

void Cyclic()
{
	if(millis() > Gu32_Timestamp002)
	{
		Gu32_Timestamp002 = 500 + millis();
		
		if(Gb_BlinkOr)
		{
			digitalWrite(orange, !digitalRead(orange));
		}
		else
		{
			digitalWrite(orange, digitalRead(Bal_active));
		}
		
		if(Communication.GetSLA() == 0xFF)
		{
			digitalWrite(green, !digitalRead(green));
		}
		else
		{
			digitalWrite(green, HIGH);
		}
	}
	
}

long readVcc()// Misst die Spannung (VCC) in mV
{// Read 1.1V reference against AVcc
	long result;
	
	ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
	delay(20); // Wait for Vref to settle 
	
	ADCSRA |= (1 << ADSC); 
	
	// Convert 
	while (bit_is_set(ADCSRA,ADSC)); 
	result = ADCL; 
	result |= ADCH<<8; 
	result = 1126400L / result; // umwandeln DN in mA
	
	result += (result / 405) * 22 - 23; // Korrektur des Vorwiderstands am Prozessor
	
	// Back-calculate AVcc in mV 
	return result;
}

long readTemp()// Misst die Chiptemperatur in Grad Celsius
{ // Read temperature sensor against 1.1V reference
	long result;
	ADMUX = (1 << REFS1) | (1 << REFS0) | (1 << MUX3);
	delay(20); // Wait for Vref to settle 
	
	ADCSRA |= (1 << ADSC);
	
	// Convert 
	while (bit_is_set(ADCSRA,ADSC)); 
	result = ADCL; 
	result |= ADCH<<8;
	double Celsius = result - 317;
	Celsius = Celsius * 0.81;
	
	return (long)Celsius;
}

void Ballance(bool On)
{
	if(On && !Gb_OverTemp)
	{
		digitalWrite(red, HIGH);
		digitalWrite(bal_passive, HIGH);
	}
	else
	{
		digitalWrite(red, LOW);
		digitalWrite(bal_passive, LOW);
	}
	
	Gb_bal_Status = On;
}

bool Ballance()
{
	return Gb_bal_Status;
}

void CheckForBallance()
{// Prueft die Spannung und schaltet den passiven Ballancer
	if(Ballance() == false)
	{// Wenn der Ballancer nicht schon an ist
		long Voltage = readVcc();
		
		if(Voltage > Gu16_BalThreshold && !Gb_OverTemp)
		{// einschalten zum Ueberladungsschutz aber der Brandschutz geht vor
			Ballance(true);
			console.print("Balancing auto on! @ Voltage ");
			console.println(Voltage);
		}
		else if(Gb_ExtBalControl && !Gb_OverTemp && Voltage > Gu16_BalUndervoltThreshold)
		{// einschalten wenn externer Befehl vorliegt aber Tiefentladungsschutz und Brandschutz gehen vor
			Ballance(true);
			console.print("Balancing external on! @ Voltage ");
			console.println(readVcc());
		}
	}
	else if(millis() > Gu32_Timestamp001)
	{// wenn der Ballancer an ist muss er zur Spannungsmessung abgeschaltet werden, das darf nicht so oft passieren
		Ballance(false);
		delay(1000);
		long Voltage = readVcc();
		Gu32_Timestamp001 = millis() + 10000;// Stellt sicher, das nur alle 10s eine Abschaltung zum Messen erfolgt
		
		if(Voltage < Gu16_BalThreshold - Gu8_BalHysteresis && !Gb_ExtBalControl)
		{// Wenn kein Ueberladungsschutz mehr gebraucht wird
			console.print("Balancing auto off! @ Voltage ");
			console.println(Voltage);
		}
		else if(Voltage < Gu16_BalUndervoltThreshold)
		{// Wenn Tiefentladung droht
			console.print("Balancing undervoltage off! @ Voltage ");
			console.println(Voltage);
		}
		else if(!Gb_OverTemp)
		{// Oben wurde zur Messung abgeschaltet hier wieder an wenn nicht eine der Abschaltbedingungen erfuellt ist
			Ballance(true);
		}
	}
}

void CheckForTemprature()
{// Misst die Temperatur und entscheidet, ob weiter geballanced werden kann
	uint32_t u32_Temprature = readTemp();
	
	if(u32_Temprature > Gu8_OvertempThreshold)
	{// Temperatur zu hoch
		Gb_OverTemp = true;
	}
	else if(u32_Temprature < Gu8_OvertempThreshold - Gu8_OvertempHysteresis)
	{// Temperatur wieder ok mit Hysterese
		Gb_OverTemp = false;
	}
	if(Gb_OverTemp)
	{// Sofortiges Ausschalten um Zersörung zu verhindern
		if(Ballance())
		{// natuerlich nur wenn auch an ist
			Ballance(false);
			console.print("Balancing overtemp off! @ Voltage ");
			console.print(readVcc());
			console.print(" mV @ temp ");
			console.println(readTemp());
		}
	}
	
}

void Sleep(uint8_t Seconds)
{
	if(Seconds > 0)
	{
		console.println("enter sleep mode");
		while(Seconds > 0)
		{
			set_sleep_mode(SLEEP_MODE_PWR_DOWN); // den tiefsten Schlaf auswählen PWR_DOWN
			sleep_enable(); // sleep mode einschalten
			digitalWrite(green, LOW);
			UCSR0B &= ~(1<<TXEN0);
			__builtin_avr_wdr();
			sleep_mode(); // in den sleep mode gehen
			/*Hier wird geschlafen bis der Watchdog kommt*/
			Seconds --;
			sleep_disable(); // sleep mode ausschalten nach dem Erwachen
			digitalWrite(green, HIGH);
		}
	
		console.println("leave sleep mode");
	}
}

void watchdogOn(uint8_t Seconds)
{
	uint8_t reg = (1 << WDP0) | (1 << WDP3);// Standard 8 Sekunden
	
	if(Seconds == 1)
	reg = (1 << WDP1) | (1 << WDP2);
	if(Seconds == 2)
	reg = (1 << WDP0) | (1 << WDP1) | (1 << WDP2);
	if(Seconds == 4)
	reg = (1 << WDP3);
	if(Seconds == 8)
	reg = (1 << WDP0) | (1 << WDP3);
	
	
	MCUSR &= ~(1 << WDRF);
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = reg;
	WDTCSR |= (1 << WDIE);
	MCUSR &= ~(1 << WDRF);
}

ISR(WDT_vect) {
//	console.println("WDT_vect");
}











