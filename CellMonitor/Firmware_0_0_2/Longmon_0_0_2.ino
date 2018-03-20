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
bool	Gb_Autoballance = true;

uint32_t	Gu32_Timestamp001 = 0;
uint32_t	Gu32_Timestamp002 = 0;
uint16_t	Gu16_BalThreshold = 4100; // Spannung ab der der pasive Ballancer auf jeden Fall aktiv werden muss
uint8_t		Gu8_BalHysteresis = 100; // Differenz zu Gu16_BalThreshold ab der der pasive Ballancer wieder aus geht
uint16_t	Gu16_BalUndervoltThreshold = 2800; // Spannung ab der der pasive Ballancer auf jeden Fall inaktiv werden muss
uint8_t		Gu8_OvertempThreshold = 60; // Abschalttemperatur(Temperature Range AtMega328p: -40?C to 85?C)
uint8_t		Gu8_OvertempHysteresis = 5;
uint8_t		Gu8_SlaveAdress = 255;
uint16_t	Gu16_Vcc = 0;
uint16_t	Gu16_IntTemp = 0;
uint16_t	Gu16_Offset = 0x7FFF; // Offset kann auch negativ sein, um das nicht im Nextion und auf dem Bus darstellen zu muessen entspricht 7FFF null
uint16_t	Gu16_Gain = 2000;

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
//	Communication.ChangeSLA(0xA1);
}

void loop() 
{
	uint8_t Instruction = 0;
	uint8_t Data_A4[4];
	int8_t retVal = 0;
	
	retVal = Communication.SlaveReceive(&Instruction, &Data_A4[0], &Data_A4[1], &Data_A4[2], &Data_A4[3]);
	Communication.ACK(retVal);
	
	switch(Instruction)
	{
		case ARE_U_THERE:// Der Master fragt einzig die Anwesenheit des Slave ab
		{
			console.println("ARE_U_THERE ping received");
			break;
		}
		case BLINK_ON:// Blinksignal orange LED ein
		{
			Gb_BlinkOr = true;
			console.println("BLINK_ON received");
			break;
		}
		case BLINK_OFF:// Blinksignal orange LED aus
		{
			Gb_BlinkOr = false;
			console.println("BLINK_OFF received");
			break;
		}
		case SET_SLA:// Eine neue Slaveadresse setzen
		{
			Communication.ChangeSLA(Data_A4[0]);
			break;
		}
		case SLEEP:// Sleep Modus
		{
			Sleep(Data_A4[0]);
			break;
		}
		case PASS_BALL_ON:// Passives Ballancing aktivieren
		{
			Gb_ExtBalControl = true;
			break;
		}
		case PASS_BALL_OFF:// Passives Ballancing deaktivieren
		{
			Gb_ExtBalControl = false;
			Ballance(false);
			console.print("Balancing external off! @ Voltage ");
			console.println(Gu16_Vcc);
			break;
		}
		case ACT_BALL_ON:// aktives Ballancing aktivieren
		{
			digitalWrite(Bal_active, HIGH);
			digitalWrite(orange, HIGH);
			delay(100);
			console.println("Charge on");
			break;
		}
		case ACT_BALL_OFF:// aktives Ballancing deaktivieren
		{
			digitalWrite(Bal_active, LOW);
			digitalWrite(orange, LOW);
			delay(100);
			console.println("Charge off");
			break;
		}
		case SEND_VOLT:// die Zellspannung zum Master senden
		{
			console.print("SEND_VOLT Volts: "); console.print(Gu16_Vcc); console.println(" mA");
			Communication.SlaveAnswer(SEND_VOLT, Gu16_Vcc);
			break;
		}
		case SEND_INT_TEMP:// die Chip-Temperatur zum Master senden
		{
			console.print("SEND_INT_TEMP Centigrade: "); console.print(Gu16_IntTemp); console.println(" C");
			Communication.SlaveAnswer(SEND_INT_TEMP, Gu16_IntTemp);
			break;
		}
		case SEND_STATUS:// Der Master fragt den Status ab
		{
			uint8_t PBalStat = Ballance();
			uint8_t ABalStat = digitalRead(Bal_active);
			uint8_t OverTStat = Gb_OverTemp;
			console.print("SEND_STATUS received");
			Communication.SlaveAnswer(SEND_STATUS, PBalStat, ABalStat, OverTStat);
			break;
		}
		case DISABLE_AUTO_BALLANCE:// Automatische Ballancen und Temperaturueberwachung aus
		{
			Gb_Autoballance = false;
			console.println("DISABLE_AUTO_BALLANCE received");
			break;
		}
		case ENABLE_AUTO_BALLANCE:// Automatische Ballancen und Temperaturueberwachung ein
		{
			Gb_Autoballance = true;
			console.println("ENABLE_AUTO_BALLANCE received");
			break;
		}
		case SEND_Volt_DN:// die Zellspannung zum Master senden
		{
			uint16_t DN = readVoltADC();
			console.print("SEND_Volt_DN DN: "); console.print(DN);
			Communication.SlaveAnswer(SEND_Volt_DN, DN);
			break;
		}
		case SEND_Volt_OFFSET:// die Zellspannung zum Master senden
		{
			console.print("SEND_Volt_OFFSET: "); console.print(Gu16_Offset); console.println(" mA");
			Communication.SlaveAnswer(SEND_Volt_OFFSET, Gu16_Offset);
			break;
		}
		case SEND_Volt_GAIN:// die Zellspannung zum Master senden
		{
			console.print("SEND_Volt_GAIN DN: "); console.print(Gu16_Gain); console.println(" mA");
			Communication.SlaveAnswer(SEND_Volt_GAIN, Gu16_Gain);
			break;
		}
		case 0x00:// Das allgemeine Geschäft erledigen wenn nichts vom Master kommt
		{
			Gu16_IntTemp = readTemp();
			Gu16_Vcc = readVcc();
			if(Gb_Autoballance)
			{
				CheckForBallance();
				CheckForTemprature();
			}
			Cyclic();
			break;
		}
	};
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

uint16_t readVoltADC()
{// Read 1.1V reference against AVcc
	uint16_t result;
	
	ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
	delay(10); // Wait for Vref to settle
	
	ADCSRA |= (1 << ADSC);
	
	// Convert
	while (bit_is_set(ADCSRA,ADSC));
	result = ADCL;
	result |= ADCH<<8;
	return result;
}

uint16_t readVcc()// Misst die Spannung (VCC) in mV
{
	double Gain = (double)Gu16_Gain / 1000;
	uint16_t result = readVoltADC();
	result = (double)(result * Gain) + ((int32_t)Gu16_Offset - 0x7FFF);
	//result = 1126400L / result; // umwandeln DN in mA
	//result += (result / 405) * 22 - 23; // Korrektur des Vorwiderstands am Prozessor
	return result;
}

uint16_t readTemp()// Misst die Chiptemperatur in Grad Celsius
{ // Read temperature sensor against 1.1V reference
	uint16_t result;
	ADMUX = (1 << REFS1) | (1 << REFS0) | (1 << MUX3);
	delay(5); // Wait for Vref to settle 
	
	ADCSRA |= (1 << ADSC);
	
	// Convert 
	while (bit_is_set(ADCSRA,ADSC)); 
	result = ADCL; 
	result |= ADCH<<8;
	double Celsius = result - 317;
	Celsius = Celsius * 0.81;
	
	return (uint16_t)Celsius;
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
		
		if(Gu16_Vcc > Gu16_BalThreshold && !Gb_OverTemp)
		{// einschalten zum Ueberladungsschutz aber der Brandschutz geht vor
			Ballance(true);
			console.print("Balancing auto on! @ Voltage ");
			console.println(Gu16_Vcc);
		}
		else if(Gb_ExtBalControl && !Gb_OverTemp && Gu16_Vcc > Gu16_BalUndervoltThreshold)
		{// einschalten wenn externer Befehl vorliegt aber Tiefentladungsschutz und Brandschutz gehen vor
			Ballance(true);
			console.print("Balancing external on! @ Voltage ");
			console.println(Gu16_Vcc);
		}
	}
	else if(millis() > Gu32_Timestamp001)
	{// wenn der Ballancer an ist muss er zur Spannungsmessung abgeschaltet werden, das darf nicht so oft passieren
		Ballance(false);
		delay(300);
		Gu16_Vcc = readVcc();
		Gu32_Timestamp001 = millis() + 10000;// Stellt sicher, das nur alle 10s eine Abschaltung zum Messen erfolgt
		
		if(Gu16_Vcc < Gu16_BalThreshold - Gu8_BalHysteresis && !Gb_ExtBalControl)
		{// Wenn kein Ueberladungsschutz mehr gebraucht wird
			console.print("Balancing auto off! @ Voltage ");
			console.println(Gu16_Vcc);
		}
		else if(Gu16_Vcc < Gu16_BalUndervoltThreshold)
		{// Wenn Tiefentladung droht
			console.print("Balancing undervoltage off! @ Voltage ");
			console.println(Gu16_Vcc);
		}
		else if(!Gb_OverTemp)
		{// Oben wurde zur Messung abgeschaltet hier wieder an wenn nicht eine der Abschaltbedingungen erfuellt ist
			Ballance(true);
		}
	}
}

void CheckForTemprature()
{// Misst die Temperatur und entscheidet, ob weiter geballanced werden kann
	
	if(Gu16_IntTemp > Gu8_OvertempThreshold)
	{// Temperatur zu hoch
		Gb_OverTemp = true;
	}
	else if(Gu16_IntTemp < Gu8_OvertempThreshold - Gu8_OvertempHysteresis)
	{// Temperatur wieder ok mit Hysterese
		Gb_OverTemp = false;
	}
	if(Gb_OverTemp)
	{// Sofortiges Ausschalten um Zersörung zu verhindern
		if(Ballance())
		{// natuerlich nur wenn auch an ist
			Ballance(false);
			console.print("Balancing overtemp off! @ Voltage ");
			console.print(Gu16_Vcc);
			console.print(" mV @ temp ");
			console.println(Gu16_IntTemp);
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











