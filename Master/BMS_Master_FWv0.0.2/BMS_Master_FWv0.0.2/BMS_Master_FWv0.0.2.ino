// States der Communication-Statemachine 
#define SELECT_SLAVE 1
#define SELECT_COMMAND 2
#define SEND_COMMAND 3
#define WAIT_FOR_ACK 4
#define RECEIVE_DATA 5
#define NEW_S_DETECTED 6

#include "Arduino.h"
//#include "EEPROM.h"
#include "Display.h"
#include "Crc16.h"
#include "CellToMasterCom.h"
#include <Wire.h>
#include "DS3231.h"
#include <avr/wdt.h>
#include "Logging_Master.h"

const uint8_t cGu8_Slaves = 20;// Zahl der maximal unterstuetzten Slaves bei 20 Slaves ergibt das eine maximale Systemspannung von 84V mehr waere gefaehrlich
const uint8_t cGu8_RTC_VCC = 2;
const uint8_t cGu8_ResetPin = 3;
const uint8_t cGu8_SD_Card_CS = 49;


class Slavetable
{
	public:
	Slavetable()
	{}
	uint8_t CellNumber = 0;// The number of the cell this slave is assigned to
	uint8_t S_Address = 0;// The Slave adress this slave is listening to
	uint8_t SleepDelay = 0;// The time in seconds this slve will sleep after the last sleep command
	uint32_t Timeout = 0;// The timestamp in milliseconds for reaction to any command
	uint8_t ErrorCounter = 0;//Counts any issue in communication with this Slave Error = +10 every sucsessfully communication = -1
	uint32_t SendCommand;// The Command which will send next time or was sent last time
	uint32_t SendData[4];// The Data which will send next time or was sent last time
	uint32_t RecivedCommand;// The Command which was received last time
	uint32_t RecivedData[4];// The Data which was received last time
	
	uint16_t Voltage;// The Voltage of the cell this slave is connected to
	uint8_t CoreTemp;// The temprature of the controlers internal sensor indicates the temprature of passive ballancer circuit
	uint8_t CellTemp;// The temprature of the external sensor directly mounted at the conectet cell
	bool ActiveBallanceAvailable = false;// indicates whether or not active ballance circuit is connected
	bool ActiveBallanceStatus = false;// indicates whether or not the active ballancer is active
	bool PassBallanceStatus = false;// indicates whether or not the passive ballancer is active
	bool IdentBlink;// wether or not the identification blink signal is on
	uint32_t ActiveBallanceTime = 0;// the time in seconds active ballancing was active since the Master was started or the counter was reset via the GUI
	uint32_t PassBallanceTime = 0;// the time in seconds passive ballancing was active since the Master was started or the counter was reset via the GUI
	
	
	private:
}
Slave[cGu8_Slaves+1];//Die Slavetabelle hat an der stelle 0 einen dummy um neue slaves zu finden die Slaves 1..20 entsprechen auch den Arrayindikatoren 1..20

Crc16 crc;
CellToMasterCom Communication(&Serial1,&Serial);
DS3231 RTC;
DateAndTime Now;
LoggingClass Logg(&Now, &Serial, LEVEL_ALL_PLUS);
DisplayClass Display(&Serial2, &Serial);

volatile bool Gb_UpdateDisp = false;
volatile bool cGb_UpdateClock = true;

uint8_t Gu8_FSM_State = SELECT_SLAVE;
uint8_t Gu8_ActiveSlave = 0;
bool Gb_SlTableFull = false;

void setup()
{
	pinMode(cGu8_RTC_VCC, OUTPUT);
	pinMode(cGu8_ResetPin, INPUT);

	Communication.m_USART->begin(1200);// Slave UART wuerde auch bis 4800 laufen
	Serial2.begin(115200);// Nextion Display UART
	Serial.begin(115200);// Debug Konsole, wird auch von der CellToMasterCom-Klasse verwendet
	while (!Serial) {
		continue; // wait for serial port to connect. Needed for native USB port only
	}
	Serial.println();
	
	Wire.begin();
	watchdogOn(8);//1, 2, 4, oder 8 Sekunden
	wdt_reset();
	SetupTimer5();
	Logg.Init(cGu8_SD_Card_CS, &SD);
	
	RTC_read(&cGb_UpdateClock);
	
	String Ausgabe;
	if((MCUSR & (1 << PORF)) && MCUSR)
	Ausgabe = "after Power-on Reset";
	else if((MCUSR & (1 << EXTRF)) && MCUSR)
	Ausgabe = "after External Reset";
	else if((MCUSR & (1 << BORF)) && MCUSR)
	Ausgabe = "after Brown-out Reset";
	else if((MCUSR & (1 << WDRF)) && MCUSR)
	Ausgabe = "after Watchdog Reset";
	else if((MCUSR & (1 << JTRF)) && MCUSR)
	Ausgabe = "after JTAG Reset";
	else
	{
		Ausgabe = "after undefined Reset";
	}
	Logg.Logging(LEVEL_INFO, "Startup Master", Ausgabe);
	MCUSR = 0;
	
	Slave[0].S_Address = 0xFF;// einen slave-0 mit Adresse FF fuer die permanente Suche nach neuen Slaves
	
	Display.sdReady(Logg.SdPresent());
}

void loop()
{
	wdt_reset();
	RTC_read(&cGb_UpdateClock);
	uint32_t DispIncoming = Display.CheckIncome();
	uint8_t DispCommand = DispIncoming&0x000000FF;
	uint8_t DispData1 = (DispIncoming>>8)&0x000000FF;
	uint8_t DispData2 = (DispIncoming>>16)&0x000000FF;
	uint8_t DispData3 = (DispIncoming>>24)&0x000000FF;
	
	if(DispIncoming != 0 && DispIncoming != 0xFF)
	{
		Logg.Logging(LEVEL_FSM, "Incomming from Display", DispIncoming, HEX);
		Logg.Logging(LEVEL_FSM, "Command from Display", DispCommand, HEX);
		Logg.Logging(LEVEL_FSM, "Data1 from Display", DispData1, HEX);
		Logg.Logging(LEVEL_FSM, "Data2 from Display", DispData2, HEX);
		Logg.Logging(LEVEL_FSM, "Data3 from Display", DispData3, HEX);

	}
	
	switch(DispCommand)
	{
		case 0xB1:
		{// 
			Logg.Logging(LEVEL_FSM, "Set Time and Date from Display to RTC");
			RTC.setYear(Display.GetData(0)-2000);
			RTC.setMonth(Display.GetData(1));
			RTC.setDate(Display.GetData(2));
			RTC.setHour(Display.GetData(3));
			RTC.setMinute(Display.GetData(4));
			RTC.setSecond(0);
			RTC.setDoW(Display.GetData(5));
			cGb_UpdateClock = true;
			break;
			}
		case 0xA1:
		{// Clock_Set Monitor aktualisieren
			Logg.Logging(LEVEL_FSM, "Send Data to Elements of Clock_Set Screen");
			Display.WriteNumElement("Clock_Set", "n0", (uint16_t)Now.m_YR+2000);
			Display.WriteNumElement("Clock_Set", "n1", Now.m_MO);
			Display.WriteNumElement("Clock_Set", "n2", Now.m_DY);
			Display.WriteNumElement("Clock_Set", "n3", Now.m_HH);
			Display.WriteNumElement("Clock_Set", "n4", Now.m_MM);
			Display.WriteNumElement("Clock_Set", "va0", Now.m_DOW);
			break;
		}
		case 0xA2:
		{// SD_Overview Monitor aktualisieren
			Logg.Logging(LEVEL_FSM, "Send Data to Elements of SD_Overview Screen");
			Display.WriteTextElement("SD_Overview", "t14", Logg.getCardType());
			Display.WriteTextElement("SD_Overview", "t15", Logg.getFatType());
			Display.WriteTextElement("SD_Overview", "t16", Logg.getVolumeSize());
			Display.WriteTextElement("SD_Overview", "t17", Logg.getLoggfileName());
			Display.WriteTextElement("SD_Overview", "t18", Logg.getFreeSpace());
			Display.WriteTextElement("SD_Overview", "t5", "");
			break;
		}
		case 0xA3:
		{
			if(!Logg.SdPresent())
			{// Button Mount SD card 
				Logg.Logging(LEVEL_FSM, "Mount SD Card");
				if(Logg.Reopen())
				{
					Display.WriteTextElement("SD_Overview", "t5", "SD Card mounted");
				}
				else
				{
					Display.WriteTextElement("SD_Overview", "t5", "SD Card not present");
				}
				Display.sdReady(Logg.SdPresent());
			}
			else
			{
				Display.WriteTextElement("SD_Overview", "t5", "SD Card allready\rmounted");
			}
			break;
		}
		case 0xA4:
		{// Button unmount sd card
			if(Logg.SdPresent())
			{
				Logg.Logging(LEVEL_FSM, "Unmount SD Card");
				Logg.unmountSD();
				Display.WriteTextElement("SD_Overview", "t5", "SD Card ejected");
				Display.sdReady(Logg.SdPresent());
			}
			else
			{
				Display.WriteTextElement("SD_Overview", "t5", "SD Card allready\runmounted");
			}
			
			break;
		}
		case 0xA5:
		{//Seite Logging mit den letzten Zeilen aus dem Loggfile versorgen
			delay(1);
			String Logg2nd;
			Display.WriteTextElement("Logging", "t1", Logg.getLastLogg(&Logg2nd));
			Display.WriteTextElement("Logging", "t0", Logg2nd);
			//Display.WriteTextElement("Logging", "b4", "Back");
			//Display.WriteTextElement("Logging", "t0", "Hier sollte was zu sehen sein");
			//Serial.print(Logg.getLastLogg());
			break;
		}
		case 0xA6:
		{// Initialisiert die Seite Slave_Conf mit Daten aus der Slavetabelle
			Logg.Logging(LEVEL_FSM, "Slave Config Seite aktualisieen");;
			for(uint8_t i = 1; i < 8; i++)
			{
				Display.WriteTextElement("Slave_Conf", "t"+String(i+13), String(Slave[i].S_Address));//t14 bis t20
				Display.WriteNumElement("Slave_Conf", "n"+String(i-1), Slave[i].CellNumber);
				String status;//Es existieren die Stati sleeping (wenn das Sleepdelay nicht 0 ist), absent (wenn die Slaveadresse 0 ist), failure und Ok
				if(Slave[i].S_Address == 0)
				{
					status = "absent";
				}
				else if(Slave[i].SleepDelay != 0)
				{
					status = "sleeping";
				}
				else
				{
					status = "Ok";
				}
				Display.WriteTextElement("Slave_Conf", "t"+String(i+6), status);//t7 bis t13
			}
			break;
		}
		case 0xC1:
		{// laesst den angesprochenen Slave zur identifikation mit einer LED blinken
			Logg.Logging(LEVEL_FSM, "Identification blink Slave", DispData2);
			Logg.Logging(LEVEL_FSM, "On / Off", DispData1);
			break;
		}
		case 0xC2:
		{// Setzt eine zuordnung des Slave zur Zelle (Zellnummer)
			Logg.Logging(LEVEL_FSM, "Set Cell Number to Slave", DispData2);
			Logg.Logging(LEVEL_FSM, "New Cell number", DispData1);
			break;
		}
		case 0xFF:
		{
			break;
		}
		case 0x00:
		{
			break;
		}
		default:
		{
			Logg.Logging(LEVEL_ERROR, "Command not recognized", DispIncoming, HEX);
			break;
		}
	};
	
	
	
	FSM_Communication();
	
	if(Gb_UpdateDisp)
	{
		Gb_UpdateDisp = false;
		Display.WriteMaintime(Now);
		Logg.CheckFileSize();
	}
}

void FSM_Communication()
{
	switch(Gu8_FSM_State)
	{
		case SELECT_SLAVE:
		{
			if(Slave[Gu8_ActiveSlave].SleepDelay!=0)
			{//Wenn der slave noch schlaeft
				if(Gu8_ActiveSlave<cGu8_Slaves)
				{//Und wir nicht am Ende der Tabelle sind
					Gu8_ActiveSlave++;// Einfach den naechsten nehmen
				}
				else
				{
					Gu8_ActiveSlave = 0;// Ansonsten wieder von vorn beginnen
				}
				if(Slave[Gu8_ActiveSlave].S_Address == 0)
				{// Falls an diesm punkt der Tabelle gar kein slave mehr drinn steht
					Gu8_ActiveSlave = 0;// wieder von vorn beginnen
				}
			}
			else
			{//Ist der Slave wach geht es mit ihm weiter
				Gu8_FSM_State = SELECT_COMMAND;
			}
			Logg.Logging(LEVEL_DEBUG, "SELECT_SLAVE", Gu8_ActiveSlave);
			break;
		}
		case SELECT_COMMAND:
		{
			if(Gu8_ActiveSlave == 0)
			{
				Slave[Gu8_ActiveSlave].SendCommand = ARE_U_THERE;
				Logg.Logging(LEVEL_DEBUG, "SELECT_COMMAND", Slave[Gu8_ActiveSlave].SendCommand, HEX);
			}
			else
			{
					Slave[Gu8_ActiveSlave].SendCommand = BLINK_ON;
					Logg.Logging(LEVEL_DEBUG, "SELECT_COMMAND", Slave[Gu8_ActiveSlave].SendCommand, HEX);
//hier kommt noch eine richtige Befehlsauswahl
			}
			Gu8_FSM_State = SEND_COMMAND;
			break;
		}
		case SEND_COMMAND:
		{
			Communication.MasterOrder(Slave[Gu8_ActiveSlave].S_Address, Slave[Gu8_ActiveSlave].SendCommand, Slave[Gu8_ActiveSlave].SendData[0], Slave[Gu8_ActiveSlave].SendData[1], Slave[Gu8_ActiveSlave].SendData[2], Slave[Gu8_ActiveSlave].SendData[3]);
			Slave[Gu8_ActiveSlave].Timeout= millis() + 500;
			Logg.Logging(LEVEL_DEBUG, "Sent Command to Slave", Gu8_ActiveSlave);
			Logg.Logging(LEVEL_DEBUG, "Command", Slave[Gu8_ActiveSlave].SendCommand, HEX);
			Logg.Logging(LEVEL_DEBUG, "Slaveadress", Slave[Gu8_ActiveSlave].S_Address, HEX);
			Gu8_FSM_State = WAIT_FOR_ACK;
			break;
		}
		case WAIT_FOR_ACK:
		{
			int32_t Val = 0;
			if(Communication.CheckAvailable() > 2)
			{//Die Anzahl der im Puffer befindlichen Bytes checken ein ACK besteht aus drei Byte erst wenn mindestens so viele im Puffer liegen werden sie abgeholt und interpretiert
				Val = Communication.CheckACK(Slave[Gu8_ActiveSlave].S_Address);
				
				if (Val == 1)
				{// Bei Erfolg geht es hier weiter
					if (Slave[Gu8_ActiveSlave].SendCommand >= SEND_VOLT && Slave[Gu8_ActiveSlave].SendCommand <= SEND_STATUS)
					{// Alle Befehle, die Daten vom Slave einfordern benoetigen nach dem ACK eine Routine zum datenempfang
						Logg.Logging(LEVEL_DEBUG, "WAIT_FOR_ACK successful next is RECEIVE_DATA");
						Gu8_FSM_State = RECEIVE_DATA;
					}
					else if(Slave[Gu8_ActiveSlave].S_Address == 0xFF && Slave[Gu8_ActiveSlave].SendCommand == ARE_U_THERE)
					{// erfolgt ein ACK von dieser SLA und ist es die Antwort auf ARE_U_THERE, handelt es sich um einen neuen Slave
						if(!Gb_SlTableFull)
						{// ist die Slavetabelle bisher noch nicht als voll gekennzeichnet worden, kann der neue Slave eingetragen werden
							Logg.Logging(LEVEL_FSM, "ACK from new Slave received");
							Gu8_FSM_State = NEW_S_DETECTED;
						}
						else
						{// ist die Slavetabelle voll, wird mit der Kommunikation wie gehabt fort gefahren
							Logg.Logging(LEVEL_DEBUG, "WAIT_FOR_ACK successful Selavetable full next is SELECT_SLAVE");
							Gu8_FSM_State = SELECT_SLAVE;
						}
					}
					else if(Slave[Gu8_ActiveSlave].S_Address == 0xFF && Slave[Gu8_ActiveSlave].SendCommand == SET_SLA)
					{// wenn das ACK auf SET_SLA erfolgt, muss die SLA in der Slavetabelle angepasst werden
						Logg.Logging(LEVEL_FSM, "Write new SLA to Slavetable", Gu8_ActiveSlave, HEX);
						Slave[Gu8_ActiveSlave].S_Address = Gu8_ActiveSlave;
						Gu8_FSM_State = SELECT_COMMAND;
					}
					else
					{// in allen anderen Faellen, kann der naechse Befehl gesendet werden
						Logg.Logging(LEVEL_DEBUG, "WAIT_FOR_ACK successful next is SELECT_COMMAND");
						Gu8_FSM_State = SELECT_COMMAND;
					}
					
					if(Slave[Gu8_ActiveSlave].S_Address != 0xFF && Slave[Gu8_ActiveSlave].ErrorCounter > 0)
					{// bei erfolgreicher Kommunikation, wird der Fehlercounter dekrementiert
						Slave[Gu8_ActiveSlave].ErrorCounter --;
					}
				}
				else
				{// liegt ein Fehlercode vor wird dieser protokolliert 
					Logg.Logging(LEVEL_ERROR, "ACK Error at Slaveadress", Slave[Gu8_ActiveSlave].S_Address, HEX);
					Logg.Logging(LEVEL_ERROR, "ACK Error-Code", Val);
					
					if(Slave[Gu8_ActiveSlave].S_Address != 0xFF && Slave[Gu8_ActiveSlave].ErrorCounter < 190)
					{// der Fehlercounter wird nur fuer bekannte Slaves gezaehlt und soll 200 nicht ueberschreiten
						Slave[Gu8_ActiveSlave].ErrorCounter += 10;
					}
					Slave[Gu8_ActiveSlave].SleepDelay = 20;
					Gu8_FSM_State = SELECT_SLAVE;// es wird erstmal mit allen anderen Slaves kommuniziert so, dass dieser Slave ein wenig Zeit bekommt
				}
			}
			else if(Slave[Gu8_ActiveSlave].Timeout < millis())
			{
				if(Gu8_ActiveSlave == 0)
				{// Beim Suchen nach neuen Mitarbeitern darf schon mal ein Timeout auftreten
				}
				else
				{// kommt von eingestelleten Mitarbeitern keine Antwort, wird hier ein timeout error protokolliert
					Logg.Logging(LEVEL_ERROR, "ACK receiving Timeout at SlavetableNo", Gu8_ActiveSlave);
					Logg.Logging(LEVEL_ERROR, "ACK receiving Timeout at Slaveadress", Slave[Gu8_ActiveSlave].S_Address, HEX);
					if(Gu8_ActiveSlave != 0 && Slave[Gu8_ActiveSlave].ErrorCounter < 190)
					{// der Fehlercounter wird nur fuer bekannte Slaves gezaehlt und soll 200 nicht ueberschreiten
						Slave[Gu8_ActiveSlave].ErrorCounter += 10;
				}
				Slave[Gu8_ActiveSlave].SleepDelay = 20;
				Gu8_FSM_State = SELECT_SLAVE;// es wird erstmal mit allen anderen Slaves kommuniziert so, dass dieser Slave ein wenig Zeit bekommt
			}
			}
			break;
		}
		case NEW_S_DETECTED:
		{
			uint8_t FreeSlot;
			for(uint8_t i = 1; i < cGu8_Slaves; i++)
			{// sucht in der Slavetabelle den ersten freien Platz
				if(Slave[i].S_Address == 0 || Slave[i].S_Address == 0xFF)
				{// erkennt das Ende der Slavetabelle ist dort bereits ein neuer Slave eingetragen wird er ueberschrieben
					FreeSlot = i;
					break;
				}
				if(i == cGu8_Slaves-1)
				{// wenn auch der Letzte Platz besetzt ist
					Logg.Logging(LEVEL_ERROR, "Slavetable full no new Slave allowed", i);
					Gb_SlTableFull = true;
					break;
				}
			}
			if(!Gb_SlTableFull)
			{
				Slave[FreeSlot].S_Address=Slave[Gu8_ActiveSlave].S_Address;
				Slave[FreeSlot].CellNumber = 0;
				Slave[FreeSlot].SendCommand = SET_SLA;
				Slave[FreeSlot].SendData[0] = FreeSlot;
				Gu8_ActiveSlave = FreeSlot;
				Gu8_FSM_State = SEND_COMMAND;
				Logg.Logging(LEVEL_FSM, "Slave added to Slavetable at", FreeSlot);
			}
			else
			{
				Gu8_FSM_State = SELECT_SLAVE;
			}
			break;
		}
	}
}

void RTC_read(bool* Update)
{
	if(*Update)
	{
		Logg.Logging(LEVEL_MAIN, "Get new Time and Date from RTC");
		*Update = false;
		bool h12;
		bool PM;
		bool Century = false;
		digitalWrite(cGu8_RTC_VCC,HIGH);// Betriebsspannung der RTC einschalten
		delay(2);
		Now.m_YR = RTC.getYear();
		Now.m_MO = RTC.getMonth(Century);
		Now.m_DY = RTC.getDate();
		Now.m_HH = RTC.getHour(h12,PM);
		Now.m_MM = RTC.getMinute();
		Now.m_SS = RTC.getSecond();
		Now.m_DOW = RTC.getDoW();
		digitalWrite(cGu8_RTC_VCC,LOW);
		Logg.Logging(LEVEL_MAIN, "Got new Time and Date from RTC");
	}
}

void SetupTimer5()
{
	uint8_t RegHelper = 0;
	TCCR5A = RegHelper;// kein Output an HW-Pins WGM50 und WGM51 auf 0

	RegHelper = 0;
	RegHelper = (1 << CS50) | (1 << CS52) | (1 << WGM52);
	TCCR5B = RegHelper;// Clear Timer on Compare Match und Prescaler = 1024

	TCNT5 = 0;// clear counter value

	//OCR5A = 15625;// timer compare alle 1 Sekunde
	OCR5A = 15625;

	RegHelper = 0;
	RegHelper = (1 << TOV5) | (1 << OCF5A);
	TIFR5 = RegHelper; // clear Timer/Counter2 Overflow and compare match Flags

	RegHelper = 0;
	RegHelper = (1 << OCIE5A);
	TIMSK5 = RegHelper;// Enable OCR2A compare match Interrupt
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

ISR(TIMER5_COMPA_vect)
{// Timer5 Interrupt genau 1 Sekunde
	Gb_UpdateDisp = true;
	for(uint8_t i = 0; i < cGu8_Slaves; i++)
	{
		if(Slave[i].SleepDelay > 0)
		{// dekrementiert die Zeit die der Slave im Sleepmodus ist bis zur naechsten Kommunikationsmoeglichkeit
			Slave[i].SleepDelay--;
		}
		if(Slave[i].PassBallanceStatus)
		{// zaehlt die Sekunden, die die Zelle passiv ballanced wurde
			Slave[i].PassBallanceTime++;
		}
		if(Slave[i].ActiveBallanceStatus)
		{// zaehlt die Sekunden, die die Zelle aktiv ballanced wurde
			Slave[i].ActiveBallanceTime++;
		}
		if(Slave[i].S_Address == 0)
		{// erkennt das Ende der Slavetabelle
			break;
		}
	}
	Now.increment(&cGb_UpdateClock);
}

ISR(WDT_vect) 
{// Watchdog Interrupt sehr Ungenau ca. 1 Sekunde
	Logg.Logging(LEVEL_ERROR, "Watchdog \"WAU\" \"WAU\"");
	Serial.flush();
	MCUSR |= (1 << WDRF);
	pinMode(cGu8_ResetPin, OUTPUT);
	digitalWrite(cGu8_ResetPin,LOW);
}
