// States der Communication-Statemachine 
#define SELECT_SLAVE 1
#define SELECT_COMMAND 2
#define SEND_COMMAND 3
#define WAIT_FOR_ACK 4
#define RECEIVE_DATA 5

#include "Arduino.h"
#include <EEPROM.h>
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
const uint8_t cGu8_Sleeptime = 20;


class Slavetable
{
	public:
	Slavetable()
	{}
	//permanent zu speichernde Daten der Slavetabelle
	uint8_t CellNumber = 0;// The number of the cell this slave is assigned to
	uint8_t S_Address = 0;// The Slave adress this slave is listening to
	
	//operativ benoetigte Daten der Slavetabelle
	uint8_t SleepDelay = 0;// The time in seconds this slave will sleep after the last sleep command
	uint32_t Timeout = 0;// The timestamp in milliseconds for reaction to any command
	uint8_t ErrorCounter = 0;//Counts any issue in communication with this Slave Error = +10 every sucsessfully communication = -1
	uint32_t SendCommand;// The Command which will send next time or was sent last time
	uint32_t SendData[4];// The Data which will send next time or was sent last time
	//uint32_t RecivedCommand;// The Command which was received last time
	uint32_t RecivedData[4];// The Data which was received last time
	
	//funktionell erhobene Daten vom Slave selbst
	uint16_t Voltage;// The Voltage of the cell this slave is connected to
	uint16_t CoreTemp;// The temprature of the controlers internal sensor indicates the temprature of passive ballancer circuit
	uint8_t CellTemp;// The temprature of the external sensor directly mounted at the conected cell
	bool ActiveBallanceAvailable = false;// indicates whether or not active ballance circuit is connected
	bool ActiveBallanceStatus = false;// indicates whether or not the active ballancer is active
	bool PassBallanceStatus = false;// indicates whether or not the passive ballancer is active
	bool IdentBlink;// wether or not the identification blink signal is on
	uint32_t ActiveBallanceTime = 0;// the time in seconds active ballancing was active since the Master was started or the counter was reset via the GUI
	uint32_t PassBallanceTime = 0;// the time in seconds passive ballancing was active since the Master was started or the counter was reset via the GUI
	
	void SaveSLA(uint8_t SlavetableNo, uint8_t SLA)
	{
		S_Address = SLA;
		eeprom_update_byte(mp_ST1stSLA + SlavetableNo, SLA);
	}

	void SaveCellNo(uint8_t SlavetableNo, uint8_t CellNo)
	{
		CellNumber = CellNo;
		eeprom_update_byte(mp_ST1stCellNo + SlavetableNo, CellNo);
	}
	
	bool Restore(uint8_t SlavetableNo)
	{
		uint8_t a = eeprom_read_byte(mp_ST1stSLA + SlavetableNo);
		if(a > 0 && a < 0xFF)
		{// wenn eine valide Slaveadresse im EEPROM steht wird diese genommen
			S_Address = a;
			CellNumber = eeprom_read_byte(mp_ST1stCellNo + SlavetableNo);
			return true;
		}
		return false;
	}
	
	void Destroy(uint8_t SlavetableNo)
	{
		S_Address = 0;
		eeprom_update_byte(mp_ST1stSLA + SlavetableNo, 0);
	}
	
	private:
	const uint8_t* mp_ST1stSLA = 100;
	const uint8_t* mp_ST1stCellNo = 200;
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
uint8_t Gu8_ImediateStNo = 0;
uint8_t Gu8_ImediateCommand = NO_COMMAND;
uint8_t Gu8_Sleeptime = cGu8_Sleeptime;
bool Gb_SlTableFull = false;
bool Gb_CommActive = false;
//bool Gb_CommBlock = false;

void setup()
{
	//Debug Ausgaenge
	pinMode(22, OUTPUT);//orange am Log. Analyzer
	pinMode(24, OUTPUT);//rot am Log. Analyzer
	pinMode(26, OUTPUT);//braun am Log. Analyzer
	pinMode(28, OUTPUT);//schwarz am Log. Analyzer
	
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
	RestoreSlavetable();
}

void loop()
{
	wdt_reset();
	RTC_read(&cGb_UpdateClock);
	uint32_t DispIncoming = Display.CheckIncome();
	uint8_t DispCommand = DispIncoming & 0x000000FF;
	uint8_t DispData1 = (DispIncoming>>8) & 0x000000FF;
	uint8_t DispData2 = (DispIncoming>>16) & 0x000000FF;
	uint8_t DispData3 = (DispIncoming>>24) & 0x000000FF;
	
	if(DispIncoming != 0 && DispIncoming != 0xFF)
	{
		Logg.Logging(LEVEL_FSM, "Incomming from Display", DispIncoming, HEX);
		Logg.Logging(LEVEL_FSM, "Command from Display", DispCommand, HEX);
		Logg.Logging(LEVEL_FSM, "Data1 from Display", DispData1, HEX);
		Logg.Logging(LEVEL_FSM, "Data2 from Display", DispData2, HEX);
		Logg.Logging(LEVEL_FSM, "Data3 from Display", DispData3, HEX);

	}
	
	switch(DispCommand)// Befehlsauswahl Kommandos vom Display
	{
		case 0xB1:// Stellt die Uhr
		{
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
		case 0xB2:// Schaltet den passiven Ballancer an und aus
		{
			
			if(DispData1 == 1)
			{
				Communication.MasterOrder(Slave[DispData2].S_Address, PASS_BALL_ON);
				Slave[DispData2].PassBallanceStatus = true;
			}
			else
			{
				Communication.MasterOrder(Slave[DispData2].S_Address, PASS_BALL_OFF);
				Slave[DispData2].PassBallanceStatus = false;
			}
			Logg.Logging(LEVEL_FSM, "switch Passiv Ballancing", DispData2);
			Logg.Logging(LEVEL_FSM, "1 = On / 0 = Off", DispData1);
			break;
		}
		case 0xA1:// Clock_Set Monitor aktualisieren 
		{
			Logg.Logging(LEVEL_FSM, "Send Data to Elements of Clock_Set Screen");
			Display.WriteNumElement("Clock_Set", "n0", (uint16_t)Now.m_YR+2000);
			Display.WriteNumElement("Clock_Set", "n1", Now.m_MO);
			Display.WriteNumElement("Clock_Set", "n2", Now.m_DY);
			Display.WriteNumElement("Clock_Set", "n3", Now.m_HH);
			Display.WriteNumElement("Clock_Set", "n4", Now.m_MM);
			Display.WriteNumElement("Clock_Set", "va0", Now.m_DOW);
			break;
		}
		case 0xA2:// SD_Overview Monitor aktualisieren 
		{
			Logg.Logging(LEVEL_FSM, "Send Data to Elements of SD_Overview Screen");
			Display.WriteTextElement("SD_Overview", "t14", Logg.getCardType());
			Display.WriteTextElement("SD_Overview", "t15", Logg.getFatType());
			Display.WriteTextElement("SD_Overview", "t16", Logg.getVolumeSize());
			Display.WriteTextElement("SD_Overview", "t17", Logg.getLoggfileName());
			Display.WriteTextElement("SD_Overview", "t18", Logg.getFreeSpace());
			Display.WriteTextElement("SD_Overview", "t5", "");
			break;
		}
		case 0xA3:// Button Mount SD card 
		{
			if(!Logg.SdPresent())
			{
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
		case 0xA4:// Button unmount sd card 
		{
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
		case 0xA5:// Seite Logging mit den letzten Zeilen aus dem Loggfile versorgen 
		{
			delay(1);
			String Logg2nd;
			Display.WriteTextElement("Logging", "t1", Logg.getLastLogg(&Logg2nd));
			Display.WriteTextElement("Logging", "t0", Logg2nd);
			//Display.WriteTextElement("Logging", "b4", "Back");
			//Display.WriteTextElement("Logging", "t0", "Hier sollte was zu sehen sein");
			//Serial.print(Logg.getLastLogg());
			break;
		}
		case 0xA6:// Initialisiert die Seite Slave_Conf mit Daten aus der Slavetabelle 
		{
			Gu8_Sleeptime = 0;
			Logg.Logging(LEVEL_FSM, "Slave Config Seite aktualisieen");
			UpdateDispSlTable();
			break;
		}
		case 0xC1:// laesst den angesprochenen Slave zur identifikation mit einer LED blinken 
		{
			
			if(DispData1 == 1)
			{
				Communication.MasterOrder(Slave[DispData2].S_Address, BLINK_ON);
			}
			else
			{
				Communication.MasterOrder(Slave[DispData2].S_Address, BLINK_OFF);
			}
			int8_t Ec1 = Communication.CheckACK(Slave[DispData2].S_Address);
			Logg.Logging(LEVEL_FSM, "Identification blink Slave", DispData2);
			Logg.Logging(LEVEL_FSM, "On / Off", DispData1);
			break;
		}
		case 0xC2:// Setzt eine zuordnung des Slave zur Zelle (Zellnummer) 
		{
			Logg.Logging(LEVEL_FSM, "Set Cell Number to Slave", DispData2);
			Slave[DispData2].SaveCellNo(DispData2, DispData1);// 
			Logg.Logging(LEVEL_FSM, "New Cell number", Slave[DispData2].CellNumber);
			UpdateDispSlTable();
			break;
		}
		case 0xC3:// Sucht nach neuen Slaves 
		{
			Display.WriteTextElement("Slave_Conf", "t1", "Searching");
			int8_t ret = SearchNewSlave();
			if(ret == 1)
			{
				Display.WriteTextElement("Slave_Conf", "t1", "found new slave");
			}
			else
			{
				Display.WriteTextElement("Slave_Conf", "t1", "no new Slave");
			}
			UpdateDispSlTable();
			break;
		}
		case 0xC4:// Loescht die Slavetabelle 
		{
			Display.WriteTextElement("Slave_Conf", "t1", "Slavetable clear");
			ClearSlavetable();
			UpdateDispSlTable();
			break;
		}
		case 0xC5:// Aktualisiert die Slave-Details Seite 
		{
			Gu8_Sleeptime = 0;
			Display.WriteTextElement("Slave_Details", "t10", String(Slave[DispData1].S_Address, HEX));
			Display.WriteNumElement("Slave_Details", "n0", Slave[DispData1].CellNumber);
			Display.WriteNumElement("Slave_Details", "n2", Slave[DispData1].ErrorCounter);
			Display.WriteNumElement("Slave_Details", "n3", Slave[DispData1].Voltage);
			Display.WriteNumElement("Slave_Details", "n6", Slave[DispData1].CoreTemp);
			Display.WriteNumElement("Slave_Details", "n7", Slave[DispData1].CellTemp);
			Display.WriteNumElement("Slave_Details", "n4", Slave[DispData1].PassBallanceTime);
			Display.WriteNumElement("Slave_Details", "n5", Slave[DispData1].ActiveBallanceTime);
			if(Slave[DispData1].PassBallanceStatus)
			{
				Display.WriteTextElement("Slave_Details", "t16", "on");
			}
			if(Slave[DispData1].ActiveBallanceStatus)
			{
				Display.WriteTextElement("Slave_Details", "t17", "on");
			}
			if(Slave[DispData1].ActiveBallanceAvailable)
			{
				Display.WriteTextElement("Slave_Details", "t16", "available");
			}
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

int8_t SearchNewSlave()// Sucht neue Slave Controller (Zellmonitore) und nimmt sie in die Slavetabelle auf 
{
	Logg.Logging(LEVEL_FSM, "BEGINN of SearchNewSlave()");
	Slave[0].SendCommand = ARE_U_THERE;
	Communication.MasterOrder(Slave[0].S_Address, Slave[0].SendCommand);
	Logg.Logging(LEVEL_DEBUG, "sending ARE_U_THERE", Slave[0].SendCommand, HEX);
	int8_t Ec1 = Communication.CheckACK(Slave[0].S_Address);
	if (Ec1 == 1)
	{// Bei Erfolg geht es hier weiter
		Logg.Logging(LEVEL_DEBUG, "found a new Slave");
		
		
		uint8_t FreeSlot;
		for(uint8_t i = 1; i < cGu8_Slaves; i++)
		{// sucht in der Slavetabelle dem ersten freien Platz 0 wird ausgelassen
			if(Slave[i].S_Address == 0 || Slave[i].S_Address == 0xFF)
			{// erkennt den ersten freien Platz in der Slavetabelle
				FreeSlot = i;
				break;
			}
			if(i == cGu8_Slaves-1)
			{// wenn auch der Letzte Platz der Slavetabelle besetzt ist
				Gb_SlTableFull = true;
				break;
			}
		}
		if(!Gb_SlTableFull)
		{// Wenn ein freier Platz vorhanden ist
			uint8_t NewSLA = 0xA0 + FreeSlot;
			Communication.MasterOrder(Slave[0].S_Address, SET_SLA, NewSLA);
			Logg.Logging(LEVEL_DEBUG, "Send Command SET_SLA to", NewSLA, HEX);
			int8_t Ec2 = Communication.CheckACK(Slave[0].S_Address);
			
			if(Ec2 == 1)
			{//Wenn das Setzen der neuen SLA erfolgreich war
				Logg.Logging(LEVEL_DEBUG, "Programming new SLA successful", NewSLA, HEX);
				Communication.MasterOrder(NewSLA, ARE_U_THERE);//Schau ob der Slave auf die neue Adresse reagiert
				int8_t Ec3 = Communication.CheckACK(NewSLA);
				
				if(Ec3 == 1)
				{//aufnehmen des neuen Slve in die Slavetabelle
					Logg.Logging(LEVEL_DEBUG, "Slave respond to new Adress successful");
					
					Slave[FreeSlot].SaveSLA(FreeSlot, NewSLA);// neue Slaveadresse in Slavetabelle aufnehmen
					Slave[FreeSlot].SaveCellNo(FreeSlot, 0);// neue Slaves sind erstmal keiner Zelle zugeordnet
					
					Logg.Logging(LEVEL_MAIN, "Slave added to Slavetable at", FreeSlot);
				}
				else
				{
					Logg.Logging(LEVEL_ERROR, "CheckACK of nwew SLA ARE_U_THERE returns error", (int32_t)Ec3);
					return -21;
				}
			}
			else
			{
				Logg.Logging(LEVEL_ERROR, "CheckACK of SET_SLA returns error", (int32_t)Ec2);
				return -21;
			}
		}
		else
		{
			Logg.Logging(LEVEL_ERROR, "Slavetable full no new Slave allowed");
			return -20;
		}
	}
	else if(Ec1 == -3)
	{
		Logg.Logging(LEVEL_DEBUG, "did not find a new Slave");
		return 0;
	}
	else
	{
		Logg.Logging(LEVEL_ERROR, "CheckACK of ARE_U_THERE returns error", (int32_t)Ec1);
		return -21;
	}
	
	return 1;
}

int8_t RestoreSlavetable()// Liest die Slavetabelle aus dem EEPROM 
{
	Logg.Logging(LEVEL_FSM, "BEGINN of RestoreSlavetable()");
	for(uint8_t i = 1; i < cGu8_Slaves; i++)
	{
		if(Slave[i].Restore(i))
		{
			Logg.Logging(LEVEL_INFO, "Slave restored to Slavetable at", i);
		}
	}
}

int8_t ClearSlavetable()//Loescht die Slavetabelle im EEPROM 
{
	Logg.Logging(LEVEL_FSM, "BEGINN of ClearSlavetable()");
	for(uint8_t i = 1; i < cGu8_Slaves; i++)
	{
		Slave[i].Destroy(i);
	}
}

void FSM_Communication()
{
	switch(Gu8_FSM_State)
	{
		case SELECT_SLAVE:
		{
			if(Slave[Gu8_ActiveSlave].SleepDelay!=0 || Slave[Gu8_ActiveSlave].S_Address == 0xFF || Slave[Gu8_ActiveSlave].S_Address == 0)
			{//Wenn der slave noch schlaeft oder unkonfiguriert ist
				if(Gu8_ActiveSlave<cGu8_Slaves)
				{//Und wir nicht am Ende der Tabelle sind
					Gu8_ActiveSlave++;// Einfach den naechsten nehmen
				}
				else
				{
					Gu8_ActiveSlave = 1;// Ansonsten wieder von vorn beginnen
				}
				if(Slave[Gu8_ActiveSlave].S_Address == 0)
				{// Falls an diesm punkt der Tabelle gar kein slave mehr drinn steht
					Gu8_ActiveSlave = 1;// wieder von vorn beginnen
				}
			}
			else
			{//Ist der Slave wach geht es mit ihm weiter
				delay(20);
				if(Slave[Gu8_ActiveSlave].CellNumber == 10)
				{
					digitalWrite(22,HIGH);
				}
				else
				{
					digitalWrite(22,LOW);
				}
				Slave[Gu8_ActiveSlave].SendCommand = NO_COMMAND;
				Gu8_FSM_State = SELECT_COMMAND;
				Logg.Logging(LEVEL_DEBUG, "SELECT_SLAVE SlaveNo", Gu8_ActiveSlave);
			}
			break;
		}
		case SELECT_COMMAND:
		{
			if(Slave[Gu8_ActiveSlave].SendCommand == SLEEP)
			{// SLEEP ist immer das letzte Kommando in der Abarbeitung
				Logg.Logging(LEVEL_DEBUG, "SELECT_COMMAND last Command reached next is SELECT_SLAVE");
				Gu8_FSM_State = SELECT_SLAVE;
			}
			else
			{
				switch(Slave[Gu8_ActiveSlave].SendCommand)
				{
					case NO_COMMAND:
					{
						Slave[Gu8_ActiveSlave].SendCommand = SEND_VOLT;
						Logg.Logging(LEVEL_DEBUG, "SELECT_COMMAND: SEND_VOLT");
						break;
					}
					case SEND_VOLT:
					{
						Slave[Gu8_ActiveSlave].SendCommand = SEND_INT_TEMP;
						Logg.Logging(LEVEL_DEBUG, "SELECT_COMMAND: SEND_INT_TEMP");
						break;
					}
					case SEND_INT_TEMP:
					{
						Slave[Gu8_ActiveSlave].SendCommand = SEND_STATUS;
						Logg.Logging(LEVEL_DEBUG, "SELECT_COMMAND: SEND_STATUS");
						break;
					}
					case SEND_STATUS:
					{
						Slave[Gu8_ActiveSlave].SendCommand = SLEEP;
						Logg.Logging(LEVEL_DEBUG, "SELECT_COMMAND: SLEEP");
						Slave[Gu8_ActiveSlave].SendData[0] = Gu8_Sleeptime;
						Slave[Gu8_ActiveSlave].SleepDelay = Gu8_Sleeptime + 1 + (Gu8_Sleeptime / 8);
						break;
					}
				}
				Logg.Logging(LEVEL_DEBUG, "SELECT_COMMAND CommandNo", Slave[Gu8_ActiveSlave].SendCommand, HEX);
				Gu8_FSM_State = SEND_COMMAND;
			}
			break;
		}
		case SEND_COMMAND:
		{
			Gb_CommActive = true;
			Communication.MasterOrder(Slave[Gu8_ActiveSlave].S_Address, Slave[Gu8_ActiveSlave].SendCommand, Slave[Gu8_ActiveSlave].SendData[0], Slave[Gu8_ActiveSlave].SendData[1], Slave[Gu8_ActiveSlave].SendData[2], Slave[Gu8_ActiveSlave].SendData[3]);
			Slave[Gu8_ActiveSlave].Timeout= millis() + 400; //Von beginn der Master Transmission bis Ende Slave ACK vergehen gemessen ca. 295ms
			Logg.Logging(LEVEL_DEBUG, "SEND_COMMAND SlaveNo", Gu8_ActiveSlave);
			Logg.Logging(LEVEL_DEBUG, "SEND_COMMAND CommandNo", Slave[Gu8_ActiveSlave].SendCommand, HEX);
			Logg.Logging(LEVEL_DEBUG, "SEND_COMMAND Slaveadress", Slave[Gu8_ActiveSlave].S_Address, HEX);
			Gu8_FSM_State = WAIT_FOR_ACK;
			break;
		}
		case RECEIVE_DATA:
		{
			int8_t ErC1 = 0;
			uint8_t* dummy1;
			uint8_t* dummy2;
			uint8_t* dummy3;
			uint8_t* dummy4;
			
			if(Slave[Gu8_ActiveSlave].SendCommand == SEND_VOLT)
			{
				ErC1 = Communication.GetData(Slave[Gu8_ActiveSlave].S_Address, (uint8_t)SEND_VOLT, &Slave[Gu8_ActiveSlave].Voltage, dummy2, dummy3);
				Logg.Logging(LEVEL_DEBUG, "GetData SEND_VOLT", Slave[Gu8_ActiveSlave].Voltage);
			}
			else if(Slave[Gu8_ActiveSlave].SendCommand == SEND_INT_TEMP)
			{
				ErC1 = Communication.GetData(Slave[Gu8_ActiveSlave].S_Address, (uint8_t)SEND_INT_TEMP, &Slave[Gu8_ActiveSlave].CoreTemp, dummy2, dummy3);
				Logg.Logging(LEVEL_DEBUG, "GetData SEND_INT_TEMP", Slave[Gu8_ActiveSlave].CoreTemp);
			}
			else if(Slave[Gu8_ActiveSlave].SendCommand == SEND_STATUS)
			{
				ErC1 = Communication.GetData(Slave[Gu8_ActiveSlave].S_Address, (uint8_t)SEND_STATUS, dummy1, dummy2, dummy3, dummy4);
				Logg.Logging(LEVEL_DEBUG, "GetData SEND_STATUS PBalStat", *dummy1);
				Logg.Logging(LEVEL_DEBUG, "GetData SEND_STATUS ABalStat", *dummy2);
				Logg.Logging(LEVEL_DEBUG, "GetData SEND_STATUS OverTStat", *dummy3);
			}
			if(ErC1 != 1)
			{
				Logg.Logging(LEVEL_ERROR, "GetData returns Error", (int32_t)ErC1);
			}

			Gb_CommActive = false;
			Gu8_FSM_State = SELECT_COMMAND;
			break;
		}
		case WAIT_FOR_ACK:
		{
			int32_t Val = 0;
			if(Communication.CheckAvailable() > 4)
			{//Die Anzahl der im Puffer befindlichen Bytes checken ein ACK besteht aus 5 Byte erst wenn mindestens so viele im Puffer liegen werden sie abgeholt und interpretiert
				Logg.Logging(LEVEL_DEBUG, "WAIT_FOR_ACK at SLA", Slave[Gu8_ActiveSlave].S_Address, HEX);
				Val = Communication.CheckACK(Slave[Gu8_ActiveSlave].S_Address);
				
				if (Val == 1)
				{// Bei Erfolg geht es hier weiter
					if (Slave[Gu8_ActiveSlave].SendCommand >= SEND_VOLT && Slave[Gu8_ActiveSlave].SendCommand <= SEND_STATUS)
					{// Alle Befehle, die Daten vom Slave einfordern benoetigen nach dem ACK eine Routine zum datenempfang
						Logg.Logging(LEVEL_DEBUG, "WAIT_FOR_ACK successful next is RECEIVE_DATA");
						Gu8_FSM_State = RECEIVE_DATA;
					}
					else
					{// in allen anderen Faellen, kann der naechse Befehl gesendet werden
							Logg.Logging(LEVEL_DEBUG, "WAIT_FOR_ACK successful next is SELECT_COMMAND");
							Gb_CommActive = false;
							Gu8_FSM_State = SELECT_COMMAND;
					}
					
					if(Slave[Gu8_ActiveSlave].ErrorCounter > 0)
					{// bei erfolgreicher Kommunikation, wird der Fehlercounter dekrementiert
						Slave[Gu8_ActiveSlave].ErrorCounter --;
					}
				}
				else
				{// liegt ein Fehlercode vor wird dieser protokolliert 
					Logg.Logging(LEVEL_ERROR, "WAIT_FOR_ACK ACK Error at Slaveadress", Slave[Gu8_ActiveSlave].S_Address, HEX);
					Logg.Logging(LEVEL_ERROR, "WAIT_FOR_ACK ACK Error-Code", Val);
					
					if(Slave[Gu8_ActiveSlave].ErrorCounter < 191)
					{// der Fehlercounter wird nur fuer bekannte Slaves gezaehlt und soll 200 nicht ueberschreiten
						Slave[Gu8_ActiveSlave].ErrorCounter += 10;
					}
					Slave[Gu8_ActiveSlave].SleepDelay =  cGu8_Sleeptime;
					Gu8_FSM_State = SELECT_SLAVE;// es wird erstmal mit allen anderen Slaves kommuniziert so, dass dieser Slave ein wenig Zeit bekommt
				}
			}
			else if(Slave[Gu8_ActiveSlave].Timeout < millis())
			{// kommt vom Mitarbeiter keine Antwort, wird hier ein timeout error protokolliert
				Logg.Logging(LEVEL_DEBUG, "WAIT_FOR_ACK ACK receiving Timeout at SlavetableNo", Gu8_ActiveSlave);
				Logg.Logging(LEVEL_ERROR, "WAIT_FOR_ACK ACK receiving Timeout at Slaveadress", Slave[Gu8_ActiveSlave].S_Address, HEX);
				if(Gu8_ActiveSlave != 0 && Slave[Gu8_ActiveSlave].ErrorCounter < 191)
				{// der Fehlercounter wird nur fuer bekannte Slaves gezaehlt und soll 200 nicht ueberschreiten
					Slave[Gu8_ActiveSlave].ErrorCounter += 10;
				}
				Slave[Gu8_ActiveSlave].SleepDelay = cGu8_Sleeptime;
				Gu8_FSM_State = SELECT_SLAVE;// es wird erstmal mit allen anderen Slaves kommuniziert so, dass dieser Slave ein wenig Zeit bekommt
			}
			break;
		}
	}
}

void UpdateDispSlTable()// Versorgt die Displayseite Slave_Details mit Daten 
{
	for(uint8_t i = 1; i < 8; i++)
	{
		Display.WriteTextElement("Slave_Conf", "t"+String(i+13), String(Slave[i].S_Address, HEX));//t14 bis t20
		Display.WriteNumElement("Slave_Conf", "n"+String(i-1), Slave[i].CellNumber);
		String status;//Es existieren die Status sleeping (wenn das Sleepdelay nicht 0 ist), absent (wenn die Slaveadresse 0 ist), failure und Ok
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
}

void RTC_read(bool* Update)// Schaltet die RTC ein und liest Zeit und Datum 
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

void SetupTimer5()// Konfiguriet Timer 5 auf Interrupt 1 Hz 
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

void watchdogOn(uint8_t Seconds)// Konfiguriert den Watchdog Timer 
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

/*
Benutzt werden Codes zwischen -40 bis -21 und 21 bis 40 alle anderen sind reserviert fuer andere Kassen
Fehlercodes:

		-23	: 
		
		-22	: 
		
		-21	: SearchNewSlave() Kommunikationsfehler
		
		-20	: SearchNewSlave() Die Slavetabelle ist voll
		
		1	: Kein Fehler
		
		0	: Kein Fehler aber auch nix gemacht
*/