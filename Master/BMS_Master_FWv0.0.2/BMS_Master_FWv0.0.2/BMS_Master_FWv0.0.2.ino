#include "Arduino.h"
//#include "EEPROM.h"
#include "Display.h"
#include "Crc16.h"
#include "CellToMasterCom.h"
#include <Wire.h>
#include "DS3231.h"
#include <avr/wdt.h>
#include "Logging_Master.h"

const uint8_t Slaves = 20;// Zahl der maximal unterstuetzten Slaves bei 20 Slaves ergibt das eine maximale Systemspannung von 84V mehr waere gefaehrlich
const uint8_t RTC_VCC = 2;
const uint8_t ResetPin = 3;
const uint8_t SD_Card_CS = 49;


class Slavetable
{
	public:
	Slavetable()
	{}
	uint8_t CellNumber = 0;
	uint8_t S_Address = 0;
	uint8_t SleepDelay = 0;
	uint32_t Timeout = 0;
	
	uint16_t Voltage;
	uint8_t CoreTemp;
	uint8_t CellTemp;
	bool ActBallanceStatus = false;
	bool PassBallanceStatus = false;
	uint32_t ActBallanceTime = 0;
	uint32_t PassBallanceTime = 0;
	
	
	private:
}
Slave[Slaves];

Crc16 crc;
CellToMasterCom Communication(&Serial1,&Serial);
DS3231 RTC;
DateAndTime Now;
LoggingClass Logg(&Now, &Serial, LEVEL_ALL_PLUS);
DisplayClass Display(&Serial2, &Serial);

volatile bool Gb_UpdateDisp = false;
volatile bool UpdateClock = true;

void setup()
{
	pinMode(RTC_VCC, OUTPUT);
	pinMode(ResetPin, INPUT);

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
	Logg.Init(SD_Card_CS, &SD);
	
	RTC_read(&UpdateClock);
	
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
	Display.sdReady(Logg.SdPresent());
}

void loop()
{
	wdt_reset();
	RTC_read(&UpdateClock);
	uint32_t DispCommand = Display.CheckIncome();
	if(DispCommand != 0 && DispCommand != 0xFF)
	{
		Logg.Logging(LEVEL_FSM, "Command from Display", DispCommand, HEX);
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
			UpdateClock = true;
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
		case 0xA4:
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
		case 0xA5:
		{
			delay(1);
			String Logg2nd;
			Display.WriteTextElement("Logging", "t1", Logg.getLastLogg(&Logg2nd));
			Display.WriteTextElement("Logging", "t0", Logg2nd);
			Display.WriteTextElement("Logging", "b4", "Back");
			//Display.WriteTextElement("Logging", "t0", "Hier sollte was zu sehen sein");
			//Serial.print(Logg.getLastLogg());
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
			Logg.Logging(LEVEL_ERROR, "Command not recognized", DispCommand, HEX);
			break;
		}
	};
	
	
	

	if(Gb_UpdateDisp)
	{
		Gb_UpdateDisp = false;
		Display.WriteMaintime(Now);
		Logg.CheckFileSize();
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
		digitalWrite(RTC_VCC,HIGH);// Betriebsspannung der RTC einschalten
		delay(2);
		Now.m_YR = RTC.getYear();
		Now.m_MO = RTC.getMonth(Century);
		Now.m_DY = RTC.getDate();
		Now.m_HH = RTC.getHour(h12,PM);
		Now.m_MM = RTC.getMinute();
		Now.m_SS = RTC.getSecond();
		Now.m_DOW = RTC.getDoW();
		digitalWrite(RTC_VCC,LOW);
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
	for(uint8_t i = 0; i < Slaves; i++)
	{
		if(Slave[i].CellNumber != 0 && Slave[i].SleepDelay > 0)
		{// dekrementiert die Zeit die der Slave im Sleepmodus ist bis zur naechsten Kommunikationsmoeglichkeit
			Slave[i].SleepDelay--;
		}
		if(Slave[i].PassBallanceStatus)
		{// zaehlt die Sekunden, die die Zelle passiv ballanced wurde
			Slave[i].PassBallanceTime++;
		}
		if(Slave[i].ActBallanceStatus)
		{// zaehlt die Sekunden, die die Zelle aktiv ballanced wurde
			Slave[i].ActBallanceTime++;
		}
	}
	Now.increment(&UpdateClock);
}

ISR(WDT_vect) 
{// Watchdog Interrupt sehr Ungenau ca. 1 Sekunde
	Logg.Logging(LEVEL_ERROR, "Watchdog \"WAU\" \"WAU\"");
	Serial.flush();
	MCUSR |= (1 << WDRF);
	pinMode(ResetPin, OUTPUT);
	digitalWrite(ResetPin,LOW);
}
