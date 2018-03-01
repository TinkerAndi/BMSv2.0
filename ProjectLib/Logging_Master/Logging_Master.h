#ifndef Logging_Master_h
#define Logging_Master_h

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

/*Logging, im Befehl steht fogendes: if(LOGGING_LEVEL < Priority) return 0*/
#define DISABLE			0 //keine Ausgaben !!! nur als LOGGING_LEVEL verwenden nicht als Priority im Logging-Befehl
#define LEVEL_ERROR		1 //Fehlermeldungen
#define LEVEL_FSM 		2 //nur Setup und Statemachine
#define LEVEL_INFO 		3 //Mess- und Einstell-Wertausgaben
#define LEVEL_MAIN 		4 //Hauptfunktionen beim Aufruf und Verlassen
#define LEVEL_DEBUG		5 //Debugausschriften
#define LEVEL_MAIN_PLUS	6 //Hilfsfunktionen beim Aufruf und Verlassen
#define LEVEL_ALL 		7 //Statusmeldungen in Funktionen und Unterfunktionen
#define LEVEL_ALL_PLUS 	8 //Ausgaben auch in Interrupts


class DateAndTime
{
	public:
		DateAndTime()
		{
		};
		
		volatile uint8_t m_HH;
		volatile uint8_t m_MM;
		volatile uint8_t m_SS;
		volatile uint8_t m_DY;
		volatile uint8_t m_MO;
		volatile uint8_t m_YR;
		volatile uint8_t m_DOW;
		
		String getTime()
		{
			return String(m_HH) + ":" + String(m_MM) + ":" + String(m_SS);
		};
		String getDate()
		{
			return String(m_DY) + ":" + String(m_MO) + ":" + String((uint16_t)m_YR+2000);
		};
		String getWeekday()
		{
			return Weekdays[m_DOW-1];
		};
		String getDateTime()
		{
			return getDate() + " " + getTime();
		};
		void increment(bool* Update)
		{
			m_SS++;
			if(m_SS > 59)
			{// Interne Zeit zaehlen
				m_MM++;
				m_SS = 0;
				if(m_MM == 1)
				{
					*Update = true;
				}
				if(m_MM > 59)
				{
					m_MM = 0;
					m_HH++;
				}
			}
		};
		
	private:
		String Weekdays[7] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
};


class LoggingClass
{
public:
	LoggingClass(DateAndTime* pTime, HardwareSerial* USART, uint8_t LoggingLevel);
	void Init(uint8_t CS_Pin, SDClass* pSDCard);
	void Logging(uint8_t Priority, String statement);
	void Logging(uint8_t Priority, String statement, double value);
	void Logging(uint8_t Priority, String statement, int32_t value);
	void Logging(uint8_t Priority, String statement, int8_t value);
	void Logging(uint8_t Priority, String statement, uint32_t value, uint8_t NumberSystem);
	void Logging(uint8_t Priority, String statement, String value);
	void Logging(uint8_t Priority, String statement, uint16_t value, uint8_t NumberSystem);
	void Logging(uint8_t Priority, String statement, uint8_t value, uint8_t NumberSystem);
	void Logging(uint8_t Priority, String statement, uint8_t value);
	void Logging(uint8_t Priority, String statement, uint16_t value);
	void Logging(uint8_t Priority, String statement, uint32_t value);
	static const String Ziffer[];
	void Errorcode(uint16_t Command, uint16_t Errorcode);
	void Errorcode(uint16_t Command, uint16_t Errorcode, uint32_t Value);
	bool SdPresent();
	bool Reopen();
	void unmountSD(){m_useSD = false;};
	String getLastLogg(String* Logg2nd);
	void CheckFileSize();
	
	String getCardType(){return m_CardType;};
	String getFatType(){return m_FileSystem;};
	String getVolumeSize(){return String(m_VolSize);};
	String getLoggfileName(){return m_LoggFileName;};
	String getFreeSpace(){return String(m_Free_kB/1024);};
	
private:
	HardwareSerial* m_USART;
	Sd2Card m_card;
	SdVolume m_volume;
	DateAndTime* mp_Now;
	SDClass* mp_SDCard;
	String m_LoggFileName;
	File m_LoggFile;
	
	
	bool m_NeedNewFile = false;
	uint8_t m_CS_Pin;
	uint8_t m_LoggingLevel;
	bool m_useSD = false;
	String m_CardType = "Unknown";
	uint16_t m_VolSize = 0;
	uint32_t m_Free_kB = 0;
	String m_FileSystem = "Unknown";
	const uint32_t m_MaxFileSize = 4194304;
	
	
	bool LoggMarker(uint8_t Priority);
	bool CheckCard(uint8_t CS_Pin);
};




















#endif // Logging_Master_h