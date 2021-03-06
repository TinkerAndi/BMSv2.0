/*
Diese Klasse stellt die Kommunikation zwischen Ballancerboards und Master her.
Durch die langsame Hardware die zur galvanischen Trennung eingesetzt ist (billige Optokoppler und Pullup-Widerstand)
und den ungenauen internen Takt der Slave-ICs, kann hier nur mit geringen baudraten gearbeitet werden.
Das Protokoll baut die Serielle Schnittstelle zu einer Punkt zu multipunkt Funktionalitaet um, so koennen Slaves ueber Adressen
angesprochen werden.
Die Kommunikation sieht wie folgt aus:
Der Master steuert alles!

Es gibt nur ein zulaessiges Datagramm, welches vom Master gesendet wird mit folgendem Aufbau:
[Arbitrierung 3 Byte]  [Slaveadresse 1 Byte]  [Befehl 1 Byte]  [Datenbytes 4 Byte]  [CRC Pruefsumme]
Arbitrierung:
	Der Master sendet den String "@@@" das veranlasst die Slaves zur Empfangsbereitschaft
Slaveadresse:
	Nur der adressierte Slave empfaengt weitere Daten zu Beginn hat jeder Slave die Adresse 255. Der Master pflegt eine Slavetabelle und
	spricht nach allen bereits individuell adressierten Slaves immer die Defaultadresse 255 an
	antwortet ein Slave darauf informiert der Master den User ueber das Display und es kann eine Slaveadresse durch Userinteraktion vergeben
	werden. Es duerfen niemals mehrere unadressierte Slaves gleichzeitig angeschlossen werden.
	Antwortet ein Slave aus der Slavetabelle nicht, wird ebenfalls der User informiert.
	Ein Slave speichert seine Adresse im EEPROM sobald er vom Master adressiert wurde.
Befehl:
	Im Slave kann durch Aufruf eines Befehls eine bestimmte Aktion ausgeloest werden.
Daten:
	Es koennen Daten zum Slave uebertragen werden, werden keine Daten benoetigt muessen diese 4 Bytes jedoch immer mit gesendet werden
	der Inhalt der Datenbytes ist dann egal. Die interpretation der Datenbytes haengt vom jeweiligen Befehl ab.
CRC Pruefsumme:
	Es wird an jedes Datagramm eine 16 bit (2 Byte) CRC-Pruefsumme angehaengt. Die Pruefsumme wird von beiden Partnern ueber die Bytes
	von Slaveadresse bis Datenbytes berechnet bei CRC-Fehlern wird das gesamte Datagramm vom Empfaenger verworfen.
	
Auf diesen Befehl des Masters hin antwortet der Slave mit einem Acknowledge Datagramm ohne CRC-Pruefsumme:
[Dummy 1 Byte] [Arbitrierung 1 Byte] [Slaveadresse 1 Byte]  [Fehlercode 1 Byte]
Dummy:
	Durch das an und bschalten der TX Leitung am Slave kommt es gelegentlich zu Parasitaeren Bytes oder Fehlern im ersten Byte
	der Master verwirft deshalb alles, was vor der Arbitrirung kommt das Dummybyte enthält einen beliebigen Wert
Arbitrierung:
	Ein '@' signalisiert dem Mater, dass jetzt Daten zu erwarten sind.
Slaveadresse:
	Der Slave sendet seine eigene Slaveadresse um vom Master identifiziert werden zu koennen
Fehlercode:
	Es gibt folgende erlaubte Fehlercodes:
		-2	: Transmission incomplete (Das Datagramm ist nicht vollstaendig uebertragen worden oder es kam zu einem 
			  unzulaessigen Timeout zwischen den Bytes (> 10ms))
		-1	: CRC Error (Die Pruefsumme hat nicht gestimmt)
		1	: Datenuebertragung erfolgreich

Wenn der Befehl eine Antwort erfordert sendet der Slave in Antwortdatagramm:
[Slaveadresse 1 Byte]  [Befehl 1 Byte]  [Datenbytes 4 Bytes]  [CRC-Pruefsumme]
Slaveadresse:
	Der Slave sendet seine eigene Slaveadresse um vom Master identifiziert werden zu koennen
Befehl:
	Die Befehlsnummer ermoeglicht dem Master die Interpretation der folgenden Datenbytes
Daten:
	Es koennen Daten vom Slave uebertragen werden, der Inhalt der Datenbytes ist dann egal. 
	Die interpretation der Datenbytes haengt vom jeweiligen Befehl ab.
CRC Pruefsumme:
	Es wird 16 bit (2 Byte) CRC-Pruefsumme angehaengt. Die Pruefsumme wird von beiden Partnern ueber die Bytes
	von Slaveadresse bis Datenbytes berechnet bei CRC-Fehlern wird das gesamte Datagramm vom Empfaenger verworfen.
	
Zwischen den einzelnen Datagrammen vom Master an die Slaves muss ein Timeout von mindestens 100ms bei 1200 baud abgewartet werden.
*/

#ifndef CellToMasterCom_h
#define CellToMasterCom_h

#include <Arduino.h>
#include "Crc16.h"
#include <EEPROM.h>
#include "SoftwareSerial.h"

// Kommandos die der Slave versteht
#define BEGINN 0// !kein Kommando zum senden! 0 bedeutet, dass der Slave von vorn beginnen muss mit der Befehlskette
//Befehle die der Slave nur ausfuehrt
#define NO_COMMAND 0x00
#define ARE_U_THERE 0x01
#define SET_SLA 0x02
#define SLEEP 0x03
#define PASS_BALL_ON 0x04
#define PASS_BALL_OFF 0x05
#define ACT_BALL_ON 0x06
#define ACT_BALL_OFF 0x07
#define BLINK_ON 0x08
#define BLINK_OFF 0x09
#define DISABLE_AUTO_BALLANCE 0x10
#define ENABLE_AUTO_BALLANCE 0x11
// Befehle auf die der Slave mit Daten antwortet
#define SEND_VOLT 0x20
#define SEND_INT_TEMP 0x21
#define SEND_EXT_TEMP 0x22
#define SEND_Volt_DN 0x23
#define SEND_Volt_OFFSET 0x24
#define SEND_Volt_GAIN 0x25
//... andere Befehle, die Daten senden hier dazwischen
#define SEND_STATUS 0x3F


class CellToMasterCom
{
public:
	HardwareSerial* m_USART;
	SoftwareSerial* m_Console;
// Funktionen, die vom Master benutzt werden
	CellToMasterCom(HardwareSerial* USART, void* Console);// Master Constructor
	int8_t MasterOrder(uint8_t SLA, uint8_t Command, uint8_t u8_Data1 = 0, uint8_t u8_Data2 = 0, uint8_t u8_Data3 = 0, uint8_t u8_Data4 = 0);
	// int8_t MasterOrder(uint8_t SLA, uint8_t Command, uint16_t u16_Data1, uint8_t u8_Data3 = 0, uint8_t u8_Data4 = 0);
	// int8_t MasterOrder(uint8_t SLA, uint8_t Command, uint16_t u16_Data1, uint16_t u1_Data2);
	// int8_t MasterOrder(uint8_t SLA, uint8_t Command, uint32_t u32_Data);
	int8_t GetData(uint8_t SLA, uint8_t Command, uint8_t* u8_Data1, uint8_t* u8_Data2, uint8_t* u8_Data3, uint8_t* u8_Data4);
	int8_t GetData(uint8_t SLA, uint8_t Command, uint16_t* u16_Data1, uint8_t* u8_Data3, uint8_t* u8_Data4);
	int8_t CheckACK(uint8_t SLA);
	
// Funktionen, dei vom Slave benutzt werden
	CellToMasterCom(HardwareSerial* USART, SoftwareSerial* Console, uint8_t* p_SLA);// Slave Constructor
	int8_t SlaveAnswer(uint8_t Command, uint8_t u8_Data1, uint8_t u8_Data2 = 0, uint8_t u8_Data3 = 0, uint8_t u8_Data4 = 0);
	int8_t SlaveAnswer(uint8_t Command, uint16_t u16_Data1, uint8_t u8_Data3 = 0, uint8_t u8_Data4 = 0);
	// int8_t SlaveAnswer(uint8_t Command, uint16_t u16_Data1, uint16_t u1_Data2);
	// int8_t SlaveAnswer(uint8_t Command, uint32_t u32_Data);
	int8_t SlaveReceive(uint8_t* Command, uint8_t* u8_Data1, uint8_t* u8_Data2, uint8_t* u8_Data3, uint8_t* u8_Data4);
	// int8_t SlaveReceive(uint8_t Command, uint16_t* u16_Data1, uint8_t* u8_Data3, uint8_t* u8_Data4);
	// int8_t SlaveReceive(uint8_t Command, uint16_t* u16_Data1, uint16_t* u1_Data2);
	// int8_t SlaveReceive(uint8_t Command, uint32_t* u32_Data);
	int8_t ACK(int8_t ErrorCode);
	int8_t ChangeSLA(uint8_t newSLA);
	int8_t CheckSLA(bool erase);
	uint8_t GetSLA();
	
// Funktionen, die von Master und Slave benutzt werden koennen
	uint8_t CheckAvailable();
	
private:
	Crc16 m_crc;
	uint8_t* mp_SLA;
	uint8_t* mp_isConf = 1;
	uint8_t* mp_slaAdr = 2;
	bool isMaster;
	int8_t GetSerialByte(uint16_t Timeout, int8_t* Byte);
	
	void Arbitration();
	
};

int8_t CellToMasterCom::GetSerialByte(uint16_t Timeout, int8_t* Byte)
{
	uint32_t Timestamp = millis() + Timeout;
	while(!m_USART->available())
	{//warten, bis Daten eintreffen
		if(Timestamp < millis())
		{
			return 0;
		}
	}
	*Byte = m_USART->read();
	return 1;
}

CellToMasterCom::CellToMasterCom(HardwareSerial* USART, void* Console)
{
	m_USART = USART;
	*mp_SLA = 0;
	m_Console = Console;
	isMaster = true;
}

CellToMasterCom::CellToMasterCom(HardwareSerial* USART, SoftwareSerial* Console, uint8_t* p_SLA)
{
	m_USART = USART;
	mp_SLA = p_SLA;
	m_Console = Console;
	isMaster = false;
}

int8_t CellToMasterCom::MasterOrder(uint8_t SLA, uint8_t Command, uint8_t u8_Data1, uint8_t u8_Data2, uint8_t u8_Data3, uint8_t u8_Data4)
{
	if(!isMaster)
	{
		return -5;
	}
	
	m_crc.clearCrc();
	uint8_t Datablock[] = {SLA,Command,u8_Data1,u8_Data2,u8_Data3,u8_Data4};
	uint16_t CRC_sum = m_crc.XModemCrc(Datablock,0,6);
	uint8_t CRC1 = CRC_sum & 0x00FF;
	CRC_sum = CRC_sum >> 8;
	uint8_t CRC2 = CRC_sum & 0x00FF;
	
	while(m_USART->available())
	{//Restmuell abholen
		m_USART->read();
	}
	
	Arbitration();
	m_USART->write(SLA);
	m_USART->write(Command);
	m_USART->write(u8_Data1);
	m_USART->write(u8_Data2);
	m_USART->write(u8_Data3);
	m_USART->write(u8_Data4);
	m_USART->write(CRC1);
	m_USART->write(CRC2);
	
	return 1;
}

int8_t CellToMasterCom::GetData(uint8_t SLA, uint8_t Command, uint8_t* u8_Data1, uint8_t* u8_Data2, uint8_t* u8_Data3, uint8_t* u8_Data4)
{
	if(!isMaster)
	{
		return -5;
	}
	
	uint8_t Incomming = 0;
	uint8_t Datablock[6];// Array zur CRC berrechnung
	uint16_t CRC_sum = 0;// Pruefsumme
	bool CRC_pass = true;// CRC stimmt ueberein
	m_crc.clearCrc();
	
	if(!GetSerialByte(400, &Incomming))//holt das erste Byte meist das Dummybyte langes Timeout da Beginn der Uebertragung
	{
		return -8;
	}
	//m_Console->print("1st Byte: "); m_Console->println(Incomming, HEX);
	
	while(Incomming != '@')
	{// alles abholen bis das Datenpaket beginnt
		if(!GetSerialByte(20, &Incomming))//holt alles bis zum Beginn des Payload
		{
			return -8;
		}
		//m_Console->print("next Byte: "); m_Console->println(Incomming, HEX);
	}
	
	for(uint8_t i = 0; i < 6; i++)
	{
		if(!GetSerialByte(20, &Incomming))//holt den Payload
		{
			return -8;
		}
		//m_Console->print("Payload Byte: "); m_Console->println(Incomming, HEX);
		Datablock[i] = Incomming;
	}
	
	if(!GetSerialByte(20, &Incomming))//holt den Payload
	{// CRC Low holen
		return -8;
	}
	//m_Console->print("CRC low: "); m_Console->println(Incomming, HEX);
	
	if(!GetSerialByte(20,(uint8_t*) &CRC_sum))//holt den Payload
	{// CRC High holen
		return -8;
	}
	//m_Console->print("CRC high: "); m_Console->println(CRC_sum, HEX);
	CRC_sum = CRC_sum << 8;
	CRC_sum += Incomming;
	
	//m_Console->print("CRC 16: "); m_Console->println(CRC_sum, HEX);
	uint16_t CRCtemp = m_crc.XModemCrc(Datablock,0,6);
	//m_Console->print("CRC expect: "); m_Console->println(CRCtemp, HEX);
	
	if(CRCtemp == CRC_sum)
	{
		if(Datablock[0] == SLA)
		{
			if(Datablock[1] == Command)
			{
				*u8_Data1 = Datablock[2];
				*u8_Data2 = Datablock[3];
				*u8_Data3 = Datablock[4];
				*u8_Data4 = Datablock[5];
				
				while(m_USART->available())
				{//Restmuell abholen
					m_USART->read();
					delay(10);
				}
				
				return 1;
			}
			else
			{
				return -10;//Kommando falsch
			}
		}
		else
		{
			return -9;//SLA falsch
		}
		
	}
	else
	{
		return -1;//CRC Fehler
	}
	

		
	
	return 0;// Nix passiert, nix zu sehen, bitte gehen sie weiter
}

int8_t CellToMasterCom::GetData(uint8_t SLA, uint8_t Command, uint16_t* u16_Data1, uint8_t* u8_Data3, uint8_t* u8_Data4)
{
	uint8_t DatHighTemp;
	uint8_t DatLowTemp;
	
	int8_t Ec1 = GetData(SLA, Command, &DatHighTemp, &DatLowTemp, u8_Data3, u8_Data4);
	
	*u16_Data1 = DatLowTemp;
	*u16_Data1 |= DatHighTemp<<8;
	
	return Ec1;
}

int8_t CellToMasterCom::SlaveReceive(uint8_t* Command, uint8_t* u8_Data1, uint8_t* u8_Data2, uint8_t* u8_Data3, uint8_t* u8_Data4)
{
	if(isMaster)
	{
		return -6;
	}
	
	static uint8_t Count = 0;// zaehlt die empfangenen Bytes
	static bool Communication = false; // zeigt an, ob Adressierung erfolgreich war
	uint8_t Datablock[6];// Array zur CRC berrechnung
	uint16_t CRC_sum = 0;// Pruefsumme
	bool CRC_pass = true;// CRC stimmt ueberein
	m_crc.clearCrc();
		
	while(m_USART->available())
	{// Empfangen bis nix mehr kommt
		uint8_t Incomming = m_USART->read();
		
		if(Communication)
		{// Wird erst nach erfolgreicher Adressierung aufgerufen
			if(Count == 0)
			{// Dieses Byte ist der Befehl an den Slave und wird per Pointer aus der Funktion uebergeben
				*Command = Incomming;
				Datablock[1] = Incomming;
			}
			else if(Count < 5)
			{// Hier werden die 4 Datenbytes empfangen, die spaeter ebenfalls per Pointer aus der Funktion uebergeben werden
				Datablock[Count+1] = Incomming;
			}
			else if(Count < 7)
			{// Die letzten zwei Byte sind die CRC-Pruefsumme
				if(Count == 5)
				{// Das erste CRC-Byte wird sofort verglichen
					CRC_sum = m_crc.XModemCrc(Datablock,0,6);
					uint8_t Part = CRC_sum & 0x00FF;
					if(Incomming != Part)
					{
						CRC_pass = false;
					}
					CRC_sum = CRC_sum >> 8;
				}
				else
				{// Hier wird das zweite CRC-Byte verglichen
					uint8_t Part = CRC_sum & 0x00FF;
					if(Incomming != Part)
					{
						CRC_pass = false;
					}
				}				
				if(CRC_pass)
				{
					if(Count == 6)
					{// Wenn alles da ist und die Pruefsumme stimmt geht es hier raus
						Communication = false;
						*u8_Data1 = Datablock[2];
						*u8_Data2 = Datablock[3];
						*u8_Data3 = Datablock[4];
						*u8_Data4 = Datablock[5];
						m_Console->print("Command: 0x"); 
						m_Console->print(*Command, HEX);
						m_Console->print("; Data 1: 0x"); 
						m_Console->print(*u8_Data1, HEX);
						m_Console->print("; Data 2: 0x"); 
						m_Console->print(*u8_Data2, HEX);
						m_Console->print("; Data 3: 0x"); 
						m_Console->print(*u8_Data3, HEX);
						m_Console->print("; Data 4: 0x"); 
						m_Console->println(*u8_Data4, HEX);
						return 1;
					}
				}
				else
				{// Wenn die Pruefsumme nicht stimmt geht es hier raus mit Fehlercode
					Communication = false;
					*Command = 0;
					m_Console->print("CRC error");
					return -1;
				}
			}
			Count++;
		}
		else
		{// Hier beginnt der Empfang mit Arbitrierung
			if(Incomming == '@')
			{// Es muessen drei Arbitrierungszeichen einlaufen damit es weiter gehen kann
				Count++;
			}
			else
			{//Wenn kein Arbitrierungszeichen mehr kommt...
				if(Count > 2)
				{//...sollte das vierte Zeichen die Slaveadresse sein
					if (Incomming == *mp_SLA)
					{
						Communication = true;// Damit wird weiter oben der Datenempfang freigeschaltet
						m_Console->print("Own SLA received SLA = 0x");
						m_Console->println(Incomming, HEX);
						Datablock[0] = Incomming;
					}
					else
					{//...wenn nicht, dann ist der Rest auch egal und alles wird verworfen
					}
				}
				Count = 0;
			}
			
		}
		delay(10);// Die Schnittstelle arbeitet sehr langsam, (7ms / Byte) da kann zwischen den Bytes schon mal viel Zeit vergehen  
	}
	
	if(Communication)
	{// Sollte unerwarteter Weise ploetzlich nichts mehr kommen wird mit Fehlercode abgebrochen
		Communication = false;
		m_Console->print("Timeout error");
		return-2;
	}
	
	return 0;// Nix passiert, nix zu sehen, bitte gehen sie weiter
}

int8_t CellToMasterCom::SlaveAnswer(uint8_t Command, uint8_t u8_Data1, uint8_t u8_Data2, uint8_t u8_Data3, uint8_t u8_Data4)
{
	if(isMaster)
	{
		return -6;
	}
	
	m_crc.clearCrc();
	uint8_t Datablock[] = {*mp_SLA,Command,u8_Data1,u8_Data2,u8_Data3,u8_Data4};
	uint16_t CRC_sum = m_crc.XModemCrc(Datablock,0,6);
	uint8_t CRC1 = CRC_sum & 0x00FF;
	CRC_sum = CRC_sum >> 8;
	uint8_t CRC2 = CRC_sum & 0x00FF;
	
	UCSR0B |= (1<<TXEN0);//TX einschalten
	// delay(15);
	m_Console->print("Send new Data = 0x");
	m_Console->print('A', HEX);
	m_USART->write('A');//Dummy Byte zur Sicherheit
	// delay(15);
	m_Console->print(" 0x");
	m_Console->print('@', HEX);
	m_USART->write('@');//Beginn der Nachricht
	// delay(15);
	m_Console->print(" 0x");
	m_Console->print(*mp_SLA, HEX);
	m_USART->write(*mp_SLA);
	// delay(15);
	m_Console->print(" 0x");
	m_Console->print(Command, HEX);
	m_USART->write(Command);
	// delay(15);
	m_Console->print(" 0x");
	m_Console->print(u8_Data1, HEX);
	m_USART->write(u8_Data1);
	// delay(15);
	m_Console->print(" 0x");
	m_Console->print(u8_Data2, HEX);
	m_USART->write(u8_Data2);
	// delay(15);
	m_Console->print(" 0x");
	m_Console->print(u8_Data3, HEX);
	m_USART->write(u8_Data3);
	// delay(15);
	m_Console->print(" 0x");
	m_Console->print(u8_Data4, HEX);
	m_USART->write(u8_Data4);
	// delay(15);
	m_Console->print(" 0x");
	m_Console->print(CRC1, HEX);
	m_USART->write(CRC1);
	// delay(15);
	m_Console->print(" 0x");
	m_Console->print(CRC2, HEX);
	m_USART->write(CRC2);
	// delay(15);
	m_Console->print(" 0x");
	m_Console->println('A', HEX);
	m_USART->write('A');//Dummy Byte zur Sicherheit
	m_USART->flush();
	delay(5);
	UCSR0B &= ~(1<<TXEN0);
}

int8_t CellToMasterCom::SlaveAnswer(uint8_t Command, uint16_t u16_Data1, uint8_t u8_Data3, uint8_t u8_Data4)
{
	uint8_t LowTemp = u16_Data1 & 0x00FF;
	uint8_t HighTemp = (u16_Data1 >> 8) & 0x00FF;
	
	return SlaveAnswer(Command, HighTemp, LowTemp, u8_Data3, u8_Data4);
}

int8_t CellToMasterCom::ChangeSLA(uint8_t newSLA)
{
	if(isMaster)
	{
		return -6;
	}
	
	*mp_SLA = newSLA;
	eeprom_write_byte(mp_slaAdr, newSLA);
	eeprom_write_byte(mp_isConf, 0xAB);
	m_Console->print("new Slave address saved to EEPROM SLA = ");
	m_Console->println(*mp_SLA, HEX);
}

int8_t CellToMasterCom::CheckSLA(bool erase)
{
	if(isMaster)
	{
		return -6;
	}
	
	if(erase)
	{
		eeprom_write_byte(mp_isConf, 0xFF);
		m_Console->print("erase Slave address in EEPROM SLA = ");
		m_Console->println(*mp_SLA, HEX);
	}
	else
	{
		if(eeprom_read_byte(mp_isConf) == 0xAB)
		{
			*mp_SLA = eeprom_read_byte(mp_slaAdr);
			m_Console->print("load Slave address from EEPROM SLA = ");
			m_Console->println(*mp_SLA, HEX);
		}
		else
		{
			m_Console->print("no Slave address in EEPROM SLA = ");
			m_Console->println(*mp_SLA, HEX);
		}
	}
	
}

uint8_t CellToMasterCom::GetSLA()
{
	return *mp_SLA;
}

int8_t CellToMasterCom::ACK(int8_t ErrorCode)
{
	if(isMaster)
	{
		return -6;
	}
	
	if(ErrorCode != 0)
	{
		UCSR0B |= (1<<TXEN0);//TX einschalten
		// delay(15);
		m_Console->print("Send new ACK = 0x");
		m_Console->print('A', HEX);
		m_USART->write('A');//Dummy Byte zur Sicherheit
		// delay(15);
		m_Console->print(" 0x");
		m_Console->print('@', HEX);
		m_USART->write('@');//Beginn der Nachricht
		// delay(15);
		m_Console->print(" 0x");
		m_Console->print(*mp_SLA, HEX);
		m_USART->write(*mp_SLA);
		// delay(15);
		m_Console->print(" 0x");
		m_Console->print(ErrorCode);
		m_USART->write(ErrorCode);
		// delay(15);
		m_Console->print(" 0x");
		m_Console->println('A', HEX);
		m_USART->write('A');//Dummy Byte zur Sicherheit
		m_USART->flush();
		delay(5);
		UCSR0B &= ~(1<<TXEN0);//TX wieder ausschalten
	}
}

int8_t CellToMasterCom::CheckACK(uint8_t SLA)
{
	if(!isMaster)
	{
		return -5;
	}
	
	uint32_t Timestamp = millis();
	uint8_t received = 0;
	while(!m_USART->available())
	{
		if(Timestamp+400 < millis())
		{
			return -3;
		}
	}
	while(m_USART->available())
	{
		received = m_USART->read();
		delay(10);// Die Uebertragung eines Byte benoetigt gemessen bei 1200 boud ca. 8ms
		// m_Console->print("ACK rec: 0x");
		// m_Console->println(received, HEX);
		if(received == '@')
		{
			break;
		}
		
	}
	received = m_USART->read();
	delay(10);
	if(received != SLA)
	{
		// m_Console->print("ACK error wrong SLA: ");
		// m_Console->println(received, HEX);
		return -4;
	}
	else
	{
		// m_Console->print("ACK SLA: ");
		// m_Console->println(received, HEX);
	}
	Timestamp = millis();
	while(!m_USART->available())
	{
		if(Timestamp+200 < millis())
		{
			return -7;
		}
	}
	received = m_USART->read();
	delay(10);

	while(m_USART->available() && m_USART->peek() != '@')
	{//Restmuell abholen bis zum beginn einer neuen Nachricht ('@')
		m_USART->read();
		delay(15);
	}
	// m_Console->print("ACK errorcode: ");
	// m_Console->println(received);
	
	return received;
}

uint8_t CellToMasterCom::CheckAvailable()
{
	return m_USART->available();
}

/*PRIVATE: ----------------------------------------------------------------------------------------*/
void CellToMasterCom::Arbitration()
{
	m_USART->print("@@@");
}



#endif // CellToMasterCom_h
/*
Benutzt werden Codes zwischen -20 und 20 alle anderen sind reserviert fuer andere Kassen oder das Hauptprogramm
Fehlercodes:
		-10	: Datenpaket von falschem Kommando

		-9	: Datenpaket von falscher Slaveadresse

		-8	: Timeout beim Warten auf Datenpaket

		-7	: Timeout ACK unvollstaendig empfangen

		-6	: Funktion nur auf einem Slave-Device unterstuetzt
		
		-5	: Funktion nur auf einem Master-Device unterstuetzt

		-4	: ACK Fehler, die Slaveadresse stimmt nicht ueberein
		
		-3	: Timeout beim Warten auf ein ACK
		
		-2	: Transmission incomplete (Das Datagramm nicht vollstaendig uebertragen oder unzulaessiges Timeout zwischen den Bytes (> 10ms))
		
		-1	: CRC Error (Die Pruefsumme hat nicht gestimmt)
		
		1	: Datenuebertragung erfolgreich
*/











































