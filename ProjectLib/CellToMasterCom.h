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
[Slaveadresse 1 Byte]  [Fehlercode 1 Byte]
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


class CellToMasterCom
{
public:
	HardwareSerial* m_USART;
	SoftwareSerial* m_Console;
	
	CellToMasterCom(HardwareSerial* USART, void* Console);// Master Constructor
	CellToMasterCom(HardwareSerial* USART, SoftwareSerial* Console, uint8_t* p_SLA);// Slave Constructor
	
	int8_t MasterOrder(uint8_t SLA, uint8_t Command, uint8_t u8_Data1 = 0, uint8_t u8_Data2 = 0, uint8_t u8_Data3 = 0, uint8_t u8_Data4 = 0);
	// int8_t MasterOrder(uint8_t SLA, uint8_t Command, uint16_t u16_Data1, uint8_t u8_Data3 = 0, uint8_t u8_Data4 = 0);
	// int8_t MasterOrder(uint8_t SLA, uint8_t Command, uint16_t u16_Data1, uint16_t u1_Data2);
	// int8_t MasterOrder(uint8_t SLA, uint8_t Command, uint32_t u32_Data);
	// int8_t SlaveAnswer(uint8_t SLA, uint8_t Command, uint8_t u8_Data1, uint8_t u8_Data2 = 0, uint8_t u8_Data3 = 0, uint8_t u8_Data4 = 0);
	// int8_t SlaveAnswer(uint8_t SLA, uint8_t Command, uint16_t u16_Data1, uint8_t u8_Data3 = 0, uint8_t u8_Data4 = 0);
	// int8_t SlaveAnswer(uint8_t SLA, uint8_t Command, uint16_t u16_Data1, uint16_t u1_Data2);
	// int8_t SlaveAnswer(uint8_t SLA, uint8_t Command, uint32_t u32_Data);
	int8_t SlaveReceive(uint8_t* Command, uint8_t* u8_Data1, uint8_t* u8_Data2, uint8_t* u8_Data3, uint8_t* u8_Data4);;
	// int8_t SlaveReceive(uint8_t Command, uint16_t* u16_Data1, uint8_t* u8_Data3, uint8_t* u8_Data4);
	// int8_t SlaveReceive(uint8_t Command, uint16_t* u16_Data1, uint16_t* u1_Data2);
	// int8_t SlaveReceive(uint8_t Command, uint32_t* u32_Data);
	int8_t ChangeSLA(uint8_t newSLA);
	int8_t CheckSLA(bool erase);
	int8_t ACK(int8_t ErrorCode);
//	int8_t Logg()
	
private:
	Crc16 m_crc;
	uint8_t* mp_SLA;
	const uint8_t* mp_isConf = 1;
	const uint8_t* mp_slaAdr = 2;
	bool isMaster;
	
	void Arbitration();
	int8_t CheckACK(uint8_t SLA);
	
};

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
	
	return CheckACK(SLA);
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

int8_t CellToMasterCom::ACK(int8_t ErrorCode)
{
	if(isMaster)
	{
		return -6;
	}
	
	if(ErrorCode != 0)
	{
		UCSR0B |= (1<<TXEN0);
		// delay(15);
		m_Console->print("Send ACK = 0x");
		m_Console->print('A', HEX);
		m_USART->write('A');
		// delay(15);
		m_Console->print(" 0x");
		m_Console->print('@', HEX);
		m_USART->write('@');
		// delay(15);
		m_Console->print(" 0x");
		m_Console->print(*mp_SLA, HEX);
		m_USART->write(*mp_SLA);
		// delay(15);
		m_Console->print(" 0x");
		m_Console->println(ErrorCode, HEX);
		m_USART->write(ErrorCode);
		// delay(15);
		UCSR0B &= ~(1<<TXEN0);
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
		if(Timestamp+500 < millis())
		{
			return -3;
		}
	}
	while(m_USART->available())
	{
		received = m_USART->read();
		delay(30);
		// m_Console->print("ACK rec 0x");
		// m_Console->println(received, HEX);
		if(received == '@')
		{
			break;
		}
		
	}
	received = m_USART->read();
	delay(30);
	if(received != SLA)
	{
		// m_Console->print("ACK error wrong SLA - ");
		// m_Console->println(received, HEX);
		return -4;
	}
	Timestamp = millis();
	while(!m_USART->available())
	{
		if(Timestamp+200 < millis())
		{
			return -3;
		}
	}
	received = m_USART->read();
	delay(30);

	while(m_USART->available())
	{//Restmuell abholen
		m_USART->read();
		delay(30);
	}
	
	return received;
}


void CellToMasterCom::Arbitration()
{
	m_USART->print("@@@");
}



#endif // CellToMasterCom_h
/*
Fehlercodes:
		-6	: Funktion nur auf einem Slave-Device unterstuetzt
		
		-5	: Funktion nur auf einem Master-Device unterstuetzt

		-4	: ACK Fehler, die Slaveadresse stimmt nicht ueberein
		
		-3	: Timeout beim Warten auf ein ACK
		
		-2	: Transmission incomplete (Das Datagramm nicht vollstaendig uebertragen oder unzulaessiges Timeout zwischen den Bytes (> 10ms))
		
		-1	: CRC Error (Die Pruefsumme hat nicht gestimmt)
		
		1	: Datenuebertragung erfolgreich
*/











































