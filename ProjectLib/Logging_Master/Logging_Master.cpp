#include "Logging_Master.h"

LoggingClass::LoggingClass(DateAndTime* pTime, HardwareSerial* USART, uint8_t LoggingLevel)
{
	m_USART = USART;
	m_LoggingLevel = LoggingLevel;
	mp_Now = pTime;
	//m_USART->begin(115200); // geht nicht aus der klasse heraus
}

const String LoggingClass::Ziffer[] = {"0","1","2","3","4","5","6","7","8","9"};


void LoggingClass::Logging(uint8_t Priority, String statement)
{
	if(m_LoggingLevel < Priority)
	{return;}
	bool useFile = LoggMarker(Priority);
	m_USART->println(statement);

	if(useFile)
	{
		m_LoggFile.println(statement);
		m_LoggFile.close();
		m_LoggFile.close();
	}
}

void LoggingClass::Logging(uint8_t Priority, String statement, double value)
{
	if(m_LoggingLevel < Priority)
	{return;}
	bool useFile = LoggMarker(Priority);
	m_USART->print(statement + ": ");
	m_USART->println(value, 8);
	if(useFile)
	{
		m_LoggFile.print(statement + ": ");
		m_LoggFile.println(value, 8);
		m_LoggFile.close();
	}
}

void LoggingClass::Logging(uint8_t Priority, String statement, int8_t value)
{
	Logging(Priority, statement, (int32_t)value);
}

void LoggingClass::Logging(uint8_t Priority, String statement, int32_t value)
{
	if(m_LoggingLevel < Priority)
	{return;}
	bool useFile = LoggMarker(Priority);
	m_USART->print(statement + ": ");
	m_USART->println(value);
	if(useFile)
	{
		m_LoggFile.print(statement + ": ");
		m_LoggFile.println(value);
		m_LoggFile.close();
	}
}

void LoggingClass::Logging(uint8_t Priority, String statement, uint32_t value, uint8_t NumberSystem)
{
	if(m_LoggingLevel < Priority)
	{return;}
	bool useFile = LoggMarker(Priority);
	m_USART->print(statement + ": ");
	m_USART->println(value, NumberSystem);
	if(useFile)
	{
		m_LoggFile.print(statement + ": ");
		m_LoggFile.println(value, NumberSystem);
		m_LoggFile.close();
	}
}

void LoggingClass::Logging(uint8_t Priority, String statement, String value)
{
	if(m_LoggingLevel < Priority)
	{return;}
	bool useFile = LoggMarker(Priority);
	m_USART->print(statement + ": ");
	m_USART->println(value);
	if(useFile)
	{
		m_LoggFile.print(statement + ": ");
		m_LoggFile.println(value);
		m_LoggFile.close();
	}
}

void LoggingClass::Logging(uint8_t Priority, String statement, uint16_t value, uint8_t NumberSystem)
{
	if(m_LoggingLevel < Priority)
	{return;}
	Logging(Priority, statement, (uint32_t)value, NumberSystem);
}

void LoggingClass::Logging(uint8_t Priority, String statement, uint8_t value, uint8_t NumberSystem)
{
	if(m_LoggingLevel < Priority)
	{return;}
	Logging(Priority, statement, (uint32_t)value, NumberSystem);
}

void LoggingClass::Logging(uint8_t Priority, String statement, uint8_t value)
{
	if(m_LoggingLevel < Priority)
	{return;}
	Logging(Priority, statement, (int32_t) value);
}

void LoggingClass::Logging(uint8_t Priority, String statement, uint16_t value)
{
	if(m_LoggingLevel < Priority)
	{return;}
	Logging(Priority, statement, (uint16_t)value, (uint8_t)DEC);
}

void LoggingClass::Logging(uint8_t Priority, String statement, uint32_t value)
{
	if(m_LoggingLevel < Priority)
	{return;}
	Logging(Priority, statement, (uint32_t)value, (uint8_t)DEC);
}

void LoggingClass::Errorcode(uint16_t Command, uint16_t Errorcode)
{
	if(m_LoggingLevel < LEVEL_ERROR)
	{return;}
	
	bool useFile = LoggMarker(LEVEL_ERROR);
	
	m_USART->print("Command: ");
	m_USART->print(Command);
	m_USART->print(" has thrown error code: ");
	m_USART->println(Errorcode);
	if(useFile)
	{
		m_LoggFile.print("Command: ");
		m_LoggFile.print(Command);
		m_LoggFile.print(" has thrown error code: ");
		m_LoggFile.println(Errorcode);
		m_LoggFile.close();
	}
}

void LoggingClass::Errorcode(uint16_t Command, uint16_t Errorcode, uint32_t Value)
{
	if(m_LoggingLevel < LEVEL_ERROR)
	{return;}
	
	bool useFile = LoggMarker(LEVEL_ERROR);
	
	m_USART->print("Command: ");
	m_USART->print(Command);
	m_USART->print(" has thrown error code: ");
	m_USART->print(Errorcode);
	m_USART->print(" Invalid Value: ");
	m_USART->println(Value);
	if(useFile)
	{
		m_LoggFile.print("Command: ");
		m_LoggFile.print(Command);
		m_LoggFile.print(" has thrown error code: ");
		m_LoggFile.print(Errorcode);
		m_LoggFile.print(" Invalid Value: ");
		m_LoggFile.println(Value);
		m_LoggFile.close();
	}
}

bool LoggingClass::LoggMarker(uint8_t Priority)
{
	bool useFile = false;
	if(m_useSD)
	{
		m_LoggFile = mp_SDCard->open(m_LoggFileName, FILE_WRITE);
		if (m_LoggFile)
		{// if the file is available
			useFile = true;
			m_LoggFile.print(mp_Now->getDateTime());
		}
	}
	uint8_t CharCount = m_USART->print(mp_Now->getDateTime());
	do
	{
		CharCount++;
		m_USART->print(" ");
		if(useFile)
		m_LoggFile.print(" ");
	}while(CharCount < 20);
	
	if(Priority == LEVEL_ERROR)
	{
		m_USART->print("[ERR]  ");
		if(useFile)
		m_LoggFile.print("[ERR]  ");
		
	}
	else if(Priority == LEVEL_FSM)
	{
		m_USART->print("[FSM]  ");
		if(useFile)
		m_LoggFile.print("[FSM]  ");
		
	}
	else if(Priority == LEVEL_INFO)
	{
		m_USART->print("[INF]  ");
		if(useFile)
		m_LoggFile.print("[INF]  ");
		
	}
	else if(Priority == LEVEL_MAIN)
	{
		m_USART->print("[MAI]  ");
		if(useFile)
		m_LoggFile.print("[MAI]  ");
		
	}
	else if(Priority == LEVEL_DEBUG)
	{
		m_USART->print("[DEB]  ");
		if(useFile)
		m_LoggFile.print("[DEB]  ");
		
	}
	else
	{
		m_USART->print("[   ]  ");
		if(useFile)
		m_LoggFile.print("[   ]  ");
	}
	if(useFile)
	{
		m_USART->print("[SD]  ");
	}

	return useFile;
}

bool LoggingClass::SdPresent()
{
	return m_useSD;
}

bool LoggingClass::CheckCard(uint8_t CS_Pin)
{
	if(!m_card.init(SPI_HALF_SPEED, CS_Pin))
	{
		Logging(LEVEL_FSM, "SD card is not present");
		m_useSD = false;
		return false;
	}
	else
	{
		Logging(LEVEL_FSM, "SD card is present");
		switch (m_card.type()) 
		{
			case SD_CARD_TYPE_SD1:
				m_CardType = "SD1";
			break;
			case SD_CARD_TYPE_SD2:
				m_CardType = "SD2";
			break;
			case SD_CARD_TYPE_SDHC:
				m_CardType = "SDHC";
			break;
			default:
				m_CardType = "Unknown";
		}
		Logging(LEVEL_FSM, "Card type" , m_CardType);
	}
	
	if(!m_volume.init(m_card))
	{
		Logging(LEVEL_FSM, "Could not find FAT16/FAT32. Card formatted?");
		m_useSD = false;
		return false;
	}
	else
	{
		uint32_t volumesize;
		m_FileSystem = m_volume.fatType();
		Logging(LEVEL_FSM, "Volume type is FAT", m_FileSystem);

		volumesize = m_volume.blocksPerCluster();    // clusters are collections of blocks
		volumesize *= m_volume.clusterCount();       // we'll have a lot of clusters
		volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
		m_Free_kB = volumesize;
		volumesize /= 1024;
		Logging(LEVEL_FSM, "Volume size (MB)", volumesize);
		m_VolSize = volumesize;
	}
	return true;
}

void LoggingClass::Init(uint8_t CS_Pin, SDClass* pSDCard)
{
	if(CheckCard(CS_Pin))
	{
		mp_SDCard = pSDCard;
		m_CS_Pin = CS_Pin;
		if(!mp_SDCard->begin(CS_Pin)) 
		{
			Logging(LEVEL_FSM, "Card failed, or not present");
			return;
		}
		else
		{
			Logging(LEVEL_FSM, "card initialized.");
		}
		uint8_t Counter = 1;
		uint32_t F_Size_Byte;
		String FileBaseName = "Logg";
		String FileNumber = "00" + String(Counter);
		String FileName = FileBaseName + FileNumber + ".txt";
		String FileExisting;
		
		while(mp_SDCard->exists(FileName)) 
		{
			File TmpFile = mp_SDCard->open(FileName, FILE_READ);
			F_Size_Byte = TmpFile.size();
			Logging(LEVEL_INFO, FileName + " found. File size (Byte): ", F_Size_Byte);
			m_Free_kB -= 1 + F_Size_Byte / 1024;
			TmpFile.close();
			
			Counter++;
			if(Counter == 0)
			{
				Logging(LEVEL_FSM, "Cant create filenumber > 255");
				return;
			}
			if(Counter < 10)
			{
				FileNumber = "00" + String(Counter);
			}
			else if(Counter < 100)
			{
				FileNumber = "0" + String(Counter);
			}
			else
			{
				FileNumber = String(Counter);
			}
			FileExisting = FileName;
			FileName = FileBaseName + FileNumber + ".txt";
		}
		if(F_Size_Byte > m_MaxFileSize || Counter == 1)
		{// open a new file and immediately close it:
			Logging(LEVEL_FSM, "Creating " + FileName);
			m_LoggFile = mp_SDCard->open(FileName, FILE_WRITE);
			m_LoggFile.close();
			m_LoggFileName = FileName;
		}
		else
		{
			Logging(LEVEL_FSM, "Open " + FileExisting);
			m_LoggFile = mp_SDCard->open(FileExisting, FILE_WRITE);
			while (m_LoggFile.available())
			{//Filepointer ans Ende bewegen
				m_LoggFile.read();
			}
			m_LoggFile.println("\nXXXXX Loggfile reopen XXXXX");
			m_LoggFile.close();
			m_LoggFileName = FileExisting;
		}
		m_useSD = true;
	}
}

bool LoggingClass::Reopen()
{
	if(CheckCard(m_CS_Pin))
	{
		uint8_t Counter = 1;
		uint32_t F_Size_Byte;
		String FileBaseName = "Logg";
		String FileNumber = "00" + String(Counter);
		String FileName = FileBaseName + FileNumber + ".txt";
		String FileExisting;
		
		while(mp_SDCard->exists(FileName)) 
		{
			File TmpFile = mp_SDCard->open(FileName, FILE_READ);
			F_Size_Byte = TmpFile.size();
			Logging(LEVEL_INFO, FileName + " found. File size (Byte): ", F_Size_Byte);
			m_Free_kB -= 1 + F_Size_Byte / 1024;
			TmpFile.close();
			
			Counter++;
			if(Counter == 0)
			{
				Logging(LEVEL_FSM, "Cant create filenumber > 255");
				return;
			}
			if(Counter < 10)
			{
				FileNumber = "00" + String(Counter);
			}
			else if(Counter < 100)
			{
				FileNumber = "0" + String(Counter);
			}
			else
			{
				FileNumber = String(Counter);
			}
			FileExisting = FileName;
			FileName = FileBaseName + FileNumber + ".txt";
		}
		if(F_Size_Byte > m_MaxFileSize || Counter == 1)
		{// open a new file and immediately close it:
			Logging(LEVEL_FSM, "Creating " + FileName);
			m_LoggFile = mp_SDCard->open(FileName, FILE_WRITE);
			m_LoggFile.close();
			m_LoggFileName = FileName;
		}
		else
		{
			Logging(LEVEL_FSM, "Open " + FileExisting);
			m_LoggFile = mp_SDCard->open(FileExisting, FILE_WRITE);
			while (m_LoggFile.available())
			{//Filepointer ans Ende bewegen
				m_LoggFile.read();
			}
			m_LoggFile.println("\nXXXXX Loggfile reopen XXXXX");
			m_LoggFile.close();
			m_LoggFileName = FileExisting;
		}
		m_useSD = true;
		return true;
	}
	return false;
}

String LoggingClass::getLastLogg(String* Logg2nd)//
{
	String Output1;
	if(m_useSD)
	{
		uint32_t CharCount = 0;
		m_LoggFile = mp_SDCard->open(m_LoggFileName, FILE_WRITE);
		if(m_LoggFile.position() > 200)
		{
			m_LoggFile.seek(m_LoggFile.position() - 200);
		}
		else
		{
			m_LoggFile.seek(0);
		}
		while (m_LoggFile.available() && m_LoggFile.read() != 0x0A)//0x0D = CR, 0x0A = LF
		{//Filepointer bis zum naechsten Zeilenende bewegen
			continue;
		}
		CharCount = m_LoggFile.position();
		while (m_LoggFile.available())
		{//
			char n = m_LoggFile.read();
			Output1 += n;
			if(n == 0x5D)//0x5D = ']'
			{	
				Output1 += (char)0x0D;
				Output1 += (char)0x0A;
			}
		}
		
		m_LoggFile.seek(CharCount);
		if(m_LoggFile.position() > 200)
		{
			m_LoggFile.seek(m_LoggFile.position() - 200);
		}
		else
		{
			m_LoggFile.seek(0);
		}
		while (m_LoggFile.available() && m_LoggFile.read() != 0x0A)//0x0D = CR, 0x0A = LF
		{//Filepointer bis zum naechsten Zeilenende bewegen
			continue;
		}
		while (m_LoggFile.available() && m_LoggFile.position() < CharCount-2)
		{//
			char n = m_LoggFile.read();
			*Logg2nd += n;
			if(n == 0x5D)//0x5D = ']'
			{	
				*Logg2nd += (char)0x0D;
				*Logg2nd += (char)0x0A;
			}
		}
		m_LoggFile.close();
	}
	else
	{
		Output1 = *Logg2nd = "No SD card available";
	}
	return Output1;
}

void LoggingClass::CheckFileSize()
{
	m_LoggFile = mp_SDCard->open(m_LoggFileName, FILE_WRITE);
	uint32_t Size = m_LoggFile.size();
	m_LoggFile.close();
	if(Size > m_MaxFileSize)
	{
		Logging(LEVEL_FSM, "Maximum File Size reached", "Create new File");
		unmountSD();
		Reopen();
	}
}
































