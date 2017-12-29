
#ifndef Display_h
#define Display_h

#include <Arduino.h>
#include "Logging_Master.h"

class DisplayClass
{
	public:
		DisplayClass(HardwareSerial* p_DisplaySerial, HardwareSerial* p_ConsoleSerial);
		void WriteMaintime(DateAndTime Time);
		void WriteTextElement(String Screen, String Element, String Text);
		void WriteNumElement(String Screen, String Element, uint32_t Value);
		uint32_t CheckIncome();
		uint32_t GetData(uint8_t Number);
		void sdReady(bool ready);
		
	
	private:
		HardwareSerial* mp_Display;
		HardwareSerial* mp_Console;
		DateAndTime m_Time;
		uint32_t Data[10];
		
		void Arbitration();
};

DisplayClass::DisplayClass(HardwareSerial* p_DisplaySerial, HardwareSerial* p_ConsoleSerial)
{
	mp_Display = p_DisplaySerial;
	mp_Console = p_ConsoleSerial;
}

void DisplayClass::WriteMaintime(DateAndTime Time)
{
	m_Time = Time;
	WriteTextElement("Home", "t0", m_Time.getWeekday() + " " + m_Time.getDate() + " " 
	+ m_Time.getTime());
}

void DisplayClass::WriteTextElement(String Screen, String Element, String Text)
{
	Arbitration();
	mp_Display->print(Screen + "." + Element + ".txt=\"" + Text + "\"");
	Arbitration();
}

void DisplayClass::WriteNumElement(String Screen, String Element, uint32_t Value)
{
	Arbitration();
	mp_Display->print(Screen + "." + Element + ".val=" + String(Value));
}

uint32_t DisplayClass::CheckIncome()
{
	uint32_t retVal = 0;
	if(mp_Display->available())
	{
		uint8_t i = 0;
		
		for(uint8_t k = 0; k < 10; k++)
		{
			Data[k] = 0;
		}
		
		// mp_Console->print("Nextion Says: ");
		while(mp_Display->available())
		{
			uint8_t income = mp_Display->read();
			// mp_Console->print(income, HEX);
			// mp_Console->print(" ");
			if(i<4)
			{
				retVal += income << 8*i;
				
			}
			else if(i < 44)
			{
				Data[(i-4)/4] += income << 8*(i%4);
			}
			else
			{
				continue;
			}
			delay(1);
			i++;
		}
		// mp_Console->println();
		
		// mp_Console->print("Command: ");
		// mp_Console->print(retVal, HEX);
		// mp_Console->print(" Data: ");
		// for(uint8_t j = 0; j < 10; j++)
		// {
			// mp_Console->print(Data[j], HEX);
			// mp_Console->print(" ");
		// }
		// mp_Console->println();
	}
	return retVal;
}

uint32_t DisplayClass::GetData(uint8_t Number)
{
	if(Number>9)
	{
		return 0;
	}
	
	return Data[Number];
}

void DisplayClass::sdReady(bool ready)
{
	Arbitration();
	uint8_t x = 4;
	if(ready)
	{	x = 5;
	}
	mp_Display->print("Home.p0.pic=" + String(x));
}




























void DisplayClass::Arbitration()
{
	for(uint8_t i = 0; i < 3; i++)
	{
		mp_Display->write(0xff);
	}
}

#endif // Display_h