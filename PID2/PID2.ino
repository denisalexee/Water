/***********************************************************

All works! Добавлен попеременный показ через 5 секунд смены 
мощности выхода необходимой температурой.

***********************************************************/


#include <OneWire.h>
#include <PID_v1.h>								// http://playground.arduino.cc/Code/PIDLibrary
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <Wire.h>
#include <DS1307.h>								// http://goo.gl/Qn0sDw
DS1307 rtc(4, 5);								// RTC на 4 (SDA) и 5 (SCL) ЦИФРОВОЙ!!! ноге;
Time  t;										// Переменная для чтения в нее времени;
byte dayOfWeek, dayOfWeekLast;					// Переменные для запоминания дня недели;
// Инициализируем объект-экран, передаём использованные
// для подключения контакты в порядке:
LiquidCrystal lcd(6, 7, 8, 9, 10, 11);			// RS, E, DB5, DB6, DB7, DB8;

// Установка переменных PID;
double Setpoint, Input, Output;
double aggKp=4, aggKi=0.2, aggKd=1;				// Определяем агрессивные и консервативные параметры PID;
double consKp=1, consKi=0.05, consKd=0.25;
// Определяем указатели и другие параметры PID;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
double gap;										// Переключатель параметров PID;

OneWire  ds(2);									// DS18S20 на ноге 2 и все для него;
byte addr[1][8];
double tempToPID;
int HighByte, LowByte, TReading, Tc_100, Whole, Fract;
char buf[20];
int timeWhenAskTemperature;
bool askTemperature = 0;

int timeWhenBottonPressed;						// Переменные для работы со временем;
int millisNow, timeNow;
byte timePowerFlagStart=0;
byte secNow;
byte printSwitch = 0;

/*************   Режим работы устройства **********************/
byte modeOfOperation = 0;	 					// 0 - автоматический режим, 1 - вкл, 2 - выключен;
byte outPercente;								// Переменная для вывода мощности на вентиляторах;
int eepromAddr = 0;								// Переменная для записи режима и температуры в EEPROM;

enum switchVariants {							// Определения для переключателя в главном цикле;
	READ_TEMPERATURE,
	READ_TIME,
	CHECK_BOTTON,
	THROW_FLAG,
	CHECK_DAY,
	PID_SWITCH_OFF,
	PID_WORK,
};
switchVariants switchPointer = READ_TEMPERATURE;	// С чего начнем цикл;

enum bottonVariants {							// Определения для работы с кнопкой;
	BOTTON_SHORT,
	BOTTON_LONG,
	BOTTON_NOT,
};
bottonVariants botton = BOTTON_NOT;

void setup()
{
	DDRD|= (1<<DDD3);					// Инициализация ножки D3, где сидит вывод ШИМ: Bit3=Out

	// Инициализация таймера 2
	// Источник счета: системная шина
	// Частота: 16000,000 kHz
	// Режим: ШИМ с коррекцией фазы, счет до 255
	// Выход D3: Неинвертированный ШИМ
	// Период таймера: 0,031875 ms
	// Начальный выход(s):
	// Период D3 : 0,031875 ms Ширина импульса: 0 us
	
	
	ASSR=(0<<EXCLK) | (0<<AS2);
	TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (1<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (1<<WGM20);
	TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (1<<CS20);
	TCNT2=0x00;
	OCR2A=0x00;
	OCR2B=0x00;
	
	rtc.halt(false);							// Запуск часов;
	// Установка часов реального времени, если нужно:
	//rtc.setDOW(WEDNESDAY);					// Установка дня недели,
	//rtc.setTime(12, 0, 0);					// Установка времени 12:00:00 (24hr формат),
	//rtc.setDate(3, 10, 2010);					// Установка даты 3 Октябрь, 2010 г.

	lcd.begin(16, 2);							// Проверим ЖК;
	lcd.print("PID Regulator v4");
	lcd.setCursor(0, 1);
	lcd.print(" by Jiraffe :-)");

	delay(2000);
	lcd.clear();								// Протрем стекло ЖК индикатору;
	
	ds.search(addr[0]);							// Найдем и запомним адрес DS18B20;

	myPID.SetMode(AUTOMATIC);					// Включим PID и установим режим;

	pinMode(12, INPUT);							// Нога 12 - здесь висит кнопка;
	pinMode(13, OUTPUT);						// Чтобы видеть нажатие кнопки будем зажигать диод на плате;
	
	modeOfOperation = EEPROM.read(eepromAddr);	// Прочитаем из памяти режим работы;
	if (modeOfOperation >2) {					// Если там чушь (первый запуск) - уйдем в автоматический;
		modeOfOperation = 0;
	}
	Setpoint = (EEPROM.read(eepromAddr+1))+100;	// Прочитаем из памяти необходимую температуру;
	// поскольку там БАЙТ (знач. менее 255) - добавим сотню.
	if (Setpoint>295 || Setpoint < 220) {		// Если там чушь, установим 24,5 градуса.
		Setpoint = 245;
	}
	
	lcd.setCursor(0, 0);						// Опишем обстановку на ЖК;
	lcd.print("Mode: ");
	
	switch (modeOfOperation) {
		case 0:
		lcd.print("  Auto");
		break;
		case 1:
		lcd.print("Switch On!");
		break;
		case 2:
		lcd.print("Switch Off");
		break;
	}
	
	lcd.setCursor(0, 1);
	lcd.print("Set Temp: ");
	lcd.print(Setpoint/10);
	lcd.print("\x99");
	
	delay(3000);
	lcd.clear();
}

void loop()
{
	switch (switchPointer)							// Все делаем в одном операторе и одной функции;
	{
		
		/******************************  Измерить температуру  **********************************/
		case READ_TEMPERATURE:
		byte i;
		byte data[12];
		millisNow = millis();						// Узнаем текущее время;
		
		if ((millisNow - timeWhenAskTemperature) < 0) // Если таймер перешел через ноль, не даем зависнуть;
		{
			timeWhenAskTemperature = millisNow;
		}
		
		if (!askTemperature) {						// Если не запрашивали температуру с датчика - запросим;
			
			ds.reset();								// Ниже - стандартная процедура чтения датчика;
			ds.select(addr[0]);
			ds.write(0x44,1);						// Старт расчетов датчика;
			askTemperature = true;					// Подъем флага - температура запрошена!
			timeWhenAskTemperature = millisNow;		// Запомним время запроса температуры;
		}
		
		// Нужна задержка для расчета 850 миллисекунд, алгоритм ниже ее реализует;
		
		if (askTemperature & (millisNow-timeWhenAskTemperature) >850) {	// Если температура запрошена и прошло более 850 мс
			ds.reset();
			ds.select(addr[0]);
			ds.write(0xBE);							// Указание на будущее чтение;
			
			for ( i = 0; i < 9; i++) {				// Читаем 9 bytes;
				data[i] = ds.read();
			}
			
			LowByte = data[0];
			HighByte = data[1];
			TReading = (HighByte << 8) + LowByte;
			Tc_100 = (6 * TReading) + TReading / 4;	// Стандартная операция пересчета для DS18b20 (100 * 0.0625) или 6.25;
			tempToPID = (double) Tc_100/100;
			
			lcd.setCursor(0, 0);					// Печатаем на ЖК;
			lcd.print(tempToPID);
			lcd.print("\x99""c");
			
			Input = tempToPID*10;					// Пишем текущую (тепературу х 10) в переменную для передачи в PID;
			askTemperature = false;					// Сбросили флаг о запросе температуры;
		}
		
		switchPointer = READ_TIME;						// Шагаем дальше;
		
		break;

		/******************************  Считать время и напечатать его  ******************************/
		
		case READ_TIME:
		
		t = rtc.getTime();						// Читаем комплект информации о времени;
		dayOfWeek = (byte) t.dow;				// Пишем в переменную значение дня недели для передачи по этапу;
		secNow = (byte) t.sec;
		if (modeOfOperation < 2) {
			lcd.setCursor(0, 1);				// Печатаем время на ЖК;
			lcd.print(rtc.getTimeStr());
			lcd.print(" ");
			lcd.print(t.date, DEC);
			lcd.print("/");
			lcd.print(rtc.getMonthStr(FORMAT_SHORT));
			lcd.print(" ");						// Почистим 14(15) знакоместо;
			
			
		}
		else {
			lcd.setCursor(8, 0);				// Печатаем время на ЖК;
			lcd.print(rtc.getTimeStr());
			
			lcd.setCursor(0, 1);
			lcd.print("   ");
			lcd.print(rtc.getDateStr());
			lcd.print(" ""\x8D");				// Почистим 14(15) знакоместо;
			
		}
		
		switchPointer = CHECK_BOTTON;			// Идем дальше;
		
		break;
		
		
		/****************************  Проверить нажатие кнопки и режим СТОП ************************************/
		case CHECK_BOTTON:
		
		if (digitalRead(12) == 1) {				// Стандартная проверка с антидребезгом;
			digitalWrite(13,1);					// Зажжем диод на плате для наглядности;
			delay(50);							// Ждем для устранения дребезга;
			
			if (digitalRead(12) == 1) {			// Проверяем еще раз;
				timeWhenBottonPressed = millis();	// Запоминаем время нажатия кнопки;
				lcd.clear();						// Пишем, что занялись изменением режима;
				lcd.print("  Change Mode!");
				lcd.setCursor(0, 1);
				lcd.print("Or wite for 2 s.");
				while (digitalRead(12) == 1) {		// Ждем отпускания кнопки;
					timeNow = millis();				// И засекаем время;
					if (timeNow - timeWhenBottonPressed >2000) {	// Если две секунды не отпустили,
						lcd.setCursor(0, 0);
						lcd.clear();				// Чистим ЖК и обещаем изменить температуру;
						lcd.print("  Change Temp!");
						delay(200);
					}
				}
				digitalWrite(13,0);					// Гасим диод на плате;
				
				if (timeNow - timeWhenBottonPressed > 2000) {
					botton = BOTTON_LONG;			// Было долгое нажатие кнопки - больше 2 с.
				}
				else {
					botton = BOTTON_SHORT;			// Было короткое нажатие - менее 2 с.
				}
			}
		}
		
		switch (botton) {						// Здесь выбираем что менять;
			
			case BOTTON_LONG:					// При долгом нажатии,
			changeTemperature();			// Идем в подпрограмму смены температуры;
			EEPROM.write(eepromAddr+1, (byte) (Setpoint-100));	// По возвращению пишем температуру в память,
			// за минусом 100 - в память (байт) записывается число максимум 255
			botton = BOTTON_NOT;
			break;
			
			case BOTTON_SHORT:					// При коротком нажатии;
			
			modeOfOperation++;				// Следующий режим работы;
			if (modeOfOperation > 2) {		// Если выщли за границы дозволенного режима,
				modeOfOperation = 0;		// возвращаемся;
			}
			if (dayOfWeek>5 && modeOfOperation == 1) {	// Если суббота или воскресенье
				modeOfOperation=2;			// Пропускаем режим принудительного запуска;
			}
			EEPROM.write(eepromAddr, (byte) modeOfOperation);	// Пишем режим в память;
			lcd.clear();
			botton = BOTTON_NOT;
			break;
			
			case BOTTON_NOT:
			break;
		}

		
		if (modeOfOperation == 2) {
			switchPointer = READ_TEMPERATURE;			// Если установился/установлен режим СТОП - уходим в начало;
			OCR2B=0x00;									// Пишем ноль в регистр сравнения Таймера - останавливаем ШИМ
		}
		
		else switchPointer = THROW_FLAG;				// Нет режима СТОП - идем дальше;
		
		break;
		
		/***************************  Сбросить флаг принудительной работы ********************************/
		case THROW_FLAG:								// Если в средине недели система была принудительно запущена,
		// или в выходные это произошло случайно, она перейдет в автоматический режим
		// с началом субботы или понедельника;
		if ((dayOfWeek == 6 && dayOfWeekLast == 5) || (dayOfWeek == 1 && dayOfWeekLast == 7)  ) {
			modeOfOperation = 0;
			EEPROM.write(eepromAddr, (byte) modeOfOperation);	// Пишем режим в память;
		}
		dayOfWeekLast = dayOfWeek;						// Запомним прошлое значение дня;
		switchPointer = CHECK_DAY;						// Шагаем дальше;
		
		break;
		
		/*************************** Проверить день недели и решить что делать ************************/
		case CHECK_DAY:
		if (dayOfWeek == 6 || dayOfWeek == 7 || modeOfOperation == 1) {
			switchPointer = PID_WORK;
		}
		else switchPointer = PID_SWITCH_OFF;
		
		break;
		
		/************************ Остановить PID ******************************************************/
		case PID_SWITCH_OFF:
		//analogWrite(3,0);					// Останавливаем ШИМ;
		OCR2B=0x00;							// Пишем ноль в регистр сравнения Таймера - останавливаем ШИМ
		
		lcd.setCursor(7,0);					// Извещаем об этом ЖК: Стоим (Stop): (W)Рабочй (D)День.
		lcd.print("  Stop:WD");
		switchPointer = READ_TEMPERATURE;
		break;
		
		/************************** Запустить PID управление ******************************************/
		case PID_WORK:
		gap = abs(Setpoint-Input);			// Разность с желаемой температурой (умноженная на 10, т.е. 0.5 градуса);
		if(gap<5) {							// Мы недалеко и используем консервативные параметры;
			myPID.SetTunings(consKp, consKi, consKd);
		}
		
		else {								// Мы далеко, используем агрессивные параметры;
			myPID.SetTunings(aggKp, aggKi, aggKd);
		}
		
		myPID.Compute();
			
		if (Output>25) {					// Чтобы лишний раз не дергать моторки, здесь - 10% мощности
			OCR2B=Output;					// Пишем число в регистр сравнения таймера - заполение ШИМ
		}
		else {
			OCR2B=0;
		}
		
		outPercente = (byte)(Output/255*100);// Считаем в процентах мощность вывода ШИМ;
						
		if (!(secNow%5) && timePowerFlagStart ) {
		
			if (printSwitch == 0) {
				printSwitch = 1;
			}
			
			else {
				printSwitch = 0;
			}
			
			timePowerFlagStart =0;
		}
		if (secNow%5) {
			timePowerFlagStart = 1;
		}
		
				switch (printSwitch )
									{
										case 0:
											if (outPercente<100 && outPercente >9) {// Чистим знакоместа при переходе выхода от 100 до < 10
												lcd.setCursor(15,0);
												lcd.print(" ");
											}
	
											if (outPercente<10) {
												lcd.setCursor(14,0);
												lcd.print("  ");
											}
											lcd.setCursor(8,0);					// Печатаем на ЖК;
											lcd.print("Out=");
											lcd.setCursor(12,0);
											lcd.print(outPercente);
											lcd.print("%");
											
										break;
				
										case 1:
											lcd.setCursor(8,0);					// Печатаем на ЖК;
											lcd.print("Get ");
											lcd.setCursor(12,0);
											lcd.print(Setpoint/10);
											lcd.print("%");
															
										break;
				
										default:
										break;
									}	
			
		
		switchPointer = READ_TEMPERATURE;	// Уходим в начало цикла;
		break;
		
		/****************************** По умолчанию ****************************************************/
		default:
		switchPointer = READ_TEMPERATURE;
		break;
	}
}


void changeTemperature (void) {					// Подпрограмма смены референсной температуры;
	byte a = 1;									// Переменная, чтобы зациклить чтение кнопки;
	
	lcd.clear();								// Напечатем, какую температуру держим сейчас;
	lcd.print("Set Temperature");
	lcd.setCursor(0, 1);
	lcd.print("Now is: ");
	lcd.print(Setpoint/10);
	
	while (a) {
		
		if (digitalRead(12) == 1) {				// Стандартная проверка с антидребезгом;
			digitalWrite(13,1);					// Зажжем диод на плате для наглядности;
			delay(50);							// Ждем для устранения дребезга;
			
			if (digitalRead(12) == 1) {			// Проверяем еще раз;
				timeWhenBottonPressed = millis(); // Запоминаем время;
			}
			
			do {								// Пока нажата кнопка;
				timeNow = millis();				// Читаем время;
				if ((timeNow - timeWhenBottonPressed) > 2000) { // И если одо дольше 2 с.
					lcd.clear();				// Обещаем ЖК выйти из режима изменения температуры;
					lcd.setCursor(0, 0);
					lcd.print("      EXIT!     ");
					delay(200);
				}
			} while (digitalRead(12) == 1);
			
			digitalWrite(13,0);					// Гасим диод на плате;
			
			if ((timeNow - timeWhenBottonPressed) > 2000) { // Если нажатие было больше 2 с
				a = 0;							// Выходим из цикла и валим из подпрограммы;
			}
			else {								// Иначе
				Setpoint +=5;					// Увеличиваем температуру на 0.5 градуса;
				if (Setpoint == 300) {			// И если достигли 30 градусов
					Setpoint = 220;				// Начинаем заново, с 22 градусов;
				}
				lcd.setCursor(8, 1);			// Известим ЖК о достигнутых результатах;
				lcd.print(Setpoint/10);
			}
		}
	}
	
	lcd.clear();								// Почистимся и назад;
	return;
}