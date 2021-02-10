/*
 * Управление краном по температуре воды
 * Сделано ПИД регулирование.
 */
#include <OneWire.h>            //Библиотека для опроса датчика температуры
#include <LiquidCrystal.h>      //Библиотека для работы с ЖКдисплеем
#include <EEPROM.h>             //Библиотека для работы с памятью
#include <PID_v1.h>             //Библиотека для расчета воздействия

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Подключаем экран  RS, E, D4, D5, D6, D7
OneWire ds(2);   // Подключаем датчик температуры ко 2 цифровому входуы

double ystTemper ;                            // Требуемое показание датчика температуры .
double celsium, Output;
double aggKp=4, aggKi=0.2, aggKd=1;                // Определяем агрессивные и консервативные параметры PID
double consKp=1, consKi=0.05, consKd=0.25;
                                                // Определяем указатели и другие параметры PID
PID Regul(&celsium, &Output, &ystTemper, consKp, consKi, consKd, DIRECT);

byte outPercente;                                // Переменная для вывода мощности на вентиляторах;
double gap;
int kol;
int flagErr = 0;

#define BTN_UP   1
#define BTN_DOWN 2
#define BTN_LEFT 3
#define BTN_RIGHT 4                         //Работа с кнопками на shield
#define BTN_SELECT 5
#define BTN_NONE 10

//----------------------------------------------------------------------------------------------------------------
// массив char для форматированного вывода
char Arr1[16];
char Arr[16];
//----------------------------------------------------------------------------------------------------------------
  int flag = 0;
//  float E = 0, U=0, I=0, Eprev = 0;
//  float Ki = -0.1;
//  float Kp = 5;
//  float Kd = 0.2;

  int PID_per = 11;                       
  int PID_step =0;
//----------------------------------------------------------------------------------------------------------------
#define TIMER_CLOCK_FREQ 62500.0 //62.5kHz for /256 prescale from 16MHz


//Returns the time load value which must be loaded into TCNT2 inside your ISR routine.
//See the example usage below.
unsigned char timerLoadValue;
unsigned char SetupTimer2(float timeoutFrequency)
{
  unsigned char result; //The value to load into the timer to control the timeout interval.

  //Calculate the timer load value
  result= (int)((257.0-(TIMER_CLOCK_FREQ/timeoutFrequency))+0.5); //the 0.5 is for rounding;
  //The 257 really should be 256 but I get better results with 257, dont know why.

 TCCR1A = 0;
 TCCR1B = 1<<CS12 | 0<<CS11 | 0<<CS10;//256 prescaller        // TCCR2B = 1<<CS22 | 1<<CS21 | 0<<CS20;//256 prescaller так более короткие режимы



  //Включить прерывание по таймеру   
  TIMSK1 = 1<<TOIE1;

  //load the timer for its first cycle
  TCNT1=result; 
  
  return(result);
}

//----------------------------------------------------------------------------------------------------------------
int flash;
//-------------------------------------------------------------------------------------------------
//      Русские буквы для createChar
byte bukva_B[8]   = {B11110,B10000,B10000,B11110,B10001,B10001,B11110,B00000,}; // Буква "Б"
byte bukva_G[8]   = {B11111,B10001,B10000,B10000,B10000,B10000,B10000,B00000,}; // Буква "Г"
byte bukva_D[8]   = {B01111,B00101,B00101,B01001,B10001,B11111,B10001,B00000,}; // Буква "Д"
byte bukva_ZH[8]  = {B10101,B10101,B10101,B11111,B10101,B10101,B10101,B00000,}; // Буква "Ж"
byte bukva_Z[8]   = {B01110,B10001,B00001,B00010,B00001,B10001,B01110,B00000,}; // Буква "З"
byte bukva_I[8]   = {B10001,B10011,B10011,B10101,B11001,B11001,B10001,B00000,}; // Буква "И"
byte bukva_IY[8]  = {B01110,B00000,B10001,B10011,B10101,B11001,B10001,B00000,}; // Буква "Й"
byte bukva_L[8]   = {B00011,B00111,B00101,B00101,B01101,B01001,B11001,B00000,}; // Буква "Л"
byte bukva_P[8]   = {B11111,B10001,B10001,B10001,B10001,B10001,B10001,B00000,}; // Буква "П"
byte bukva_Y[8]   = {B10001,B10001,B10001,B01010,B00100,B01000,B10000,B00000,}; // Буква "У"
byte bukva_F[8]   = {B00100,B11111,B10101,B10101,B11111,B00100,B00100,B00000,}; // Буква "Ф"
byte bukva_TS[8]  = {B10010,B10010,B10010,B10010,B10010,B10010,B11111,B00001,}; // Буква "Ц"
byte bukva_CH[8]  = {B10001,B10001,B10001,B01111,B00001,B00001,B00001,B00000,}; // Буква "Ч"
byte bukva_Sh[8]  = {B10101,B10101,B10101,B10101,B10101,B10101,B11111,B00000,}; // Буква "Ш"
byte bukva_Shch[8]= {B10101,B10101,B10101,B10101,B10101,B10101,B11111,B00001,}; // Буква "Щ"
byte bukva_Mz[8]  = {B10000,B10000,B10000,B11110,B10001,B10001,B11110,B00000,}; // Буква "Ь"
byte bukva_IYI[8] = {B10001,B10001,B10001,B11001,B10101,B10101,B11001,B00000,}; // Буква "Ы"
byte bukva_Yu[8]  = {B10010,B10101,B10101,B11101,B10101,B10101,B10010,B00000,}; // Буква "Ю"
byte bukva_Ya[8]  = {B01111,B10001,B10001,B01111,B00101,B01001,B10001,B00000,}; // Буква "Я"
byte bukva_O[8]  = {B00000,B00000,B00000,B00000,B00000,B00000,B00000,B00000,}; // Нулевой символ

//------------------------------------------------------------------------------------------------- 
byte i;
byte data[12];
byte addr[8];
double celsius;

int detectButton() {
  delay(50);
  int keyAnalog =  analogRead(A0);
  delay(50);
  if (keyAnalog < 100) {
    // Значение меньше 100 – нажата кнопка right
    return BTN_RIGHT;
  } else if (keyAnalog < 200) {
    // Значение больше 100 (иначе мы бы вошли в предыдущий блок результата сравнения, но меньше 200 – нажата кнопка UP
    return BTN_UP;
  } else if (keyAnalog < 400) {
    // Значение больше 200, но меньше 400 – нажата кнопка DOWN
    return BTN_DOWN;
  } else if (keyAnalog < 600) {
    // Значение больше 400, но меньше 600 – нажата кнопка LEFT
    return BTN_LEFT;
  } else if (keyAnalog < 800) {
    // Значение больше 600, но меньше 800 – нажата кнопка SELECT
    return BTN_SELECT;
  } else {
    // Все остальные значения (до 1023) будут означать, что нажатий не было
    return BTN_NONE;
  }
}


void clearLine(int line){
  lcd.setCursor(0, line);
  lcd.print("                ");
}
void printDisplay(String message,String message1){
 // Serial.println(message);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);
  lcd.setCursor(0, 1);
  lcd.print(message1);
}


//--------------------------------------------------------------------------------------------------------------------
//Обработка прерываний по таймеру timer1
ISR(TIMER1_OVF_vect) {

  noInterrupts(); // Отключаем обработку прерываний
  
if ((flag ==0)&&(flagErr==0)){
  flag = 1;
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // Запуск измерения температуры
  }
  else if (flagErr==0){
  flag = 0;               
  ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Читаем память датчика
  for ( i = 0; i < 9; i++) {
        data[i] = ds.read();
        }
  int16_t raw = (data[1] << 8) | data[0];     // датчик может быть настроен на разную точность, выясняем её 
  byte cfg = (data[4] & 0x60);
  if (cfg == 0x00) raw = raw & ~7;            // точность 9-разрядов, 93,75 мс 
    else if (cfg == 0x20) raw = raw & ~3;     // точность 10-разрядов, 187,5 мс 
    else if (cfg == 0x40)raw = raw & ~1;      // точность 11-разрядов, 375 мсelse if (cfg == 0x40)
    // преобразование показаний в градусы Цельсия 
    celsius = (float)raw*0.5;// * 0.0625 ; /// 16.0;        !!!!!!Временно под мой датчик изменен коэффициент
  PID_step = 1;
  }
 
  
  TCNT1=timerLoadValue;
  timerLoadValue=SetupTimer2(1);

         Serial.print("celsius = ");
         Serial.println(celsius);

  interrupts();   //Включаем обработку прерываний обратно
}

void setup() {

  Regul.SetMode(AUTOMATIC);  //Запускаем ПИД
  
  

  EEPROM.get(1,flash);                 //Читаем записанную ттемпературу
  if (flash > 100) flash = 25;         // Если записано что то то непонятное то устанавливаем значение равное 0
  ystTemper = flash;
  lcd.begin(16, 2);                    //Инициализируем экран
  Serial.begin(9600);                  // Инициализируем RS
  //----------------------Прописываем буквы использующиеся в приветствии------------------------
  lcd.createChar(1,bukva_P );
  lcd.createChar(2,bukva_L );
  lcd.createChar(3,bukva_I);
  lcd.createChar(4,bukva_D );
  lcd.createChar(5,bukva_CH );
  lcd.createChar(6,bukva_IY );
  lcd.createChar(7,bukva_IYI );
  lcd.setCursor(2, 0);
  lcd.print("Y\1PAB\2EH\3E");
  lcd.setCursor(0, 1);
  lcd.print("\1O\4A\5E\6 BO\4\7");
  delay(3000);
  lcd.clear();
 //----------------------Выводим что считали из памяти и запускаем таймер--------------------- 
  Serial.print("flash = ");
  Serial.println(flash);
  
  timerLoadValue=SetupTimer2(1); 
}

void loop() {
 
  if (!ds.search(addr)) {
       ds.reset_search();
        delay(250);
//        kol+=1;
//        if ((!ds.search(addr)&&(kol>10))){
//        flagErr= 1;         //Датчик не исправен
//        }
        return;
   }
  lcd.createChar(1,bukva_I );
  lcd.createChar(2,bukva_Z );
  celsium = celsius;
  int button = detectButton();
  
  switch (button) {
    case BTN_UP:
      ystTemper+=1;
      sprintf (Arr1, "T.\1\2M =%3d*C",(int)celsium );
      sprintf (Arr,  "T.YCT =%3d*C",(int)ystTemper );
      printDisplay(Arr1,Arr);
      break;
    case BTN_DOWN:
      ystTemper-=1;
      sprintf (Arr1, "T.\1\2M =%3d*C",(int)celsium );
      sprintf (Arr,  "T.YCT =%3d*C",(int)ystTemper );
      printDisplay(Arr1,Arr);
      break;
    case BTN_LEFT:
     // printDisplay("LEFT","LEFT");
      break;
    case BTN_RIGHT:
    //  printDisplay("RIGHT","RIGHT");
      break;
    case BTN_SELECT:
     // printDisplay("SELECT","SELECT");
      break;
    default:
      //printDisplay("Press any key");
      sprintf (Arr1, "T.\1\2M =%3d*C",(int)celsium );
      sprintf (Arr,  "T.YCT =%3d*C",(int)ystTemper );
      printDisplay(Arr1,Arr);
      break;
  }

 if (flash != ystTemper){
     flash = ystTemper;
     TIMSK1 = 0<<TOIE1;//Выключить прерывание по таймеру 
     EEPROM.put(1,flash);
     TIMSK1 = 1<<TOIE1;//Включить прерывание по таймеру 
     Serial.print("Put_flash = ");
     Serial.println(flash);
  } 
   if (flagErr ==1){
  lcd.createChar(1,bukva_I );
  lcd.createChar(2,bukva_P );
  lcd.createChar(3,bukva_Mz);
  lcd.createChar(4,bukva_D );
  lcd.createChar(5,bukva_CH );
  lcd.setCursor(0, 0);
  lcd.print("HE\1C\2PABHOCT\3  ");
  lcd.setCursor(0, 1);
  lcd.print("\4AT\5\1KA!!!      ");
  analogWrite(11,0);      //Выключаем ШИМ
  
   }
/*
//*** вычисление воздействия *****
//***----------------------*******
if (PID_step == 1)
{
E = celsium - ystTemper;
U = I + Ki * E;               // интегральный коэффициент
I = U;

U += Kp * E;                  // пропорциональный коэффициент

U += Kd * (E - Eprev);        // дифференциальный коэффициент
Eprev = E;

PID_step = 0;     
     Serial.print("E = ");
     Serial.println(E);
     Serial.print("U = ");
     Serial.println(U);
}

//***----------------------*******

*/

if ((PID_step == 1)&&(flagErr == 0)){

  gap = abs(ystTemper-celsium);         // Разность с желаемой температурой 
  if(gap<5) {                            // Мы недалеко и используем консервативные параметры;
     Regul.SetTunings(consKp, consKi, consKd);
  }
  else {                                // Мы далеко, используем агрессивные параметры;
     Regul.SetTunings(aggKp, aggKi, aggKd);
  }

   Regul.Compute();
   analogWrite(11,Output);      //Включаем ШИМ
   //Serial.print(Output);
   //Serial.println(" = Output");
   outPercente = (byte)(Output/255*100);// Считаем в процентах мощность вывода ШИМ;
   lcd.setCursor(12,0);                    // Печатаем на ЖК;
   lcd.print("O=    ");
   lcd.setCursor(14,0);
   lcd.print(outPercente);
   //lcd.print("%");
  PID_step == 0;
}

}
