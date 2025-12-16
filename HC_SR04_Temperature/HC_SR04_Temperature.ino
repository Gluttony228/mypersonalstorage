
// Для работы с дальномером HC-SR04
#include <NewPing.h>
// Для работы с сервоприводом
#include <Servo.h>
// libraries

#include "ST7735_LTSM.hpp"
// Fonts needed
#include "fonts_LTSM/FontSinclairS_LTSM.hpp"

// Макросы пинов на Arduino
// Дальномер
#define HCSR04_TRIG 3
#define HCSR04_ECHO 4
// Дисплей
#define LCD_CS 10
#define LCD_DC 9
#define LCD_RST 8

#define SCREEN_W 160
#define SCREEN_H 128

#define SCREEN_SERVICE_AREA_W SCREEN_W
#define SCREEN_SERVICE_AREA_H 20

#define SCREEN_RADAR_AREA_W SCREEN_W
#define SCREEN_RADAR_AREA_H (SCREEN_H-SCREEN_SERVICE_AREA_H)

// Сервопривод
#define SERVO_CTRL 5


// ================================================
// Глобальные переменные

// create an instance of the library
ST7735_LTSM myTFT;

// Дальномер
NewPing Sonar(HCSR04_TRIG, HCSR04_ECHO, 400);

Servo servo1;

void setup() 
{
  Serial.begin(9600);
  servo1.attach(SERVO_CTRL);

  //*************** USER OPTION 1 SPI_SPEED + TYPE ***********
  uint32_t TFT_SCLK_FREQ = 8000000;  // Spi freq in Hertz
  myTFT.setupGPIO_SPI(TFT_SCLK_FREQ, LCD_RST, LCD_DC, LCD_CS);
  //********************************************************
  // ****** USER OPTION 2 Screen Setup ******
  uint8_t OFFSET_COL = 0;     // 2, These offsets can be adjusted for any issues->
  uint8_t OFFSET_ROW = 0;     // 3, with screen manufacture tolerance/defects
  uint16_t TFT_WIDTH = 128;   // Screen width in pixels
  uint16_t TFT_HEIGHT = 160;  // Screen height in pixels
  myTFT.TFTInitScreenSize(OFFSET_COL, OFFSET_ROW, TFT_WIDTH, TFT_HEIGHT);
  // ******************************************
  // ******** USER OPTION 3 PCB_TYPE  **************************
  myTFT.TFTInitPCBType(myTFT.TFT_ST7735R_Red);  // pass enum,4 choices,see README
  //**********************************************************

  // Начальная картинка
  myTFT.setRotation(3);
	myTFT.fillScreen(myTFT.C_BLACK);
  myTFT.drawRectWH(0, 0, SCREEN_W, SCREEN_H-SCREEN_SERVICE_AREA_H, myTFT.C_GREEN);
  myTFT.drawRectWH(0, 0, SCREEN_W, SCREEN_H,                     myTFT.C_GREEN);

  myTFT.drawCircle(SCREEN_RADAR_AREA_W/2, SCREEN_RADAR_AREA_H, SCREEN_RADAR_AREA_H-80, myTFT.C_GREEN);
  myTFT.drawCircle(SCREEN_RADAR_AREA_W/2, SCREEN_RADAR_AREA_H, SCREEN_RADAR_AREA_H-50, myTFT.C_GREEN);
  myTFT.drawCircle(SCREEN_RADAR_AREA_W/2, SCREEN_RADAR_AREA_H, SCREEN_RADAR_AREA_H-20, myTFT.C_GREEN);

  myTFT.fillRect(0, SCREEN_H-SCREEN_SERVICE_AREA_H, SCREEN_W, SCREEN_SERVICE_AREA_H, myTFT.C_BLACK);


	myTFT.setTextColor(myTFT.C_GREEN, myTFT.C_BLACK);
  myTFT.setFont(FontSinclairS);

	myTFT.setCursor(0, SCREEN_H-(SCREEN_SERVICE_AREA_H/2)-4);
  myTFT.print("Distance:\n");
  
}

// ===============================================
// Медианный фильтр измерения дальности из 3 значений
float MiddleOf3(float a, float b, float c) 
{
  float middle = 0;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  }
  else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}

// Функция получения дальности
float GetDistance()
{
  static unsigned int NumPos = 60;
  static unsigned int iPos = 0;

  static float DistanceFiltered = 0;
  // массив для хранения трёх последних измерений
  static float         Last3Measures[3] = {0.0, 0.0, 0.0};   
  static byte          IndexMeasure = 0;
  static unsigned long LastTimeMs = 0;

  // Локальные переменные

  float CurrentDistance = 0;
  byte  DeltaMeasures   = 0;
  float Coeff           = 0;

  if (millis() - LastTimeMs > 200) 
  { // измерение и вывод каждые 50 мс
    // счётчик от 0 до 2
    // каждую итерацию таймера i последовательно принимает значения 0, 1, 2, и так по кругу
    if (IndexMeasure > 1) IndexMeasure = 0;
    else IndexMeasure++;

    // получить расстояние в текущую ячейку массива
    Last3Measures[IndexMeasure] = (float)Sonar.ping() / 57.5; 
    Serial.print(Last3Measures[IndexMeasure]);                
    // фильтровать медианным фильтром из 3ёх последних измерений
    CurrentDistance = MiddleOf3(Last3Measures[0], Last3Measures[1], Last3Measures[2]);    

    // расчёт изменения с предыдущим
    DeltaMeasures = abs(DistanceFiltered - CurrentDistance);     
                  
    if (DeltaMeasures > 1) 
    {
      // если большое - резкий коэффициент
      Coeff = 0.7;
    }                                 
    else 
    {
      // если маленькое - плавный коэффициент
      Coeff = 0.1;
    }   
   //Coeff = 0.2     ;                                   

    DistanceFiltered = CurrentDistance * Coeff + DistanceFiltered * (1 - Coeff);     // фильтр "бегущее среднее"

    LastTimeMs = millis();                                  // сбросить таймер

    int Step = radians(60)/NumPos; 
   

    myTFT.setCursor(SCREEN_W/2, SCREEN_H-(SCREEN_SERVICE_AREA_H/2)-4);
    myTFT.print(DistanceFiltered);

    myTFT.drawLine(SCREEN_W/2, SCREEN_H-(SCREEN_SERVICE_AREA_H), 
    cos(iPos*Step)*10, 
    sin(iPos*Step)*10, 
    myTFT.C_WHITE);

    iPos++;
    if (iPos > 60)
      iPos = 0;
    
    Serial.print(',');
    Serial.println(DistanceFiltered);
    return DistanceFiltered;
  }
}

void loop()
{

  GetDistance();

  //myTFT.setCursor(5,30);
  //myTFT.print(str_out);
  //delay(50);
}