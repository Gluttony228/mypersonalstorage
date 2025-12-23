
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
#define LCD_CS  10
#define LCD_DC  9
#define LCD_RST 8
// Сервопривод
#define SERVO_CTRL 5

// Дисплей, константы

#define SCREEN_W 160
#define SCREEN_H 128

#define SCREEN_SERVICE_AREA_W SCREEN_W
#define SCREEN_SERVICE_AREA_H 20

#define SCREEN_RADAR_AREA_W SCREEN_W
#define SCREEN_RADAR_AREA_H (SCREEN_H-SCREEN_SERVICE_AREA_H)

#define CENTER_X SCREEN_W / 2
#define CENTER_Y SCREEN_H / 2

#define MEASURE_PERIOD_MS 150


#define RADAR_SCAN_RADIUS_PIX (SCREEN_RADAR_AREA_H - 12)
#define DMAX_CM               (96)
#define D_IN_PIX              (DMAX_CM/RADAR_SCAN_RADIUS_PIX)

#define SECTOR_SCAN_RB_DEG 30
#define SECTOR_SCAN_LB_DEG 150

// ================================================
// Глобальные переменные

// create an instance of the library
ST7735_LTSM myTFT;

// Дальномер
NewPing Sonar(HCSR04_TRIG, HCSR04_ECHO, 400);

Servo servo1;

void StartScreen()
{
  myTFT.fillScreen(myTFT.C_BLACK);
  myTFT.drawRectWH(0, 0, SCREEN_W, SCREEN_H-SCREEN_SERVICE_AREA_H, myTFT.C_GREEN);
  myTFT.drawRectWH(0, 0, SCREEN_W, SCREEN_H,                       myTFT.C_GREEN);

  myTFT.drawCircle(SCREEN_RADAR_AREA_W/2, SCREEN_RADAR_AREA_H, SCREEN_RADAR_AREA_H-8,  myTFT.C_GREEN);
  myTFT.drawCircle(SCREEN_RADAR_AREA_W/2, SCREEN_RADAR_AREA_H, SCREEN_RADAR_AREA_H-38, myTFT.C_GREEN);
  myTFT.drawCircle(SCREEN_RADAR_AREA_W/2, SCREEN_RADAR_AREA_H, SCREEN_RADAR_AREA_H-68, myTFT.C_GREEN);

  myTFT.fillRect(0, SCREEN_H-SCREEN_SERVICE_AREA_H, SCREEN_W, SCREEN_SERVICE_AREA_H, myTFT.C_BLACK);


	myTFT.setTextColor(myTFT.C_GREEN, myTFT.C_BLACK);
  myTFT.setFont(FontSinclairS);

	myTFT.setCursor(0, SCREEN_H-(SCREEN_SERVICE_AREA_H/2)-4);
  myTFT.print(" D(cm):");
  myTFT.setCursor((SCREEN_W/2)+5, SCREEN_H-(SCREEN_SERVICE_AREA_H/2)-4);
  myTFT.print("A(dg):");
}

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
  StartScreen();
  
}

#define TO_RAD (M_PI/180.0) 
#define BEAM_HALF_WIDTH_DEG 1

// ===============================================
// Функция для рисования линии под заданным углом (в градусах)
void DrawLineAtAngle(int angle, int distance, uint16_t color) 
{
  static int VecLB[2]   = {0};
  static int VecDir[2]  = {0};

  // Вычисляем конечную точку линии по радиусу
  int XDir = CENTER_X                +  ((RADAR_SCAN_RADIUS_PIX+4)* cos(radians(angle)));
  int YDir = (SCREEN_RADAR_AREA_H-2) -  ((RADAR_SCAN_RADIUS_PIX+4)* sin(radians(angle)));

  int XLB   = CENTER_X                +  ((RADAR_SCAN_RADIUS_PIX+8) * cos(radians(angle)));
  int YLB   = (SCREEN_RADAR_AREA_H-2) -  ((RADAR_SCAN_RADIUS_PIX+8) * sin(radians(angle)));

  // Рисуем линию от центра к конечной точке
  myTFT.drawLine(XDir, YDir, XLB, YLB, myTFT.C_GREEN);
    // очистка пред. позиции луча
  if (VecLB[1])
  {
    myTFT.drawLine(VecDir[0], VecDir[1], VecLB[0], VecLB[1], myTFT.C_BLACK);
  }
  VecLB[0] = XLB;
  VecLB[1] = YLB;
  VecDir[0] = XDir;
  VecDir[1] = YDir;


  // прибитие дальности
  if (distance > DMAX_CM)
    distance = DMAX_CM;
  int distancetopix = (distance/D_IN_PIX);

  // отрисовка луча
  for (int i = BEAM_HALF_WIDTH_DEG-1; i > -BEAM_HALF_WIDTH_DEG; i--)
  {
    int XDist = CENTER_X                +  (distancetopix) * cos(radians(angle)+(i*TO_RAD));
    int YDist = (SCREEN_RADAR_AREA_H-2) -  (distancetopix) * sin(radians(angle)+(i*TO_RAD));
    int XNo   = CENTER_X                +  (RADAR_SCAN_RADIUS_PIX) * cos(radians(angle)+(i*TO_RAD));
    int YNo   = (SCREEN_RADAR_AREA_H-2) -  (RADAR_SCAN_RADIUS_PIX) * sin(radians(angle)+(i*TO_RAD));
    // Рисуем линию от центра к конечной точке
    myTFT.drawLine(SCREEN_RADAR_AREA_W/2, SCREEN_RADAR_AREA_H-2, XDist, YDist, myTFT.C_GREEN);
    myTFT.drawLine(XDist, YDist, XNo, YNo, myTFT.C_BLUE);
  }
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
  static int   Angle    =  30;
  static int   AngleDir =  0;

  static float DistanceFiltered = 0;
  // массив для хранения трёх последних измерений
  static float         Last3Measures[3] = {0.0, 0.0, 0.0};   
  static byte          IndexMeasure = 0;
  static unsigned long LastTimeMs = 0;

  // Локальные переменные

  float CurrentDistance = 0;
  byte  DeltaMeasures   = 0;
  float Coeff           = 0;

  if (millis() - LastTimeMs > MEASURE_PERIOD_MS) 
  { 
    servo1.write(1);
    // измерение и вывод каждые 100 мс
    // счётчик от 0 до 2
    // каждую итерацию таймера i последовательно принимает значения 0, 1, 2, и так по кругу

    if (IndexMeasure > 1) IndexMeasure = 0;
    else IndexMeasure++;

    // получить расстояние в текущую ячейку массива
    Last3Measures[IndexMeasure] = (float)Sonar.ping() / 57.5; 
    //Serial.print(Last3Measures[IndexMeasure]);                
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

    DistanceFiltered = CurrentDistance * Coeff + DistanceFiltered * (1 - Coeff);     // фильтр "бегущее среднее"

    LastTimeMs = millis();                                  // сбросить таймер

    servo1.write(IndexMeasure*10); 

    myTFT.setCursor((SCREEN_W/2)-20, SCREEN_H-(SCREEN_SERVICE_AREA_H/2)-4);
    myTFT.print("   ");
    myTFT.setCursor((SCREEN_W/2)-20, SCREEN_H-(SCREEN_SERVICE_AREA_H/2)-4);
    myTFT.print((int)DistanceFiltered);
    myTFT.setCursor((SCREEN_W/2)+55, SCREEN_H-(SCREEN_SERVICE_AREA_H/2)-4);
    myTFT.print("   ");
    myTFT.setCursor((SCREEN_W/2)+55, SCREEN_H-(SCREEN_SERVICE_AREA_H/2)-4);
    myTFT.print(Angle-90);
    //Serial.print(',');
    //Serial.println(DistanceFiltered);

    DrawLineAtAngle(Angle, (int)DistanceFiltered, myTFT.C_GREEN);

    //Serial.print(',');
   // Serial.println(Angle);
   
    if (!AngleDir)
      Angle += 1;
    else
      Angle -= 1;

    if (Angle == 150 || Angle == 30)
    {
      AngleDir = !AngleDir;
    }

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