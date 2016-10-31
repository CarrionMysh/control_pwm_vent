//управление 3 вентиляторами (3-pin) ШИМом. вход: 3 датчика ds18b20, энкодер как орган управления. вывод: экран LCD 16x2.
// питание от molex-разъема комп.БП +12в.
//энкодер работает в режиме с прерываниями (оба)
//значения оборотов снимаются через внешнюю atmega8a (работает)
//atmega8a связь через SoftSerial. буфер не меняем.
//формат передачи данных: три пакета с оборотами вентов: "500,700,120.". таймаут по умолчанию 1 сек.
//полчученные данные с атмеги есть данные х2, так как за оборот вента, тахометр считает дважды. полученные данные умножаем на 30сек вместо 60сек
//

#include <Encoder.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>


//используемые пины (arduino uno)
#define ENCODER_USE_INTERRUPTS
#define pin_pwm_v1 9        //шим вент1
#define pin_pwm_v2 10        //шим вент2
#define pin_pwm_v3 11        //шим вент3
#define pin_pwm_v4 6        //шим вент3
#define clk        3        //clk энкодера
#define dt         2        //dt энкодера
#define sw         8        //sw энкодера
#define pin_tx_enable   5    //пин для флага начала передачи данных по atmega.serial
#define pin_temp        12  //пин температурных датчиков
#define pin_atmegaSerial_rx 4  //пин для софтового порта rx
#define pin_atmegaSerial_tx 7  //пин для софтового порта tx
//остальные константы
#define wait 0             //задержка для срабатывания прерывания на atmega8a
#define tau2  500          //переменная времени для периода. равняется tau на атмеге8а минус примерное время основных операций (считывание температуры, передача rpm). разность с tau_атмеги есть "глухота" кнопки энкодера.
#define tau_atmega8a    1000
#define temp_resolut    10   //точность измерения температуры 9..12
#define addr_pwm_min1   1
#define addr_pwm_min2   2
#define addr_pwm_min3   3
#define addr_pwm_stop1  4
#define addr_pwm_stop2  5
#define addr_pwm_stop3  6
#define addr_pwm_start1 7
#define addr_pwm_start2 8
#define addr_pwm_start3 9
#define addr_vent1      10
#define addr_vent2      11
#define addr_vent3      12
#define addr_pwm1       13
#define addr_pwm2       14
#define addr_pwm3       15
#define addr_pwm1_up    16
#define addr_pwm2_up    17
#define addr_pwm3_up    18


Encoder enc(clk, dt);
OneWire wire(pin_temp);
SoftwareSerial atmega(pin_atmegaSerial_rx, pin_atmegaSerial_tx);
LiquidCrystal_I2C lcd(0x20, 4, 5, 6, 0, 1, 2, 3, 7, NEGATIVE);

DeviceAddress probe1 = {0x28, 0xFF, 0x68, 0x57, 0x77, 0x4, 0x0, 0x97};
DeviceAddress probe2 = {0x28, 0xFF, 0x44, 0x57, 0x73, 0x4, 0x0, 0xD0};
DeviceAddress probe3 = {0x28, 0xFF, 0x91, 0x4E, 0x77, 0x4, 0x0, 0x69};
DallasTemperature probes(&wire);

volatile int val;                                                 //переменная для счетчика оборотов в обработчике прерывания
byte step_pwm = 1;                                                //шаг ШИМа
byte step_alarm = 10;                                             //базовый "шаг" ШИМа при превышении порога температуры
byte pwm1 = 0, pwm2 = 0, pwm3 = 0;                                //значение ШИМ. внимание, после 255 будет 0 и обратно.
unsigned int pwm1_a, pwm2_a, pwm3_a;                                      //значение ШИМ во время превышения порога температуры
byte pwm_min1, pwm_min2, pwm_min3;                                //минимальные "честные" значения pwm после калибровки
byte pwm_start1, pwm_start2, pwm_start3;                          //стартовые значения pwm, при которых венты стартуют (могут быть меньше чем pwm при которых венты уже крутятся)
byte pwm_stop1, pwm_stop2, pwm_stop3;                             //значения pwm, когда венты стопорятся
float t_probe1, t_probe2, t_probe3;                               //температура
byte var_n;                                                       //полученное количество опрошенных датчиков
long enc_pos = 0;                                                 //начальное положение энкодера
long enc_new_pos = 0;                                             //новое значение энкодера
byte sw_count = 1;                                                //счетчик нажатий на кнопку энкодера, он же указатель выбора номера вентов, изначально первый канал
boolean sw_flag = false;                                          //флаг sw энкодера. "было нажатие"
float t_alarm1 = 30;                                              //пороговое значение температуры радиатора проца
float t_alarm2 = 50;                                              //пороговое значение температуры радиатора на материнской плате
float t_alarm3 = 50;                                              //пороговое значение температуры радиатора БП
boolean alarm1 = 0, alarm2 = 0, alarm3 = 0;                       //алярмы превышения порога температуры, при включении сброшены в ноль
byte ch   ;                                                       //переменная для SoftSerial
byte count = 0;                                                   //счетчик пакетов
int rpm1, rpm2, rpm3;                                             //обороты вентов
unsigned long time1;                                              //переменные времени формирования периода измерений rpm На atmega8a ~1000мс
unsigned long time3;                                              //Для дебага
unsigned long view_time = 3000;                                   //врямя показа переменных данных
int rpm_max1 = 0, rpm_max2 = 0, rpm_max3 = 0, rpm_min1 = 0, rpm_min2 = 0, rpm_min3 = 0; //максимальные/минимальные значения для вентов
boolean vent1, vent2, vent3;                                      //флаги наличия вентов
float rpm_s1[10], rpm_tt1, rpm_s2[10], rpm_tt2, rpm_s3[10], rpm_tt3; //массив значений реальных оборотов, переменная средней скользящей
int n = 0;                                                        //переменная для перебора массивов rpm
byte nn = 1;                                                      // 0..х коэфициент сглаживания скользящей средней (до10 - ограничено массивом)
boolean zero_data1 = true, zero_data2 = true, zero_data3 = true;  //флаг первых данных или после остановки вентов
boolean flag_loop_tau = true;                                     //флаг цикла по времени.
float t_post_alarm = 2;                                           //сколько градусов нужно держать обороты посталярмы
float t_pred1, t_pred2, t_pred3, dtt1, dtt2, dtt3;                //предыдущее значение дельта температуры
unsigned long time_sw;                                            //переменная времени после нажатия
int old_pwm1, old_pwm2, old_pwm3;
boolean flag_pwm_view = false;
unsigned long time_tau;
float temp_mem[1][3];
float rpm_mem [1][3];
int pwm1_up, pwm2_up, pwm3_up;


void setup() {
  pinMode(sw, INPUT);
  pinMode(pin_tx_enable, OUTPUT);
  digitalWrite(pin_tx_enable, LOW);     //опускаем флаг передачи данных по atmega.serial
  Serial.begin(9600);                 //для отладки
  atmega.begin(9600);                 //открываем порт в сторону atmega8a
  probes.begin();                       //инициализация датчиков
  lcd.begin(16, 2);
  lcd.rightToLeft();
  lcd.noCursor();
  analogWrite(pin_pwm_v1, 255);         //при включении все венты в максимум, своего рода тест работоспособности
  analogWrite(pin_pwm_v2, 255);
  analogWrite(pin_pwm_v3, 255);

  var_n = probes.getDeviceCount();      //проверяем количество датчиков на шине
  if (var_n != 3) {
    lcd.home();
    lcd.clear();
    lcd.print("Warning: "); lcd.setCursor(0, 1);
    lcd.print(3 - var_n);
    lcd.print(" probes lost");
    delay(3000);                              //переписать. дрянь какая-то
    lcd.home();
    lcd.clear();
    if (!probes.getAddress(probe1, 0)) {
      lcd.print("lost connecting probe1");
      delay(2000);
      lcd.home();
      lcd.clear();
    }
    if (!probes.getAddress(probe2, 1)) {
      lcd.print("lost connecting probe2");
      delay(2000);
      lcd.home();
      lcd.clear();
    }
    if (!probes.getAddress(probe3, 2)) {
      lcd.print("lost connecting probe3");
      delay(2000);
      lcd.home();
      lcd.clear();
    }
  }

  probes.setResolution(probe1, temp_resolut);   //устанавливаем точность измерения температуры
  probes.setResolution(probe2, temp_resolut);
  probes.setResolution(probe3, temp_resolut);

  //проверяем связь с atmega8a
  while (atmega.available()) {                  //почистим что прилетело в буфер, на всякий.
    ch = atmega.read();
  }

  pwm_min1 = EEPROM.read(addr_pwm_min1);
  pwm_min2 = EEPROM.read(addr_pwm_min2);
  pwm_min3 = EEPROM.read(addr_pwm_min3);
  pwm_stop1 = EEPROM.read(addr_pwm_stop1);
  pwm_stop2 = EEPROM.read(addr_pwm_stop2);
  pwm_stop3 = EEPROM.read(addr_pwm_stop3);
  pwm_start1 = EEPROM.read(addr_pwm_start1);
  pwm_start2 = EEPROM.read(addr_pwm_start2);
  pwm_start3 = EEPROM.read(addr_pwm_start3);
  vent1 = EEPROM.read(addr_vent1);
  vent2 = EEPROM.read(addr_vent2);
  vent3 = EEPROM.read(addr_vent3);
  pwm1 = EEPROM.read(addr_pwm1);
  pwm2 = EEPROM.read(addr_pwm2);
  pwm3 = EEPROM.read(addr_pwm3);
  pwm1_up = EEPROM.read(addr_pwm1_up);
  pwm2_up = EEPROM.read(addr_pwm2_up);
  pwm3_up = EEPROM.read(addr_pwm3_up);

  if (!vent1 && !vent2 && !vent3) calib_vent();   //если в EEPROM'е все венты отсутствуют - запустить калибровку (первое включение девайса)
  analogWrite(pin_pwm_v1, pwm_start1);
  analogWrite(pin_pwm_v2, pwm_start2);
  analogWrite(pin_pwm_v3, pwm_start3);
  lcd.home();
  lcd.clear();
  lcd.print("Launching VENTs");
  delay(view_time);
  analogWrite(pin_pwm_v1, pwm1);
  analogWrite(pin_pwm_v2, pwm2);
  analogWrite(pin_pwm_v3, pwm3);
}

void temp_req() {
  t_probe1 = probes.getTempC(probe1);   //получаем с датчиков температуру
  t_probe2 = probes.getTempC(probe2);
  t_probe3 = probes.getTempC(probe3);
}

void read_rpm() {
  rpm1 = 0;
  rpm2 = 0;
  rpm3 = 0;
  time3 = millis();
  digitalWrite(pin_tx_enable, HIGH);  //посылаем сигнал передачи данных
  delay(wait);
  count = 1;                          //счетчик пакетов в ноль
  while (true) {
    if (atmega.available()) {           //читаем данные с atmega.serial
      ch = atmega.read();
      //     Serial.print("ch=");Serial.println(ch);
      if (char(ch) == '.') break;
      if (char(ch) == ',') count++; else
        switch (count) {
          case 1:
            rpm1 = rpm1 * 10 + (ch - '0');
            break;
          case 2:
            rpm2 = rpm2 * 10 + (ch - '0');
            break;
          case 3:
            rpm3 = rpm3 * 10 + (ch - '0');
            break;
        }
    }
    // Serial.println(millis()-time3);
    if (millis() - time3 > 1000) break; //если атмега не ответила за секунду - не стопоримся в вечном ожидании
  }
  digitalWrite(pin_tx_enable, LOW);   //опустили флаг
}

void lcd_rpm(float rpm_tt1, float rpm_tt2, float rpm_tt3) {
  rpm_mem[0][1] = rpm_tt1;
  rpm_mem[0][2] = rpm_tt2;
  rpm_mem[0][3] = rpm_tt3;
  if (rpm_mem[0][1] != rpm_mem[1][1]) {
    lcd.setCursor(0, 0);
    lcd.print("     ");
    lcd.setCursor(0, 0);
    if (rpm1 == 0) lcd.print("stop"); else if (pwm1 > pwm_stop1 && pwm1 < pwm_min1)lcd.print("slow"); else lcd.print(rpm_tt1 * 30, 0);
  }
  if (rpm_mem[0][2] != rpm_mem[1][2]) {
    lcd.setCursor(5, 0);
    lcd.print("     ");
    lcd.setCursor(5, 0);
    if (rpm2 == 0) lcd.print("stop"); else if (pwm2 > pwm_stop2 && pwm2 < pwm_min2)lcd.print("slow"); else lcd.print(rpm_tt2 * 30, 0);
  }
  if (rpm_mem[0][2] != rpm_mem[1][3]) {
    lcd.setCursor(10, 0);
    lcd.print("     ");
    lcd.setCursor(10, 0);
    if (rpm3 == 0) lcd.print("stop"); else if (pwm3 > pwm_stop3 && pwm3 < pwm_min3)lcd.print("slow"); else lcd.print(rpm_tt3 * 30, 0);
  }
  rpm_mem[1][1] = rpm_tt1;
  rpm_mem[1][2] = rpm_tt2;
  rpm_mem[1][3] = rpm_tt3;
}

void lcd_cal() {
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  if (rpm1 == 0) lcd.print("stop"); else lcd.print(rpm1 * 30);
  lcd.setCursor(5, 0);
  if (rpm2 == 0) lcd.print("stop"); else lcd.print(rpm2 * 30);
  lcd.setCursor(10, 0);
  if (rpm3 == 0) lcd.print("stop"); else lcd.print(rpm3 * 30);
}

void lcd_temp() {
  temp_mem[0][1] = t_probe1;
  temp_mem[0][2] = t_probe2;
  temp_mem[0][3] = t_probe3;
  if (temp_mem[0][1] != temp_mem[1][1]) {
    lcd.setCursor(0, 1);
    lcd.print("     ");
    lcd.setCursor(0, 1);
    lcd.print(t_probe1, 1);
  }
  if (temp_mem[0][2] != temp_mem[1][1]) {
    lcd.setCursor(5, 1);
    lcd.print("     ");
    lcd.setCursor(5, 1);
    lcd.print(t_probe2, 1);
  }
  if (temp_mem[0][3] != temp_mem[1][1]) {
    lcd.setCursor(10, 1);
    lcd.print("     ");
    lcd.setCursor(10, 1);
    lcd.print(t_probe3, 1);
  }
  temp_mem[1][1] = t_probe1;
  temp_mem[1][2] = t_probe2;
  temp_mem[1][3] = t_probe3;

}

void lcd_pwm() {
  if (flag_pwm_view) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    switch (sw_count) {
      case 1:
        lcd.setCursor(0, 1);
        lcd.print(pwm1, 1);
        break;
      case 2:
        lcd.setCursor(5, 1);
        lcd.print(pwm2, 1);
        break;
      case 3:
        lcd.setCursor(10, 1);
        lcd.print(pwm3, 1);
        break;
    }
  }
  flag_pwm_view = false;
}

void calib_vent() {                                                   //калибровка вентиляторов
  boolean fault_rotation1 = false, fault_rotation2 = false, fault_rotation3 = false;
  boolean fake_rpm1, fake_rpm2, fake_rpm3;                            //флаги ложных значений оборотов
  boolean flag_rpm1_up = 0, flag_rpm2_up = 0, flag_rpm3_up = 0, flag_rpm11_up = 0, flag_rpm22_up = 0, flag_rpm33_up = 0;                        //флаги первого снижения оборотов,
  //  byte rf=0;                                                      //счетчик случаев снижения скорости
  vent1 = false;
  vent2 = false;
  vent3 = false;
  fake_rpm1 = false;
  fake_rpm2 = false;
  fake_rpm3 = false;



  int cc_step = 2;                                                    //шаг pwm в цикле калибровки. для дебага =2(быстрее). в релизе лучше 1
  lcd.home();
  lcd.clear();
  lcd.print("Testing..");
  analogWrite(pin_pwm_v1, 255);                                       //меряем максимум
  analogWrite(pin_pwm_v2, 255);
  analogWrite(pin_pwm_v3, 255);
  delay(view_time * 2);                                                 //даем время на раскрутку вентов
  read_rpm();
  rpm_max1 = rpm1;
  rpm_max2 = rpm2;
  rpm_max3 = rpm3;
  rpm_min1 = 32767;                                                  //начинаем мерить минимум. минимальным значениям задаем заведомо высокое значение (32767=предел для int)
  rpm_min2 = 32767;
  rpm_min3 = 32767;
  pwm1_up = 255;
  pwm2_up = 255;
  pwm3_up = 255;
  for (byte i = 255; i >= 0; i -= cc_step) {                                  //в цикле снижаем pwm.
    pwm1 = i;
    pwm2 = i;
    pwm3 = i;
    analogWrite(pin_pwm_v1, pwm1);
    analogWrite(pin_pwm_v2, pwm2);
    analogWrite(pin_pwm_v3, pwm3);
    lcd.setCursor(0, 1); lcd.print("                "); lcd.setCursor(0, 1); lcd.print("pwm: "); lcd.print(i);
    delay(tau2);
    read_rpm();                                                      //получаем данные с atmega8a

    if (rpm1 > rpm_min1 * 1.5) fake_rpm1 = true;                     //проверка на бред тахометра. если бред поймали, то флаг дальше не даст снижать rpm_min. залман, сволочь, после бреда начинает честно сообщать обороты
    if ((rpm1 < rpm_min1) && !fake_rpm1 && rpm1 != 0) {              //проверяем, что вент замедляется. третье условие необходимо для проверки что вент вообще подключен
      if (!flag_rpm1_up) flag_rpm1_up = true; else {
        if (!flag_rpm11_up) {
          flag_rpm11_up = true;
          pwm1_up = i;                                                 //верхняя граница pwm при которой начинает реагировать вент - снижение скорости второй раз, а значит гарантированно снижаются обороты. первым сниженеим пренебрегаем, т.к. есть девиация показаний
        }
      }
      rpm_min1 = rpm1;
      pwm_min1 = i;
    }
    if (rpm2 > rpm_min2 * 1.5) fake_rpm2 = true;
    if ((rpm2 < rpm_min2) && !fake_rpm2 && rpm2 != 0) {
      if (!flag_rpm2_up) flag_rpm2_up = true; else {
        if (!flag_rpm22_up) {
          flag_rpm22_up = true;
          pwm2_up = i;                                                 //верхняя граница pwm при которой начинает реагировать вент - снижение скорости второй раз, а значит гарантированно снижаются обороты. первым сниженеим пренебрегаем, т.к. есть девиация показаний
        }
      }
      rpm_min2 = rpm2;
      pwm_min2 = i;
    }
    if (rpm3 > rpm_min3 * 1.5) fake_rpm3 = true;
    if ((rpm3 < rpm_min3) && !fake_rpm3 && rpm3 != 0) {
      if (!flag_rpm3_up) flag_rpm3_up = true; else {
        if (!flag_rpm33_up) {
          flag_rpm33_up = true;
          pwm3_up = i;                                                 //верхняя граница pwm при которой начинает реагировать вент - снижение скорости второй раз, а значит гарантированно снижаются обороты. первым сниженеим пренебрегаем, т.к. есть девиация показаний
        }
      }
      rpm_min3 = rpm3;
      pwm_min3 = i;
    }
    //   Serial.println();
  //  Serial.print("flag_rpm1_up="); Serial.print(flag_rpm1_up); Serial.print( "flag_rpm11_up="); Serial.print(flag_rpm11_up);
    Serial.print(" pwm1_up="); Serial.print(pwm1_up); Serial.print(" pwm2_up="); Serial.print(pwm2_up); Serial.print(" pwm3_up="); Serial.println(pwm3_up);
    if (rpm1 == 0 && !fault_rotation1) {
      pwm_stop1 = i;
      fault_rotation1 = true;
    }
    if (rpm2 == 0 && !fault_rotation2) {
      pwm_stop2 = i;
      fault_rotation2 = true;
    }
    if (rpm3 == 0 && !fault_rotation3) {
      pwm_stop3 = i;
      fault_rotation3 = true;
    }
    if ((rpm1 == 0) && (rpm2 == 0) && (rpm3 == 0)) break;
    lcd_cal();
  }
  lcd.home(); lcd.clear();
  lcd.print("test end");
  lcd.setCursor(0, 1);
  lcd.print("vent stopping");
  delay(view_time);
  lcd.home(); lcd.clear();
  lcd.print("max:"); delay(view_time / 2);
  lcd.home(); lcd.clear();
  lcd.print(rpm_max1 * 30);
  lcd.setCursor(5, 0);
  lcd.print(rpm_max2 * 30);
  lcd.setCursor(10, 0);
  lcd.print(rpm_max3 * 30);
  lcd.setCursor(0, 1); lcd.print(pwm1_up); lcd.setCursor(5, 1); lcd.print(pwm2_up); lcd.setCursor(10, 5); lcd.print(pwm3_up);
  delay(view_time);
  lcd.home(); lcd.clear();
  lcd.print("min:"); delay(view_time / 2);
  lcd.home(); lcd.clear();

  if (rpm_min1 != 32767) {        //если rpm_min осталось с базовым значением, т.е. ни разу не изменилось - тогда вента не было совсем.
    lcd.setCursor(0, 0);
    lcd.print(rpm_min1 * 30);
    lcd.setCursor(0, 1);
    lcd.print(pwm_min1);
    vent1 = true;
  } else {
    lcd.setCursor(0, 0);
    lcd.print("n/a");
  }
  if (rpm_min2 != 32767) {
    lcd.setCursor(5, 0);
    lcd.print(rpm_min2 * 30);
    lcd.setCursor(5, 1);
    lcd.print(pwm_min2);
    vent2 = true;
  } else {
    lcd.setCursor(5, 0);
    lcd.print("n/a");
  }
  if (rpm_min3 != 32767) {
    lcd.setCursor(10, 0);
    lcd.print(rpm_min3 * 30);
    lcd.setCursor(10, 1);
    lcd.print(pwm_min3);
    vent3 = true;
  } else {
    lcd.setCursor(10, 0);
    lcd.print("n/a");
  }
  delay(view_time);
  lcd.home(); lcd.clear(); lcd.print("launching VENTS");
  pwm_start1 = pwm_min1;
  pwm_start2 = pwm_min2;
  pwm_start3 = pwm_min3;
  fault_rotation1 = true;
  fault_rotation2 = true;
  fault_rotation3 = true;
  while ((fault_rotation1 && vent1) || (fault_rotation2 && vent2) || (fault_rotation3 && vent3)) {          //проверяем что при минимальных значениях pwm_min вентиляторы способны раскрутится вновь.
    pwm1 = pwm_start1;
    pwm2 = pwm_start2;
    pwm3 = pwm_start3;
    analogWrite(pin_pwm_v1, pwm1);
    analogWrite(pin_pwm_v2, pwm2);
    analogWrite(pin_pwm_v3, pwm3);
    delay(tau2 * 2);                                                                                        //ждем раскрутки.
    read_rpm();
    if ((rpm1 == 0) && vent1) {                                                                             //добавляем pwm_start если вент есть и он стоит
      pwm_start1 += 1;
      fault_rotation1 = true;
    } else if (vent1) fault_rotation1 = false;
    if ((rpm2 == 0) && vent2) {
      pwm_start2 += 1;
      fault_rotation2 = true;
    } else if (vent2) fault_rotation2 = false;
    if ((rpm3 == 0) && vent3) {
      pwm_start3 += 1;
      fault_rotation3 = true;
    } else if (vent3) fault_rotation3 = false;
    lcd.setCursor(0, 1); lcd.print("                "); lcd.setCursor(0, 1); lcd.print(pwm_start1); lcd.setCursor(5, 1); lcd.print(pwm_start2); lcd.setCursor(10, 1); lcd.print(pwm_start3);
  }
  lcd.home();
  lcd.clear();
  lcd.print("Calibrate");
  lcd.setCursor(0, 1);
  lcd.print("finished");
  delay(view_time);
  pwm1 = pwm_min1;
  pwm2 = pwm_min2;
  pwm3 = pwm_min3;
  analogWrite(pin_pwm_v1, pwm1);
  analogWrite(pin_pwm_v2, pwm2);
  analogWrite(pin_pwm_v3, pwm3);
  EEPROM.update(addr_pwm_min1, pwm_min1);
  EEPROM.update(addr_pwm_min2, pwm_min2);
  EEPROM.update(addr_pwm_min3, pwm_min3);
  EEPROM.update(addr_pwm_stop1, pwm_stop1);
  EEPROM.update(addr_pwm_stop2, pwm_stop2);
  EEPROM.update(addr_pwm_stop3, pwm_stop3);
  EEPROM.update(addr_pwm_start1, pwm_start1);
  EEPROM.update(addr_pwm_start2, pwm_start2);
  EEPROM.update(addr_pwm_start3, pwm_start3);
  EEPROM.update(addr_vent1, vent1);
  EEPROM.update(addr_vent2, vent2);
  EEPROM.update(addr_vent3, vent3);
  EEPROM.update(addr_pwm1, pwm1);
  EEPROM.update(addr_pwm2, pwm2);
  EEPROM.update(addr_pwm3, pwm3);
  EEPROM.update(addr_pwm1_up, pwm1_up);
  EEPROM.update(addr_pwm2_up, pwm2_up);
  EEPROM.update(addr_pwm3_up, pwm3_up);
}

void call_temp_controll() {

  //реакция на превышение температуры. временной интервал прироста pwm - tau atmega8a
  //проц
  if (t_probe1 > t_alarm1) {          //видим превышение
    if (!alarm1) {                    //при первом входе за инцидент
      alarm1 = true;                  //алярма!!
      Serial.println("alarm1!!");
      pwm1_a = pwm1;                  //при выходе вверх за порог, отталкиваемся от текущего значения ШИМ
    }
    //    dtt1 = t_probe1 - t_pred1;        //считаем дельту (текущая-предыдущая температура)
    //    if (dtt1 > 0) pwm1_a += step_alarm;        //увеличиваем аварийное значение ШИМ на (аварийный шаг)
    pwm1_a += step_alarm;
    if (pwm1_a > 255) pwm1_a = 255;
    Serial.println(pwm1_a);
    analogWrite(pin_pwm_v1, pwm1_a);
  } else {                            //температура ниже пороговой, ниже в коде проверяем была ли алярма
    if (alarm1) {                     //алярма была, и:
      if ((t_probe1 + t_post_alarm) < t_alarm1) { //и: ниже пороговой на t_post_alarm выход за порог вниз, возвращаем запомненное значение оборотов, отбой тревоги. иначе будет крутиться последние значение rpm, при которых пошло снижение t
        alarm1 = false;
        analogWrite(pin_pwm_v1, pwm1);
      } else analogWrite(pin_pwm_v1, pwm1_a);     //если ниже пороговой, но в районе t_post_alarm, то продолжаем крутиться на последнем аварийном значении rpm.
    }
  }                                   //_____
  //мать
  if (t_probe2 > t_alarm2) {          //видим превышение
    if (!alarm2) {                    //при первом входе за инцидент
      alarm2 = true;                  //алярма!!
      pwm2_a = pwm2;                  //при выходе вверх за порог, отталкиваемся от текущего значения ШИМ
    }
    dtt2 = t_probe2 - t_pred2;
    pwm2_a += step_alarm * dtt2;           //увеличиваем аварийное значение ШИМ на аварийный шаг
    if (pwm2_a > 255) pwm2_a = 255;
    analogWrite(pin_pwm_v2, pwm2_a);
  } else {                            //температура ниже пороговой, ниже проверяем была ли алярма
    if (alarm2) {                     //алярма была, выход за порог вниз, см для канала 1.
      if ((t_probe2 + t_post_alarm) < t_alarm2) {
        alarm2 = false;
        analogWrite(pin_pwm_v2, pwm2);
      } else analogWrite(pin_pwm_v2, pwm2_a);
    }
  }                                   //_____
  //БП
  if (t_probe3 > t_alarm3) {          //видим превышение
    if (!alarm3) {                    //при первом входе за инцидент
      alarm3 = true;                  //алярма!!
      pwm3_a = pwm3;                  //при выходе вверх за порог, отталкиваемся от текущего значения ШИМ
    }
    dtt3 = t_probe3 - t_pred3;
    pwm3_a += step_alarm * dtt3;           //увеличиваем аварийное значение ШИМ на аварийный шаг
    if (pwm3_a > 255) pwm3_a = 255;
    analogWrite(pin_pwm_v3, pwm3_a);
  } else {                            //температура ниже пороговой, ниже проверяем была ли алярма
    if (alarm3) {                     //алярма была, выход за порог вниз, возвращаем запомненное значение оборотов, отбой тревоги
      if ((t_probe3 + t_post_alarm) < t_alarm3) {
        alarm3 = false;
        analogWrite(pin_pwm_v3, pwm3);
      } else analogWrite(pin_pwm_v3, pwm3_a);
    }
  }
}


void list_enc() {
  enc_new_pos = enc.read();                                           //слушаем энкодер;
  if (((enc_pos / 2) - (enc_new_pos / 2)) > 0) {                      // деление на /2 получено эксперементальным путем
    //  sw_flag = true;                                                   //флаг изменения управлялки
    time_sw = millis();                                               //точка отсчета отображения переменных данных
    switch (sw_count) {
      case 1:
        pwm1 -= (step_pwm * abs(enc_pos / 2 - enc_new_pos / 2));      //увеличиваем значение ШИМ на заданный шаг
        analogWrite(pin_pwm_v1, pwm1);
        flag_pwm_view = true;
        //       lcd_pwm();
        break;
      case 2:
        pwm2 -= (step_pwm * abs(enc_pos / 2 - enc_new_pos / 2));      //увеличиваем значение ШИМ на заданный шаг
        analogWrite(pin_pwm_v2, pwm2);
        flag_pwm_view = true;
        //       lcd_pwm();
        break;
      case 3:
        pwm3 -= (step_pwm * abs(enc_pos / 2 - enc_new_pos / 2));      //увеличиваем значение ШИМ на заданный шаг
        analogWrite(pin_pwm_v3, pwm3);
        flag_pwm_view = true;
        //       lcd_pwm();
        break;
    }
    // Serial.println(millis() - time_tau);
    if ((millis() - time_tau) > tau_atmega8a / 2) {
      read_rpm();
      //     Serial.print("rpm1="); Serial.println(rpm1);
      lcd_rpm(rpm1, rpm2, rpm3);
      time_tau = millis();
    }

    enc_pos = enc_new_pos;
  }
  if (((enc_pos / 2) - (enc_new_pos / 2)) < 0) {                      // деление на /2 получено эксперементальным путем
    //  sw_flag = true;                                                   //флаг изменения управлялки
    time_sw = millis();                                               //точка отсчета отображения переменных данных
    switch (sw_count) {
      case 1:
        pwm1 += (step_pwm * abs(enc_pos / 2 - enc_new_pos / 2));      //уменьшаем значение ШИМ на заданный шаг
        analogWrite(pin_pwm_v1, pwm1);
        flag_pwm_view = true;
        //       lcd_pwm();
        break;
      case 2:
        pwm2 += (step_pwm * abs(enc_pos / 2 - enc_new_pos / 2));      //уменьшаем значение ШИМ на заданный шаг
        analogWrite(pin_pwm_v2, pwm2);
        flag_pwm_view = true;
        //       lcd_pwm();
        break;
      case 3:
        pwm3 += (step_pwm * abs(enc_pos / 2 - enc_new_pos / 2));      //уменьшаем значение ШИМ на заданный шаг
        analogWrite(pin_pwm_v3, pwm3);
        flag_pwm_view = true;
        //       lcd_pwm();
        break;
    }
    //  Serial.println(millis() - time_tau);
    if ((millis() - time_tau) > tau_atmega8a / 2) {
      read_rpm();
      //     Serial.print("rpm1="); Serial.println(rpm1);
      lcd_rpm(rpm1, rpm2, rpm3);
      time_tau = millis();
    }
    enc_pos = enc_new_pos;
  }
}

void push_enc() {
  time_sw = millis();                                                //точка отсчета отображения переменных данных
  time_tau = millis();
  flag_pwm_view = true;
  lcd_pwm();
  while ((millis() - time_sw <= view_time)) {
    if ((digitalRead(sw) == LOW) && !sw_flag) {                                       //слушаем нажатие энкодера.
      sw_count++;
      sw_flag = true;                                                   //флаг изменения управлялки
      flag_pwm_view = true;                                             //флаг, что надо показать pwm (переключается обратно в lcd_pwm)
      time_sw = millis();
    }
    if (digitalRead(sw) == HIGH) sw_flag = false;
    if (sw_count == 4) {
      lcd.setCursor(0, 1); lcd.print("                "); lcd.setCursor(0, 1); lcd.print("Calibrate?");
      enc_pos = enc_new_pos;
      delay(1000);
      enc_new_pos = enc.read();
      if ((enc_pos - enc_new_pos) > 5) {                                //подтверждение - поворот на 5 (для предотвращения ложных срабатываний)
        lcd.setCursor(0, 1); lcd.print("                "); lcd.setCursor(0, 1); lcd.print("Confirm?"); delay(500);
        enc_pos = enc_new_pos;
        delay(1000);
        enc_new_pos = enc.read();
        if ((enc_pos - enc_new_pos) > 5) {
          calib_vent();
          return;
        }
      }
    }
    if (sw_count > 3) sw_count = 1;

    list_enc();
    lcd_pwm();
    EEPROM.update(addr_pwm1, pwm1);
    EEPROM.update(addr_pwm2, pwm2);
    EEPROM.update(addr_pwm3, pwm3);

  }
}

void loop() {
  if (flag_loop_tau) {
    time1 = millis();                                                   //в начале цикла засекаем время
    probes.requestTemperatures();                                     //датчики начинают мерять температуру. время 94...750 мс (зависит от разрешения)
  }

  lcd_temp();

  call_temp_controll();                                               //проверка на алярмы по температуре

  if (digitalRead(sw) == LOW) {                                       //слушаем нажатие энкодера. обработка нажатия энкодера. нажали кнопку sw
    sw_flag = true;
    push_enc();
  }

  time1 = millis() - time1;            //замеряем время, затраченное с начала цикла.
  if (time1 < tau2) flag_loop_tau = false; else { //проверяем сколько времени прошло с начала цикла, если больше чем tau2 - то
    flag_loop_tau = true;
    t_pred1 = t_probe1;                                               //запоминаем температуру предыдущую
    t_pred2 = t_probe2;
    t_pred3 = t_probe3;
    temp_req();                         //получение температуры с датчиков.
    read_rpm();                          //получаем данные с atmega8a

    if (zero_data1) rpm_tt1 = rpm1; else {                     //проверяем флаг на первую пачку данных
      rpm_tt1 = 0;                          //зануляем сс
      for (int w = 0; w <= nn - 1; w++) {  //вычисление сс
        rpm_tt1 += rpm_s1[w];
      }
      rpm_tt1 = rpm_tt1 + (rpm1);
      rpm_tt1 = rpm_tt1 / (nn + 1);
    }
    rpm_s1[n] = rpm_tt1;                    //расчет сс закончен

    if (zero_data2) rpm_tt2 = rpm2; else {
      rpm_tt2 = 0;
      for (int w = 0; w <= nn - 1; w++) {  //вычисление сс
        rpm_tt2 += rpm_s2[w];
      }
      rpm_tt2 = rpm_tt2 + (rpm2);
      rpm_tt2 = rpm_tt2 / (nn + 1);
    }
    rpm_s2[n] = rpm_tt2;

    if (zero_data3) rpm_tt3 = rpm3; else {
      rpm_tt3 = 0;
      for (int w = 0; w <= nn - 1; w++) {  //вычисление сс
        rpm_tt3 += rpm_s3[w];
      }
      rpm_tt3 = rpm_tt3 + (rpm3);
      rpm_tt3 = rpm_tt3 / (nn + 1);
    }
    rpm_s3[n] = rpm_tt3;

    lcd_rpm(rpm_tt1, rpm_tt2, rpm_tt3);                          //выводим обороты
    n++;
    if (n > nn) {
      n = 0;
      zero_data1 = false;
      zero_data2 = false;
      zero_data3 = false;
    }
  }
}
