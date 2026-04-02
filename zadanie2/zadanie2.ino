/*
 * Zadanie 2 – poprawiona wersja zgodna z PDF i zachowaniem przycisków z Alarm_Clock.
 * FreeRTOS: osobne wątki (display/buttons/clock), kolejka strukturalna + semafory binarne.
 */

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>

#define PIN_LATCH   4
#define PIN_CLK     7
#define PIN_DATA    8
#define PIN_DIG1    6
#define PIN_DIG2    11
#define PIN_DIG3    10
#define PIN_DIG4    9
#define PIN_BTN1    A1
#define PIN_BTN2    A2
#define PIN_BTN3    A3
#define PIN_BUZZER  3

static const uint8_t SEG_MAP[11] = {
  0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0xFF
};
#define BLANK     10u
#define DOT_MASK  0x7Fu

typedef struct {
  uint8_t d[4];
  bool dot;
  bool buzzer;
} DisplayMsg_t;

typedef enum {
  MODE_CLOCK_TIME,
  MODE_CLOCK_TIME_SET_HOUR,
  MODE_CLOCK_TIME_SET_MINUTE,
  MODE_ALARM_TIME,
  MODE_ALARM_TIME_SET_HOUR,
  MODE_ALARM_TIME_SET_MINUTE
} DisplayMode_t;

static QueueHandle_t xDisplayQueue;
static SemaphoreHandle_t xSemB1Press, xSemB1Long;
static SemaphoreHandle_t xSemB2Press, xSemB2Release;
static SemaphoreHandle_t xSemB3Press, xSemB3Long, xSemB3LongRelease;

static void vDisplayTask(void *);
static void vButtonTask(void *);
static void vClockTask(void *);

static void writeToDisplay(uint8_t digitPin, uint8_t segByte)
{
  digitalWrite(PIN_DIG1, HIGH);
  digitalWrite(PIN_DIG2, HIGH);
  digitalWrite(PIN_DIG3, HIGH);
  digitalWrite(PIN_DIG4, HIGH);

  digitalWrite(PIN_LATCH, LOW);
  shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, segByte);
  digitalWrite(PIN_LATCH, HIGH);

  digitalWrite(digitPin, LOW);
}

void setup()
{
  Serial.begin(9600);

  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_DIG1, OUTPUT);
  pinMode(PIN_DIG2, OUTPUT);
  pinMode(PIN_DIG3, OUTPUT);
  pinMode(PIN_DIG4, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_BTN3, INPUT_PULLUP);

  xDisplayQueue = xQueueCreate(1, sizeof(DisplayMsg_t));
  xSemB1Press = xSemaphoreCreateBinary();
  xSemB1Long = xSemaphoreCreateBinary();
  xSemB2Press = xSemaphoreCreateBinary();
  xSemB2Release = xSemaphoreCreateBinary();
  xSemB3Press = xSemaphoreCreateBinary();
  xSemB3Long = xSemaphoreCreateBinary();
  xSemB3LongRelease = xSemaphoreCreateBinary();

  xTaskCreate(vDisplayTask, "Disp", 180, NULL, 3, NULL);
  xTaskCreate(vButtonTask, "Btn", 180, NULL, 2, NULL);
  xTaskCreate(vClockTask, "Clock", 320, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {}

static void vDisplayTask(void *)
{
  static const uint8_t digitPins[4] = {PIN_DIG1, PIN_DIG2, PIN_DIG3, PIN_DIG4};
  DisplayMsg_t msg = {{BLANK, BLANK, BLANK, BLANK}, false, false};
  uint8_t pos = 0;

  for (;;)
  {
    DisplayMsg_t tmp;
    if (xQueueReceive(xDisplayQueue, &tmp, 0) == pdPASS) {
      msg = tmp;
    }

    digitalWrite(PIN_BUZZER, msg.buzzer ? HIGH : LOW);

    uint8_t seg = SEG_MAP[msg.d[pos]];
    if (msg.dot && pos == 1u) {
      seg &= DOT_MASK;
    }

    writeToDisplay(digitPins[pos], seg);
    pos = (uint8_t)((pos + 1u) & 0x03u);
    vTaskDelay(pdMS_TO_TICKS(3));
  }
}

static void vButtonTask(void *)
{
  const TickType_t poll = pdMS_TO_TICKS(20);
  const uint8_t longTicksThreshold = 25; // 25*20ms = 500ms

  bool prev1 = true, prev2 = true, prev3 = true;
  uint8_t hold1 = 0, hold3 = 0;
  bool longSent1 = false, longSent3 = false;

  for (;;)
  {
    bool b1 = (digitalRead(PIN_BTN1) == HIGH);
    bool b2 = (digitalRead(PIN_BTN2) == HIGH);
    bool b3 = (digitalRead(PIN_BTN3) == HIGH);

    // BUTTON1
    if (!b1 && prev1) { hold1 = 0; longSent1 = false; }
    if (!b1) {
      if (hold1 < 255u) hold1++;
      if (!longSent1 && hold1 >= longTicksThreshold) {
        xSemaphoreGive(xSemB1Long);
        longSent1 = true;
      }
    }
    if (b1 && !prev1 && !longSent1) {
      xSemaphoreGive(xSemB1Press);
    }

    // BUTTON2
    if (!b2 && prev2) xSemaphoreGive(xSemB2Press);
    if (b2 && !prev2) xSemaphoreGive(xSemB2Release);

    // BUTTON3
    if (!b3 && prev3) { hold3 = 0; longSent3 = false; }
    if (!b3) {
      if (hold3 < 255u) hold3++;
      if (!longSent3 && hold3 >= longTicksThreshold) {
        xSemaphoreGive(xSemB3Long);
        longSent3 = true;
      }
    }
    if (b3 && !prev3) {
      if (longSent3) xSemaphoreGive(xSemB3LongRelease);
      else xSemaphoreGive(xSemB3Press);
    }

    prev1 = b1;
    prev2 = b2;
    prev3 = b3;

    vTaskDelay(poll);
  }
}

static void vClockTask(void *)
{
  uint8_t h = 12, m = 0, s = 0;
  uint8_t alH = 6, alM = 30;
  bool alarmEnabled = false;
  bool alarmRinging = false;

  bool snooze = false;
  uint8_t snoozeSec = 0;

  bool alarmTogglePressed = false;
  DisplayMode_t mode = MODE_CLOCK_TIME;

  TickType_t xLastWake = xTaskGetTickCount();
  TickType_t xLastSec = xTaskGetTickCount();

  for (;;)
  {
    bool b1Press = (xSemaphoreTake(xSemB1Press, 0) == pdPASS);
    bool b1Long = (xSemaphoreTake(xSemB1Long, 0) == pdPASS);
    bool b2Press = (xSemaphoreTake(xSemB2Press, 0) == pdPASS);
    bool b2Release = (xSemaphoreTake(xSemB2Release, 0) == pdPASS);
    bool b3Press = (xSemaphoreTake(xSemB3Press, 0) == pdPASS);
    bool b3Long = (xSemaphoreTake(xSemB3Long, 0) == pdPASS);
    bool b3LongRelease = (xSemaphoreTake(xSemB3LongRelease, 0) == pdPASS);

    if (alarmRinging) {
      if (b2Press) {
        alarmRinging = false;
        snooze = true;
        snoozeSec = 20;
        Serial.println(F("Drzemka 20s"));
      }
      if (b1Press || b1Long || b2Release) {
        alarmRinging = false;
        snooze = false;
        Serial.println(F("Alarm wylaczony"));
      }
    }

    switch (mode)
    {
      case MODE_CLOCK_TIME:
        if (b2Press) {
          mode = MODE_ALARM_TIME;
        } else if (b1Long) {
          mode = MODE_CLOCK_TIME_SET_HOUR;
          s = 0;
        } else if (b3Long && !alarmTogglePressed) {
          alarmTogglePressed = true;
          alarmEnabled = !alarmEnabled;
        } else if (b3LongRelease) {
          alarmTogglePressed = false;
        }
        break;

      case MODE_CLOCK_TIME_SET_HOUR:
        if (b1Press) mode = MODE_CLOCK_TIME_SET_MINUTE;
        else if (b3Press || b3Long) h = (uint8_t)((h + 1u) % 24u);
        break;

      case MODE_CLOCK_TIME_SET_MINUTE:
        if (b1Press) mode = MODE_CLOCK_TIME;
        else if (b3Press || b3Long) m = (uint8_t)((m + 1u) % 60u);
        break;

      case MODE_ALARM_TIME:
        if (b2Release) mode = MODE_CLOCK_TIME;
        else if (b1Long) mode = MODE_ALARM_TIME_SET_HOUR;
        break;

      case MODE_ALARM_TIME_SET_HOUR:
        if (b1Press) mode = MODE_ALARM_TIME_SET_MINUTE;
        else if (b3Press || b3Long) alH = (uint8_t)((alH + 1u) % 24u);
        break;

      case MODE_ALARM_TIME_SET_MINUTE:
        if (b1Press) {
          mode = MODE_CLOCK_TIME;
          alarmEnabled = true;
        } else if (b3Press || b3Long) alM = (uint8_t)((alM + 1u) % 60u);
        break;
    }

    TickType_t xNow = xTaskGetTickCount();
    while ((xNow - xLastSec) >= pdMS_TO_TICKS(1000))
    {
      xLastSec += pdMS_TO_TICKS(1000);
      s++;
      if (s >= 60u) {
        s = 0u;
        m++;
        if (m >= 60u) {
          m = 0u;
          h = (uint8_t)((h + 1u) % 24u);
        }

        if (alarmEnabled && !alarmRinging && !snooze && h == alH && m == alM) {
          alarmRinging = true;
          Serial.println(F("ALARM!"));
        }
      }

      if (snooze && snoozeSec > 0u) {
        snoozeSec--;
        if (snoozeSec == 0u) {
          snooze = false;
          alarmRinging = true;
          Serial.println(F("ALARM! (po drzemce)"));
        }
      }
    }

    DisplayMsg_t msg;
    msg.dot = ((xNow % pdMS_TO_TICKS(1000)) < pdMS_TO_TICKS(500));
    msg.buzzer = alarmRinging;

    switch (mode)
    {
      case MODE_CLOCK_TIME:
        msg.d[0] = h / 10u; msg.d[1] = h % 10u; msg.d[2] = m / 10u; msg.d[3] = m % 10u;
        break;
      case MODE_CLOCK_TIME_SET_HOUR:
        msg.d[0] = msg.dot ? (h / 10u) : BLANK;
        msg.d[1] = msg.dot ? (h % 10u) : BLANK;
        msg.d[2] = m / 10u; msg.d[3] = m % 10u;
        msg.dot = false;
        break;
      case MODE_CLOCK_TIME_SET_MINUTE:
        msg.d[0] = h / 10u; msg.d[1] = h % 10u;
        msg.d[2] = msg.dot ? (m / 10u) : BLANK;
        msg.d[3] = msg.dot ? (m % 10u) : BLANK;
        msg.dot = false;
        break;
      case MODE_ALARM_TIME:
        msg.d[0] = alH / 10u; msg.d[1] = alH % 10u; msg.d[2] = alM / 10u; msg.d[3] = alM % 10u;
        break;
      case MODE_ALARM_TIME_SET_HOUR:
        msg.d[0] = msg.dot ? (alH / 10u) : BLANK;
        msg.d[1] = msg.dot ? (alH % 10u) : BLANK;
        msg.d[2] = alM / 10u; msg.d[3] = alM % 10u;
        msg.dot = false;
        break;
      default:
        msg.d[0] = alH / 10u; msg.d[1] = alH % 10u;
        msg.d[2] = msg.dot ? (alM / 10u) : BLANK;
        msg.d[3] = msg.dot ? (alM % 10u) : BLANK;
        msg.dot = false;
        break;
    }

    xQueueOverwrite(xDisplayQueue, &msg);
    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(20));
  }
}
