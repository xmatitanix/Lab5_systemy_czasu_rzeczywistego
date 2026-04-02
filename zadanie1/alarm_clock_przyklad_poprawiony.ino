#include <TimerOne.h>
#include <Wire.h>
#include <MultiFuncShield.h>

volatile unsigned int clockMilliSeconds = 0;
volatile byte clockSeconds = 0;
volatile byte clockMinutes = 0;
volatile byte clockHours = 12;
volatile byte clockEnabled = 1;

byte alarmMinutes = 30;
byte alarmHours = 6;
volatile byte alarmEnabled = false;
byte alarmTogglePressed = false;

enum displayModeValues {
  MODE_CLOCK_TIME,
  MODE_CLOCK_TIME_SET_HOUR,
  MODE_CLOCK_TIME_SET_MINUTE,
  MODE_ALARM_TIME,
  MODE_ALARM_TIME_SET_HOUR,
  MODE_ALARM_TIME_SET_MINUTE
};

byte displayMode = MODE_CLOCK_TIME;

void displayTime(byte hours, byte minutes) {
  char time[5];
  sprintf(time, "%04u", (unsigned)((hours * 100u) + minutes)); // poprawka: 4 cyfry
  MFS.write(time, 1);
}

void clockISR() {
  if (!clockEnabled) return;

  clockMilliSeconds++;
  if (clockMilliSeconds < 1000) return;

  clockMilliSeconds = 0;
  clockSeconds++;
  if (clockSeconds < 60) return;

  clockSeconds = 0;
  clockMinutes++;
  if (clockMinutes >= 60) {
    clockMinutes = 0;
    clockHours = (clockHours + 1u) % 24u;
  }

  if (alarmEnabled && (clockMinutes == alarmMinutes) && (clockHours == alarmHours)) {
    MFS.beep(10, 5, 4, 100, 50);
  }
}

void setup() {
  Timer1.initialize();
  MFS.userInterrupt = clockISR;
  MFS.initialize(&Timer1);
  MFS.blinkDisplay(DIGIT_ALL);
}

void loop() {
  byte btn = MFS.getButton();

  switch (displayMode) {
    case MODE_CLOCK_TIME:
      displayTime(clockHours, clockMinutes);
      if (btn == BUTTON_2_PRESSED) {
        MFS.beep(0);
        displayMode = MODE_ALARM_TIME;
      } else if (btn == BUTTON_1_LONG_PRESSED) {
        MFS.blinkDisplay(DIGIT_ALL, OFF);
        MFS.blinkDisplay(DIGIT_1 | DIGIT_2);
        displayMode = MODE_CLOCK_TIME_SET_HOUR;
        clockEnabled = false;
        clockMilliSeconds = 0;
        clockSeconds = 0;
      } else if (btn == BUTTON_3_LONG_PRESSED && !alarmTogglePressed) {
        alarmTogglePressed = true;
        alarmEnabled = !alarmEnabled;
        MFS.writeLeds(LED_1, alarmEnabled);
      } else if (btn == BUTTON_3_LONG_RELEASE) {
        alarmTogglePressed = false;
      }
      break;

    case MODE_CLOCK_TIME_SET_HOUR:
      if (btn == BUTTON_1_PRESSED) {
        MFS.blinkDisplay(DIGIT_1 | DIGIT_2, OFF);
        MFS.blinkDisplay(DIGIT_3 | DIGIT_4);
        displayMode = MODE_CLOCK_TIME_SET_MINUTE;
      } else if (btn == BUTTON_3_PRESSED || btn == BUTTON_3_LONG_PRESSED) {
        clockHours = (clockHours + 1u) % 24u;
        displayTime(clockHours, clockMinutes);
      }
      break;

    case MODE_CLOCK_TIME_SET_MINUTE:
      if (btn == BUTTON_1_PRESSED) {
        MFS.blinkDisplay(DIGIT_3 | DIGIT_4, OFF);
        displayMode = MODE_CLOCK_TIME;
        clockEnabled = true;
      } else if (btn == BUTTON_3_PRESSED || btn == BUTTON_3_LONG_PRESSED) {
        clockMinutes = (clockMinutes + 1u) % 60u;
        displayTime(clockHours, clockMinutes);
      }
      break;

    case MODE_ALARM_TIME:
      displayTime(alarmHours, alarmMinutes);
      if (btn == BUTTON_2_SHORT_RELEASE || btn == BUTTON_2_LONG_RELEASE) {
        displayMode = MODE_CLOCK_TIME;
      } else if (btn == BUTTON_1_LONG_PRESSED) {
        MFS.blinkDisplay(DIGIT_ALL, OFF);
        MFS.blinkDisplay(DIGIT_1 | DIGIT_2);
        displayMode = MODE_ALARM_TIME_SET_HOUR;
        alarmEnabled = false;
      }
      break;

    case MODE_ALARM_TIME_SET_HOUR:
      if (btn == BUTTON_1_PRESSED) {
        MFS.blinkDisplay(DIGIT_1 | DIGIT_2, OFF);
        MFS.blinkDisplay(DIGIT_3 | DIGIT_4);
        displayMode = MODE_ALARM_TIME_SET_MINUTE;
      } else if (btn == BUTTON_3_PRESSED || btn == BUTTON_3_LONG_PRESSED) {
        alarmHours = (alarmHours + 1u) % 24u;
        displayTime(alarmHours, alarmMinutes);
      }
      break;

    case MODE_ALARM_TIME_SET_MINUTE:
      if (btn == BUTTON_1_PRESSED) {
        MFS.blinkDisplay(DIGIT_3 | DIGIT_4, OFF);
        displayMode = MODE_CLOCK_TIME;
        alarmEnabled = true;
        MFS.writeLeds(LED_1, ON);
      } else if (btn == BUTTON_3_PRESSED || btn == BUTTON_3_LONG_PRESSED) {
        alarmMinutes = (alarmMinutes + 1u) % 60u;
        displayTime(alarmHours, alarmMinutes);
      }
      break;
  }
}
