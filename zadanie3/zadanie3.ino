/*
 * ============================================================
 * Zadanie 3 – Zegar z kalendarzem, DST, dwoma alarmami
 * Systemy Czasu Rzeczywistego – Laboratorium 5
 * Sprzęt: Arduino Uno + MultiFuncShield
 * Biblioteka: Arduino_FreeRTOS
 * ============================================================
 *
 * ROZSZERZENIA WZGLĘDEM ZADANIA 2:
 *   1. Kalendarz (rok, miesiąc, dzień) z obsługą lat przestępnych.
 *   2. Rotacja wyświetlacza:
 *        [HH.MM] przez 20 s → [RRRR] przez 1 s → [MM.DD] przez 1 s → powrót
 *   3. Automatyczna zmiana czasu zimowego/letniego (Polska):
 *        - ostatnia niedziela marca  godzina 2:00 → 3:00 (lato)
 *        - ostatnia niedziela października godzina 3:00 → 2:00 (zima)
 *   4. Dwa alarmy (al[0] i al[1]).
 *   5. Nastawy daty przez Serial Monitor – komendy:
 *        DATE RRRR MM DD   np. DATE 2025 03 15
 *        TIME GG MM SS     np. TIME 12 30 00
 *        ALARM1 GG MM      np. ALARM1 07 00
 *        ALARM2 GG MM      np. ALARM2 08 30
 *
 * ARCHITEKTURA WĄTKÓW (identyczna jak w Zadaniu 2):
 *   vDisplayTask  (prio 3) – przemiatanie wyświetlacza
 *   vButtonTask   (prio 2) – detekcja S1/S2/S3, wysyła semafory
 *   vClockTask    (prio 1) – logika czasu/kalendarz/DST/alarmy
 *
 * KOMUNIKACJA:
 *   xDisplayQueue  – kolejka strukturalna głębokości 1 (xQueueOverwrite)
 *   xSem_S1/S2/S3  – semafory binarne przycisków
 *
 * OBSŁUGA PRZYCISKÓW:
 *   Tryb 0 (normalny):
 *     S1 kolejno → tryby 1-6 → powrót do 0
 *   Tryby 1-6 (ustawianie):
 *     1 = godz. zegara, 2 = min. zegara
 *     3 = godz. alarm1, 4 = min. alarm1
 *     5 = godz. alarm2, 6 = min. alarm2
 *     S2 = +1 | S3 = -1
 *   Alarm dzwoni:
 *     S1 = wyłącz | S2 = drzemka 20 s
 * ============================================================ */

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <string.h>

/* ============================================================
 * PINY
 * ============================================================ */
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

/* ============================================================
 * MAPA SEGMENTÓW (common-anode, active LOW)
 * ============================================================ */
static const uint8_t SEG_MAP[11] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99,
    0x92, 0x82, 0xF8, 0x80, 0x90,
    0xFF  // BLANK
};
#define BLANK     10u
#define DOT_MASK  0x7Fu

/* ============================================================
 * KOLEJKA I SEMAFORY
 * ============================================================ */
typedef struct {
    uint8_t d[4];
    bool    dot;
    bool    buzzer;
} DisplayMsg_t;

static QueueHandle_t     xDisplayQueue;
static SemaphoreHandle_t xSem_S1;
static SemaphoreHandle_t xSem_S2;
static SemaphoreHandle_t xSem_S3;

/* ============================================================
 * STRUKTURA ALARMU
 * ============================================================ */
typedef struct {
    uint8_t h;
    uint8_t m;
    bool    enabled;
    bool    ringing;
    bool    snooze;
    uint8_t snoozeLeft;
} Alarm_t;

/* ============================================================
 * PROTOTYPY
 * ============================================================ */
static void vDisplayTask(void *pvParameters);
static void vButtonTask (void *pvParameters);
static void vClockTask  (void *pvParameters);

/* ---- funkcje pomocnicze kalendarza ---- */
static bool    isLeap      (uint16_t y);
static uint8_t daysInMonth (uint16_t y, uint8_t mo);
static uint8_t dayOfWeek   (uint16_t y, uint8_t mo, uint8_t d);
static uint8_t lastSunday  (uint16_t y, uint8_t mo);
static void    advanceDay  (uint16_t *y, uint8_t *mo, uint8_t *d);
static void    parseSerial (const char *cmd,
                             uint16_t *y, uint8_t *mo, uint8_t *dy,
                             uint8_t  *h, uint8_t *m,  uint8_t *s,
                             Alarm_t  alarms[], bool *isDST);

/* ============================================================
 * SETUP
 * ============================================================ */
void setup()
{
    Serial.begin(9600);
    Serial.println(F("Zegar startuje. Komendy: DATE RRRR MM DD | TIME GG MM SS | ALARM1 GG MM | ALARM2 GG MM"));

    pinMode(PIN_LATCH,  OUTPUT);
    pinMode(PIN_CLK,    OUTPUT);
    pinMode(PIN_DATA,   OUTPUT);
    pinMode(PIN_DIG1,   OUTPUT);
    pinMode(PIN_DIG2,   OUTPUT);
    pinMode(PIN_DIG3,   OUTPUT);
    pinMode(PIN_DIG4,   OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_BTN1,   INPUT_PULLUP);
    pinMode(PIN_BTN2,   INPUT_PULLUP);
    pinMode(PIN_BTN3,   INPUT_PULLUP);

    digitalWrite(PIN_DIG1,   HIGH);
    digitalWrite(PIN_DIG2,   HIGH);
    digitalWrite(PIN_DIG3,   HIGH);
    digitalWrite(PIN_DIG4,   HIGH);
    digitalWrite(PIN_BUZZER, LOW);

    xDisplayQueue = xQueueCreate(1, sizeof(DisplayMsg_t));
    xSem_S1       = xSemaphoreCreateBinary();
    xSem_S2       = xSemaphoreCreateBinary();
    xSem_S3       = xSemaphoreCreateBinary();

    xTaskCreate(vDisplayTask, "Disp",  128, NULL, 3, NULL);
    xTaskCreate(vButtonTask,  "Btn",   128, NULL, 2, NULL);
    xTaskCreate(vClockTask,   "Clock", 384, NULL, 1, NULL); // większy stos przez parseSerial

    vTaskStartScheduler();
}

void loop() { }

/* ============================================================
 * FUNKCJA POMOCNICZA: zapis do rejestru + aktywacja cyfry
 * ============================================================ */
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

/* ============================================================
 * WĄTEK 1: vDisplayTask – przemiatanie wyświetlacza
 * (identyczny jak w Zadaniu 2)
 * ============================================================ */
static void vDisplayTask(void *pvParameters)
{
    static const uint8_t digitPins[4] = { PIN_DIG1, PIN_DIG2, PIN_DIG3, PIN_DIG4 };

    DisplayMsg_t msg;
    msg.d[0] = BLANK; msg.d[1] = BLANK;
    msg.d[2] = BLANK; msg.d[3] = BLANK;
    msg.dot    = false;
    msg.buzzer = false;

    uint8_t pos = 0;

    for (;;)
    {
        DisplayMsg_t tmp;
        if (xQueueReceive(xDisplayQueue, &tmp, 0) == pdPASS)
            msg = tmp;

        digitalWrite(PIN_BUZZER, msg.buzzer ? HIGH : LOW);

        uint8_t segByte = SEG_MAP[msg.d[pos]];
        if (msg.dot && (pos == 1))
            segByte &= DOT_MASK;

        writeToDisplay(digitPins[pos], segByte);
        pos = (pos + 1u) % 4u;

        vTaskDelay(1); // 1 tyk; przy tick=1ms → 5ms dla pdMS_TO_TICKS(5)
    }
}

/* ============================================================
 * WĄTEK 2: vButtonTask – detekcja przycisków
 * (identyczny jak w Zadaniu 2)
 * ============================================================ */
static void vButtonTask(void *pvParameters)
{
    static const uint8_t     btnPins[3] = { PIN_BTN1, PIN_BTN2, PIN_BTN3 };
    static SemaphoreHandle_t sems[3];
    sems[0] = xSem_S1; sems[1] = xSem_S2; sems[2] = xSem_S3;

    bool lastState[3] = { true, true, true };

    for (;;)
    {
        for (uint8_t i = 0; i < 3u; i++)
        {
            bool cur = (bool)digitalRead(btnPins[i]);
            if (!cur && lastState[i])
                xSemaphoreGive(sems[i]);
            lastState[i] = cur;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ============================================================
 * WĄTEK 3: vClockTask – zegar, kalendarz, DST, dwa alarmy
 *
 * ROTACJA WYŚWIETLACZA (w trybie 0):
 *   - Faza 0: [HH.MM] przez 20 sekund
 *   - Faza 1: [RRRR]  przez  1 sekundę
 *   - Faza 2: [MM.DD] przez  1 sekundę
 *   - powrót do fazy 0
 *
 * DST (czas letni/zimowy, Polska):
 *   - Ostatnia niedziela marca  godz. 2:00 → 3:00 (lato, UTC+2)
 *   - Ostatnia niedziela października godz. 3:00 → 2:00 (zima, UTC+1)
 * ============================================================ */
static void vClockTask(void *pvParameters)
{
    /* ---- czas ---- */
    uint8_t  h = 12, m = 0, s = 0;

    /* ---- data ---- */
    uint16_t year  = 2025;
    uint8_t  month = 1;
    uint8_t  day   = 1;

    /* ---- DST ---- */
    bool isDST = false;       // false = zima (CET), true = lato (CEST)
    bool dstDoneSpring = false; // czy zmiana wiosenna już wykonana w tym roku
    bool dstDoneFall   = false; // czy zmiana jesienna już wykonana w tym roku

    /* ---- dwa alarmy ---- */
    Alarm_t alarms[2];
    alarms[0].h = 7;  alarms[0].m = 0;  alarms[0].enabled = false;
    alarms[0].ringing = false; alarms[0].snooze = false; alarms[0].snoozeLeft = 0;
    alarms[1].h = 8;  alarms[1].m = 30; alarms[1].enabled = false;
    alarms[1].ringing = false; alarms[1].snooze = false; alarms[1].snoozeLeft = 0;

    /* ---- tryb ustawiania ---- */
    uint8_t mode = 0;
    /*
     * 0 – wyświetlanie  | 1 – godz. zegara   | 2 – min. zegara
     * 3 – godz. alarm1  | 4 – min. alarm1
     * 5 – godz. alarm2  | 6 – min. alarm2
     */

    /* ---- rotacja wyświetlacza ---- */
    uint8_t  dispPhase    = 0; // 0=HH.MM, 1=RRRR, 2=MM.DD
    uint8_t  phaseSecs    = 0; // sekundy spędzone w bieżącej fazie
    const uint8_t phaseLimit[3] = { 20u, 1u, 1u }; // czas trwania każdej fazy [s]

    /* ---- bufor Serial ---- */
    char    serialBuf[32];
    uint8_t serialIdx = 0;

    TickType_t xLastSec  = xTaskGetTickCount();
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        TickType_t xNow = xTaskGetTickCount();

        /* ================================================
         * 1. Odczyt semaforów przycisków
         * ================================================ */
        bool s1 = (xSemaphoreTake(xSem_S1, 0) == pdPASS);
        bool s2 = (xSemaphoreTake(xSem_S2, 0) == pdPASS);
        bool s3 = (xSemaphoreTake(xSem_S3, 0) == pdPASS);

        /* ================================================
         * 2. Obsługa alarmów gdy dzwonią
         * ================================================ */
        for (uint8_t i = 0; i < 2u; i++)
        {
            if (alarms[i].ringing)
            {
                if (s1)
                {
                    alarms[i].ringing = false;
                    s1 = false;
                    Serial.print(F("Alarm ")); Serial.print(i + 1); Serial.println(F(" wylaczony."));
                }
                else if (s2)
                {
                    alarms[i].ringing  = false;
                    alarms[i].snooze   = true;
                    alarms[i].snoozeLeft = 20u;
                    s2 = false;
                    Serial.print(F("Alarm ")); Serial.print(i + 1); Serial.println(F(" drzema 20s."));
                }
            }
        }

        /* ================================================
         * 3. Obsługa przycisków w trybach ustawiania
         * ================================================ */
        bool anyRinging = alarms[0].ringing || alarms[1].ringing;
        if (!anyRinging)
        {
            if (s1)
            {
                mode = (mode + 1u) % 7u;
                if (mode == 0) Serial.println(F("Ustawienia zapisane."));
                if (mode == 3) alarms[0].enabled = true;
                if (mode == 5) alarms[1].enabled = true;
            }
            if (s2)
            {
                if      (mode == 1) h           = (h           + 1u) % 24u;
                else if (mode == 2) m           = (m           + 1u) % 60u;
                else if (mode == 3) alarms[0].h = (alarms[0].h + 1u) % 24u;
                else if (mode == 4) alarms[0].m = (alarms[0].m + 1u) % 60u;
                else if (mode == 5) alarms[1].h = (alarms[1].h + 1u) % 24u;
                else if (mode == 6) alarms[1].m = (alarms[1].m + 1u) % 60u;
            }
            if (s3)
            {
                if      (mode == 1) h           = (h           == 0u) ? 23u : h           - 1u;
                else if (mode == 2) m           = (m           == 0u) ? 59u : m           - 1u;
                else if (mode == 3) alarms[0].h = (alarms[0].h == 0u) ? 23u : alarms[0].h - 1u;
                else if (mode == 4) alarms[0].m = (alarms[0].m == 0u) ? 59u : alarms[0].m - 1u;
                else if (mode == 5) alarms[1].h = (alarms[1].h == 0u) ? 23u : alarms[1].h - 1u;
                else if (mode == 6) alarms[1].m = (alarms[1].m == 0u) ? 59u : alarms[1].m - 1u;
            }
        }

        /* ================================================
         * 4. Odczyt Serial (komendy daty/czasu)
         *    Czytamy znak po znaku nieblokująco.
         * ================================================ */
        while (Serial.available() > 0)
        {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r')
            {
                if (serialIdx > 0)
                {
                    serialBuf[serialIdx] = '\0';
                    parseSerial(serialBuf, &year, &month, &day,
                                &h, &m, &s, alarms, &isDST);
                    serialIdx = 0;
                }
            }
            else if (serialIdx < (uint8_t)(sizeof(serialBuf) - 1u))
            {
                serialBuf[serialIdx++] = c;
            }
        }

        /* ================================================
         * 5. Inkrementacja czasu co 1000 ms
         * ================================================ */
        if ((xNow - xLastSec) >= pdMS_TO_TICKS(1000))
        {
            xLastSec += pdMS_TO_TICKS(1000);

            s++;
            if (s >= 60u)
            {
                s = 0u; m++;
                if (m >= 60u)
                {
                    m = 0u; h++;
                    if (h >= 24u)
                    {
                        h = 0u;
                        advanceDay(&year, &month, &day);

                        /* Nowy rok – reset flag DST */
                        if (month == 1u && day == 1u)
                        {
                            dstDoneSpring = false;
                            dstDoneFall   = false;
                        }
                    }

                    /* ---- DST: sprawdzamy tylko przy zmianie godziny ---- */
                    /* Zmiana na czas letni: ostatnia niedziela marca, 2:00 → 3:00 */
                    if (!isDST && !dstDoneSpring &&
                        month == 3u && day == lastSunday(year, 3u) && h == 2u)
                    {
                        h = 3u;
                        isDST = true;
                        dstDoneSpring = true;
                        Serial.println(F("Zmiana na czas letni (CEST +2)."));
                    }
                    /* Zmiana na czas zimowy: ostatnia niedziela października, 3:00 → 2:00 */
                    if (isDST && !dstDoneFall &&
                        month == 10u && day == lastSunday(year, 10u) && h == 3u)
                    {
                        h = 2u;
                        isDST = false;
                        dstDoneFall = true;
                        Serial.println(F("Zmiana na czas zimowy (CET +1)."));
                    }
                }
            }

            /* Drzemka alarmów */
            for (uint8_t i = 0; i < 2u; i++)
            {
                if (alarms[i].snooze)
                {
                    alarms[i].snoozeLeft--;
                    if (alarms[i].snoozeLeft == 0u)
                    {
                        alarms[i].snooze   = false;
                        alarms[i].ringing  = true;
                        Serial.print(F("ALARM ")); Serial.print(i + 1);
                        Serial.println(F(" (po drzemce)!"));
                    }
                }
            }

            /* Sprawdzenie alarmów */
            for (uint8_t i = 0; i < 2u; i++)
            {
                if (alarms[i].enabled && !alarms[i].ringing && !alarms[i].snooze &&
                    h == alarms[i].h && m == alarms[i].m && s == 0u)
                {
                    alarms[i].ringing = true;
                    Serial.print(F("ALARM ")); Serial.print(i + 1); Serial.print(F("! "));
                    Serial.print(alarms[i].h); Serial.print(':');
                    if (alarms[i].m < 10u) Serial.print('0');
                    Serial.println(alarms[i].m);
                }
            }

            /* Aktualizacja fazy wyświetlacza co sekundę */
            phaseSecs++;
            if (phaseSecs >= phaseLimit[dispPhase])
            {
                phaseSecs = 0u;
                dispPhase = (dispPhase + 1u) % 3u;
            }
        }

        /* ================================================
         * 6. Budowanie wiadomości wyświetlacza
         * ================================================ */
        bool dot = ((xNow % pdMS_TO_TICKS(1000)) < pdMS_TO_TICKS(500));
        anyRinging = alarms[0].ringing || alarms[1].ringing;

        DisplayMsg_t msg;
        msg.buzzer = anyRinging;

        if (mode != 0)
        {
            /* Tryby ustawiania */
            switch (mode)
            {
                case 1:
                    msg.d[0] = dot ? (h / 10u) : BLANK;
                    msg.d[1] = dot ? (h % 10u) : BLANK;
                    msg.d[2] = m / 10u; msg.d[3] = m % 10u;
                    msg.dot  = false;
                    break;
                case 2:
                    msg.d[0] = h / 10u; msg.d[1] = h % 10u;
                    msg.d[2] = dot ? (m / 10u) : BLANK;
                    msg.d[3] = dot ? (m % 10u) : BLANK;
                    msg.dot  = false;
                    break;
                case 3:
                    msg.d[0] = dot ? (alarms[0].h / 10u) : BLANK;
                    msg.d[1] = dot ? (alarms[0].h % 10u) : BLANK;
                    msg.d[2] = alarms[0].m / 10u; msg.d[3] = alarms[0].m % 10u;
                    msg.dot  = false;
                    break;
                case 4:
                    msg.d[0] = alarms[0].h / 10u; msg.d[1] = alarms[0].h % 10u;
                    msg.d[2] = dot ? (alarms[0].m / 10u) : BLANK;
                    msg.d[3] = dot ? (alarms[0].m % 10u) : BLANK;
                    msg.dot  = false;
                    break;
                case 5:
                    msg.d[0] = dot ? (alarms[1].h / 10u) : BLANK;
                    msg.d[1] = dot ? (alarms[1].h % 10u) : BLANK;
                    msg.d[2] = alarms[1].m / 10u; msg.d[3] = alarms[1].m % 10u;
                    msg.dot  = false;
                    break;
                case 6:
                    msg.d[0] = alarms[1].h / 10u; msg.d[1] = alarms[1].h % 10u;
                    msg.d[2] = dot ? (alarms[1].m / 10u) : BLANK;
                    msg.d[3] = dot ? (alarms[1].m % 10u) : BLANK;
                    msg.dot  = false;
                    break;
                default:
                    msg.d[0] = msg.d[1] = msg.d[2] = msg.d[3] = BLANK;
                    msg.dot = false;
                    break;
            }
        }
        else
        {
            /* Tryb 0 – rotacja wyświetlacza */
            switch (dispPhase)
            {
                case 0: /* HH.MM – czas */
                    msg.d[0] = h / 10u;
                    msg.d[1] = h % 10u;
                    msg.d[2] = m / 10u;
                    msg.d[3] = m % 10u;
                    msg.dot  = dot; /* pulsuje co 500 ms */
                    break;

                case 1: /* RRRR – rok */
                    msg.d[0] = (uint8_t)((year / 1000u) % 10u);
                    msg.d[1] = (uint8_t)((year / 100u)  % 10u);
                    msg.d[2] = (uint8_t)((year / 10u)   % 10u);
                    msg.d[3] = (uint8_t)( year           % 10u);
                    msg.dot  = false;
                    break;

                case 2: /* MM.DD – miesiąc i dzień */
                    msg.d[0] = month / 10u;
                    msg.d[1] = month % 10u;
                    msg.d[2] = day   / 10u;
                    msg.d[3] = day   % 10u;
                    msg.dot  = true; /* stała kropka MM.DD */
                    break;

                default:
                    msg.d[0] = msg.d[1] = msg.d[2] = msg.d[3] = BLANK;
                    msg.dot = false;
                    break;
            }
        }

        xQueueOverwrite(xDisplayQueue, &msg);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(50));
    }
}

/* ============================================================
 * FUNKCJE POMOCNICZE KALENDARZA
 * ============================================================ */

/**
 * isLeap – zwraca true jeśli rok przestępny.
 * Reguła: podzielny przez 4, ale nie przez 100,
 *         chyba że podzielny przez 400.
 */
static bool isLeap(uint16_t y)
{
    return ((y % 4u == 0u) && (y % 100u != 0u)) || (y % 400u == 0u);
}

/**
 * daysInMonth – liczba dni w danym miesiącu uwzględniając lata przestępne.
 */
static uint8_t daysInMonth(uint16_t y, uint8_t mo)
{
    static const uint8_t days[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    if (mo == 2u && isLeap(y)) return 29u;
    return days[mo];
}

/**
 * dayOfWeek – dzień tygodnia (0=Niedziela, 1=Poniedziałek, …, 6=Sobota)
 * Algorytm Tomohiko Sakamoto.
 */
static uint8_t dayOfWeek(uint16_t y, uint8_t mo, uint8_t d)
{
    static const uint8_t t[] = { 0u, 3u, 2u, 5u, 0u, 3u, 5u, 1u, 4u, 6u, 2u, 4u };
    uint16_t yr = y;
    if (mo < 3u) yr--;
    return (uint8_t)((yr + yr/4u - yr/100u + yr/400u + t[mo - 1u] + d) % 7u);
}

/**
 * lastSunday – ostatnia niedziela danego miesiąca (numer dnia 1–31).
 */
static uint8_t lastSunday(uint16_t y, uint8_t mo)
{
    uint8_t last = daysInMonth(y, mo);
    uint8_t dow  = dayOfWeek(y, mo, last);
    /* dow=0 → niedziela; odejmij tyle co dow aby dojść do niedzieli */
    return last - dow;
}

/**
 * advanceDay – przesuwa datę o jeden dzień do przodu.
 */
static void advanceDay(uint16_t *y, uint8_t *mo, uint8_t *d)
{
    (*d)++;
    if (*d > daysInMonth(*y, *mo))
    {
        *d = 1u;
        (*mo)++;
        if (*mo > 12u)
        {
            *mo = 1u;
            (*y)++;
        }
    }
}

/**
 * parseSerial – parsowanie komendy z Serial Monitor.
 *
 * Obsługiwane komendy:
 *   DATE RRRR MM DD   – ustawia datę
 *   TIME GG MM SS     – ustawia czas
 *   ALARM1 GG MM      – ustawia alarm 1
 *   ALARM2 GG MM      – ustawia alarm 2
 *
 * Parser ręczny (bez sscanf/atoi) – oszczędność stosu na AVR.
 */
static uint16_t parseNum(const char **p)
{
    while (**p == ' ') (*p)++; /* pomiń spacje */
    uint16_t val = 0u;
    while (**p >= '0' && **p <= '9')
    {
        val = val * 10u + (uint16_t)(**p - '0');
        (*p)++;
    }
    return val;
}

static void parseSerial(const char *cmd,
                         uint16_t *y,  uint8_t *mo, uint8_t *dy,
                         uint8_t  *h,  uint8_t *m,  uint8_t *s,
                         Alarm_t  alarms[], bool *isDST)
{
    (void)isDST; /* nieużywane w parserze, dostępne dla rozszerzenia */

    if (strncmp(cmd, "DATE ", 5) == 0)
    {
        const char *p = cmd + 5;
        uint16_t ny  = parseNum(&p);
        uint8_t  nmo = (uint8_t)parseNum(&p);
        uint8_t  nd  = (uint8_t)parseNum(&p);
        if (ny >= 2000u && ny <= 2099u && nmo >= 1u && nmo <= 12u &&
            nd >= 1u && nd <= daysInMonth(ny, nmo))
        {
            *y = ny; *mo = nmo; *dy = nd;
            Serial.print(F("Data: "));
            Serial.print(ny); Serial.print('/');
            if (nmo < 10u) Serial.print('0'); Serial.print(nmo); Serial.print('/');
            if (nd  < 10u) Serial.print('0'); Serial.println(nd);
        }
        else { Serial.println(F("ERR: nieprawidlowa data")); }
    }
    else if (strncmp(cmd, "TIME ", 5) == 0)
    {
        const char *p = cmd + 5;
        uint8_t nh = (uint8_t)parseNum(&p);
        uint8_t nm = (uint8_t)parseNum(&p);
        uint8_t ns = (uint8_t)parseNum(&p);
        if (nh < 24u && nm < 60u && ns < 60u)
        {
            *h = nh; *m = nm; *s = ns;
            Serial.print(F("Czas: "));
            if (nh < 10u) Serial.print('0'); Serial.print(nh); Serial.print(':');
            if (nm < 10u) Serial.print('0'); Serial.print(nm); Serial.print(':');
            if (ns < 10u) Serial.print('0'); Serial.println(ns);
        }
        else { Serial.println(F("ERR: nieprawidlowy czas")); }
    }
    else if (strncmp(cmd, "ALARM1 ", 7) == 0)
    {
        const char *p = cmd + 7;
        uint8_t ah = (uint8_t)parseNum(&p);
        uint8_t am = (uint8_t)parseNum(&p);
        if (ah < 24u && am < 60u)
        {
            alarms[0].h = ah; alarms[0].m = am; alarms[0].enabled = true;
            Serial.print(F("Alarm1: "));
            if (ah < 10u) Serial.print('0'); Serial.print(ah); Serial.print(':');
            if (am < 10u) Serial.print('0'); Serial.println(am);
        }
        else { Serial.println(F("ERR: nieprawidlowy alarm")); }
    }
    else if (strncmp(cmd, "ALARM2 ", 7) == 0)
    {
        const char *p = cmd + 7;
        uint8_t ah = (uint8_t)parseNum(&p);
        uint8_t am = (uint8_t)parseNum(&p);
        if (ah < 24u && am < 60u)
        {
            alarms[1].h = ah; alarms[1].m = am; alarms[1].enabled = true;
            Serial.print(F("Alarm2: "));
            if (ah < 10u) Serial.print('0'); Serial.print(ah); Serial.print(':');
            if (am < 10u) Serial.print('0'); Serial.println(am);
        }
        else { Serial.println(F("ERR: nieprawidlowy alarm")); }
    }
    else
    {
        Serial.println(F("Nieznana komenda. Uzyj: DATE RRRR MM DD | TIME GG MM SS | ALARM1 GG MM | ALARM2 GG MM"));
    }
}
