/*
 * ============================================================
 * Zadanie 2 – Zegar z alarmem i drzemką
 * Systemy Czasu Rzeczywistego – Laboratorium 5
 * Sprzęt: Arduino Uno + MultiFuncShield
 * Biblioteka: Arduino_FreeRTOS (https://github.com/feilipu/Arduino_FreeRTOS_Library)
 * ============================================================
 *
 * ARCHITEKTURA WĄTKÓW:
 *   vDisplayTask  (prio 3) – przemiatanie wyświetlacza 7-seg. (multipleksing)
 *   vButtonTask   (prio 2) – detekcja wciśnięć S1/S2/S3, wysyła semafory
 *   vClockTask    (prio 1) – logika: czas, alarm, drzemka; wysyła dane do kolejki
 *
 * KOMUNIKACJA:
 *   xDisplayQueue  – kolejka strukturalna (głębokość 1, xQueueOverwrite)
 *                    przekazuje dane wyświetlacza z vClockTask → vDisplayTask
 *   xSem_S1/S2/S3  – semafory binarne: wciśnięcie przycisku → vClockTask
 *
 * OBSŁUGA PRZYCISKÓW:
 *   Tryb normalny:
 *     S1 (raz)    → ustawianie godzin (godziny migają)
 *     S1 (x2)    → ustawianie minut
 *     S1 (x3)    → ustawianie godzin alarmu
 *     S1 (x4)    → ustawianie minut alarmu
 *     S1 (x5)    → powrót do wyświetlania
 *     S2          → wartość +1
 *     S3          → wartość -1
 *   Alarm aktywny:
 *     S1          → wyłącz alarm
 *     S2          → drzemka 20 sekund
 *
 * WYMAGANIA DOTYCZĄCE TICK RATE:
 *   Domyślny tick FreeRTOS na Arduino = 15 ms (configTICK_RATE_HZ ≈ 67).
 *   Dla płynnego przemiatania wyświetlacza zalecane jest ustawienie
 *   configTICK_RATE_HZ = 1000 w FreeRTOSConfig.h (tick = 1 ms).
 * ============================================================
 */

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>

/* ============================================================
 * PINY MULTIFUNCSHIELD
 * ============================================================ */
#define PIN_LATCH   4    // ST_CP rejestru przesuwnego 74HC595
#define PIN_CLK     7    // SH_CP rejestru przesuwnego 74HC595
#define PIN_DATA    8    // DS    rejestru przesuwnego 74HC595
#define PIN_DIG1    6    // Anoda cyfry 1 (aktywna LOW)
#define PIN_DIG2    11   // Anoda cyfry 2
#define PIN_DIG3    10   // Anoda cyfry 3
#define PIN_DIG4    9    // Anoda cyfry 4
#define PIN_BTN1    A1   // Przycisk S1 (aktywny LOW, wewnętrzny pull-up)
#define PIN_BTN2    A2   // Przycisk S2
#define PIN_BTN3    A3   // Przycisk S3
#define PIN_BUZZER  3    // Buzzer (aktywny HIGH przez tranzystor)

/* ============================================================
 * MAPA SEGMENTÓW – wyświetlacz common-anode, logika ujemna (active LOW)
 * Kolejność bitów w bajcie: DP g f e d c b a (MSB→LSB)
 * Bit = 0 → segment świeci, bit = 1 → segment zgaszony
 * ============================================================ */
static const uint8_t SEG_MAP[11] = {
    0xC0, // 0  → 1100 0000
    0xF9, // 1  → 1111 1001
    0xA4, // 2  → 1010 0100
    0xB0, // 3  → 1011 0000
    0x99, // 4  → 1001 1001
    0x92, // 5  → 1001 0010
    0x82, // 6  → 1000 0010
    0xF8, // 7  → 1111 1000
    0x80, // 8  → 1000 0000
    0x90, // 9  → 1001 0000
    0xFF, // 10 → BLANK (wszystkie segmenty zgaszone)
};
#define BLANK     10       // indeks "cyfry" dla pustego miejsca
#define DOT_MASK  0x7Fu    // AND z bajtem segmentu → włącza kropkę (DP = bit 7 = 0)

/* ============================================================
 * STRUKTURA DANYCH PRZEKAZYWANA PRZEZ KOLEJKĘ
 * ============================================================ */
typedef struct {
    uint8_t d[4];   // cyfry 0–9 lub BLANK dla każdej z 4 pozycji
    bool    dot;    // true → kropka aktywna na pozycji d[1] (HH.MM)
    bool    buzzer; // true → buzzer włączony
} DisplayMsg_t;

/* ============================================================
 * UCHWYTY FREERTOS
 * ============================================================ */
static QueueHandle_t     xDisplayQueue; // kolejka głębokości 1
static SemaphoreHandle_t xSem_S1;
static SemaphoreHandle_t xSem_S2;
static SemaphoreHandle_t xSem_S3;

/* ============================================================
 * DEKLARACJE ZADAŃ
 * ============================================================ */
static void vDisplayTask(void *pvParameters);
static void vButtonTask (void *pvParameters);
static void vClockTask  (void *pvParameters);

/* ============================================================
 * SETUP
 * ============================================================ */
void setup()
{
    Serial.begin(9600);

    /* Konfiguracja pinów wyjściowych */
    pinMode(PIN_LATCH,  OUTPUT);
    pinMode(PIN_CLK,    OUTPUT);
    pinMode(PIN_DATA,   OUTPUT);
    pinMode(PIN_DIG1,   OUTPUT);
    pinMode(PIN_DIG2,   OUTPUT);
    pinMode(PIN_DIG3,   OUTPUT);
    pinMode(PIN_DIG4,   OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);

    /* Konfiguracja pinów wejściowych z wewnętrznym pull-up */
    pinMode(PIN_BTN1, INPUT_PULLUP);
    pinMode(PIN_BTN2, INPUT_PULLUP);
    pinMode(PIN_BTN3, INPUT_PULLUP);

    /* Stan początkowy: wszystkie cyfry wyłączone, buzzer cichy */
    digitalWrite(PIN_DIG1,   HIGH);
    digitalWrite(PIN_DIG2,   HIGH);
    digitalWrite(PIN_DIG3,   HIGH);
    digitalWrite(PIN_DIG4,   HIGH);
    digitalWrite(PIN_BUZZER, LOW);

    /* Tworzenie zasobów FreeRTOS */
    xDisplayQueue = xQueueCreate(1, sizeof(DisplayMsg_t)); // głębokość 1 → xQueueOverwrite
    xSem_S1       = xSemaphoreCreateBinary();
    xSem_S2       = xSemaphoreCreateBinary();
    xSem_S3       = xSemaphoreCreateBinary();

    /* Tworzenie wątków
     * Stack 128 słów (256 B) dla Display i Button, 256 słów (512 B) dla Clock */
    xTaskCreate(vDisplayTask, "Disp",  128, NULL, 3, NULL);
    xTaskCreate(vButtonTask,  "Btn",   128, NULL, 2, NULL);
    xTaskCreate(vClockTask,   "Clock", 256, NULL, 1, NULL);

    vTaskStartScheduler(); // FreeRTOS przejmuje kontrolę; loop() nie jest wywoływany
}

void loop() { /* celowo puste – FreeRTOS */ }

/* ============================================================
 * FUNKCJA POMOCNICZA: wyślij bajt do 74HC595 i aktywuj jedną cyfrę
 * Wywołana z vDisplayTask, krótki sekcja krytyczna na poziomie I/O.
 * ============================================================ */
static void writeToDisplay(uint8_t digitPin, uint8_t segByte)
{
    /* 1. Wyłącz wszystkie cyfry – zapobiega "ghostingowi" (duchy) */
    digitalWrite(PIN_DIG1, HIGH);
    digitalWrite(PIN_DIG2, HIGH);
    digitalWrite(PIN_DIG3, HIGH);
    digitalWrite(PIN_DIG4, HIGH);

    /* 2. Załaduj bajt segmentów do rejestru przesuwnego */
    digitalWrite(PIN_LATCH, LOW);
    shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, segByte);
    digitalWrite(PIN_LATCH, HIGH);

    /* 3. Aktywuj wybraną cyfrę (LOW = anoda podłączona do VCC przez tranzystor) */
    digitalWrite(digitPin, LOW);
}

/* ============================================================
 * WĄTEK 1: vDisplayTask – przemiatanie (multipleksing) wyświetlacza
 *
 * Priorytet 3 (najwyższy): wymaga regularnego dostępu do CPU,
 * żeby uniknąć migotania wyświetlacza.
 *
 * Działanie:
 *   - Co 1 tyk (domyślnie 15 ms, z tick=1ms → 5 ms) przechodzi na
 *     następną cyfrę (0→1→2→3→0…).
 *   - Odczytuje wiadomość z kolejki (nieblokująco); jeśli brak – używa
 *     poprzednich danych.
 *   - Wyświetla segment bieżącej cyfry; dla pozycji 1 (HH.MM) dodaje
 *     kropkę jeśli msg.dot == true.
 *   - Steruje buzzerem na podstawie msg.buzzer.
 * ============================================================ */
static void vDisplayTask(void *pvParameters)
{
    static const uint8_t digitPins[4] = { PIN_DIG1, PIN_DIG2, PIN_DIG3, PIN_DIG4 };

    DisplayMsg_t msg;
    msg.d[0] = BLANK; msg.d[1] = BLANK; msg.d[2] = BLANK; msg.d[3] = BLANK;
    msg.dot    = false;
    msg.buzzer = false;

    uint8_t pos = 0; // aktualnie wyświetlana cyfra (0–3)

    for (;;)
    {
        /* Odczyt nowej wiadomości (timeout=0 → nieblokujące) */
        DisplayMsg_t tmp;
        if (xQueueReceive(xDisplayQueue, &tmp, 0) == pdPASS)
        {
            msg = tmp;
        }

        /* Sterowanie buzzerem */
        digitalWrite(PIN_BUZZER, msg.buzzer ? HIGH : LOW);

        /* Oblicz bajt segmentów dla bieżącej cyfry */
        uint8_t segByte = SEG_MAP[msg.d[pos]];

        /* Dodaj kropkę na pozycji 1 (separacja HH|MM) jeśli wymagana */
        if (msg.dot && (pos == 1))
        {
            segByte &= DOT_MASK; // bit 7 = 0 → DP świeci
        }

        writeToDisplay(digitPins[pos], segByte);

        /* Przejdź do następnej cyfry */
        pos = (pos + 1u) % 4u;

        /*
         * Opóźnienie 1 tyk na cyfrę.
         * Przy tick=15 ms: cykl = 4×15 = 60 ms ≈ 17 Hz (może lekko migotać).
         * Przy tick= 1 ms i vTaskDelay(5): cykl = 4×5 = 20 ms ≈ 50 Hz (płynny obraz).
         * Aby uzyskać tick=1ms: w FreeRTOSConfig.h ustaw configTICK_RATE_HZ = 1000.
         */
        vTaskDelay(1);
    }
}

/* ============================================================
 * WĄTEK 2: vButtonTask – obsługa przycisków
 *
 * Priorytet 2: odpytuje przyciski co 50 ms (debouncowanie),
 * wykrywa zbocze opadające (wciśnięcie) i daje semafor.
 * ============================================================ */
static void vButtonTask(void *pvParameters)
{
    static const uint8_t      btnPins[3] = { PIN_BTN1, PIN_BTN2, PIN_BTN3 };
    static SemaphoreHandle_t  sems[3];
    sems[0] = xSem_S1; sems[1] = xSem_S2; sems[2] = xSem_S3;

    bool lastState[3] = { true, true, true }; // HIGH = nieprzytykniete (pull-up)

    for (;;)
    {
        for (uint8_t i = 0; i < 3u; i++)
        {
            bool cur = (bool)digitalRead(btnPins[i]);
            if (!cur && lastState[i]) // zbocze opadające → wciśnięcie
            {
                xSemaphoreGive(sems[i]);
            }
            lastState[i] = cur;
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 50 ms debouncowanie
    }
}

/* ============================================================
 * WĄTEK 3: vClockTask – logika zegara, alarmu i drzemki
 *
 * Priorytet 1 (najniższy): logika aplikacji, nie potrzebuje
 * sztywnej częstotliwości – działa co 50 ms.
 *
 * TRYBY (mode):
 *   0 – wyświetlanie czasu (normalny)
 *   1 – ustawianie godzin  (godziny migają)
 *   2 – ustawianie minut   (minuty migają)
 *   3 – ustawianie godzin alarmu
 *   4 – ustawianie minut alarmu
 *
 * ZMIENNE PRYWATNE (bez zmiennych globalnych – dane tylko w kolejce):
 *   h, m, s          – aktualny czas
 *   alH, alM         – czas alarmu
 *   alEnabled        – alarm aktywowany
 *   alRinging        – alarm dzwoni
 *   snooze/snoozeLeft – drzemka aktywna + pozostałe sekundy
 * ============================================================ */
static void vClockTask(void *pvParameters)
{
    /* ---- czas bieżący ---- */
    uint8_t h = 12, m = 0, s = 0;

    /* ---- alarm ---- */
    uint8_t alH = 7, alM = 0;
    bool    alEnabled  = false;
    bool    alRinging  = false;

    /* ---- drzemka ---- */
    bool    snooze     = false;
    uint8_t snoozeLeft = 0;

    /* ---- tryb pracy ---- */
    uint8_t mode = 0;

    TickType_t xLastSec  = xTaskGetTickCount(); // moment ostatniej inkrementacji sekundy
    TickType_t xLastWake = xTaskGetTickCount(); // dla vTaskDelayUntil

    for (;;)
    {
        TickType_t xNow = xTaskGetTickCount();

        /* ================================================
         * 1. Odczyt semaforów przycisków (timeout=0 → nieblokujące)
         * ================================================ */
        bool s1 = (xSemaphoreTake(xSem_S1, 0) == pdPASS);
        bool s2 = (xSemaphoreTake(xSem_S2, 0) == pdPASS);
        bool s3 = (xSemaphoreTake(xSem_S3, 0) == pdPASS);

        /* ================================================
         * 2. Obsługa przycisków gdy alarm dzwoni
         *    S1 = wyłącz  |  S2 = drzemka 20 s
         * ================================================ */
        if (alRinging)
        {
            if (s1)
            {
                alRinging = false;
                Serial.println(F("Alarm wylaczony."));
            }
            else if (s2)
            {
                alRinging  = false;
                snooze     = true;
                snoozeLeft = 20;
                Serial.println(F("Drzema: 20 sekund..."));
            }
        }
        else
        {
            /* ================================================
             * 3. Obsługa przycisków w trybach ustawiania
             *    S1 = zmiana trybu | S2 = +1 | S3 = -1
             * ================================================ */
            if (s1)
            {
                mode = (mode + 1u) % 5u;
                if (mode == 0) Serial.println(F("Czas/alarm zapisane."));
                if (mode == 3) alEnabled = true; // włącz alarm przy wejściu w jego ustawianie
            }
            if (s2)
            {
                if      (mode == 1) h   = (h   + 1u) % 24u;
                else if (mode == 2) m   = (m   + 1u) % 60u;
                else if (mode == 3) alH = (alH + 1u) % 24u;
                else if (mode == 4) alM = (alM + 1u) % 60u;
            }
            if (s3)
            {
                if      (mode == 1) h   = (h   == 0u) ? 23u : h   - 1u;
                else if (mode == 2) m   = (m   == 0u) ? 59u : m   - 1u;
                else if (mode == 3) alH = (alH == 0u) ? 23u : alH - 1u;
                else if (mode == 4) alM = (alM == 0u) ? 59u : alM - 1u;
            }
        }

        /* ================================================
         * 4. Inkrementacja czasu co 1000 ms
         *    Używamy różnicy tyknięć; xLastSec += 1000ms
         *    zapobiega dryfowi (nie przypisujemy xNow).
         * ================================================ */
        if ((xNow - xLastSec) >= pdMS_TO_TICKS(1000))
        {
            xLastSec += pdMS_TO_TICKS(1000);

            s++;
            if (s >= 60u) { s = 0u; m++; }
            if (m >= 60u) { m = 0u; h++; }
            if (h >= 24u) { h = 0u; }

            /* Odliczanie drzemki */
            if (snooze)
            {
                snoozeLeft--;
                if (snoozeLeft == 0u)
                {
                    snooze    = false;
                    alRinging = true;
                    Serial.println(F("ALARM! (po drzemica)"));
                }
            }

            /* Sprawdzenie alarmu (tylko raz na minutę, przy s==0) */
            if (alEnabled && !alRinging && !snooze && (h == alH) && (m == alM) && (s == 0u))
            {
                alRinging = true;
                Serial.print(F("ALARM! "));
                Serial.print(alH); Serial.print(':');
                if (alM < 10) Serial.print('0');
                Serial.println(alM);
            }
        }

        /* ================================================
         * 5. Budowanie wiadomości do kolejki wyświetlacza
         *    Kropka pulsuje: 500 ms świeci / 500 ms nie świeci
         * ================================================ */
        bool dot = ((xNow % pdMS_TO_TICKS(1000)) < pdMS_TO_TICKS(500));

        DisplayMsg_t msg;
        msg.buzzer = alRinging;

        switch (mode)
        {
            case 0: // normalny czas: HH.MM
                msg.d[0] = h  / 10u;
                msg.d[1] = h  % 10u;
                msg.d[2] = m  / 10u;
                msg.d[3] = m  % 10u;
                msg.dot  = dot;
                break;

            case 1: // ustawianie godzin – godziny migają
                msg.d[0] = dot ? (h / 10u) : BLANK;
                msg.d[1] = dot ? (h % 10u) : BLANK;
                msg.d[2] = m  / 10u;
                msg.d[3] = m  % 10u;
                msg.dot  = false;
                break;

            case 2: // ustawianie minut – minuty migają
                msg.d[0] = h  / 10u;
                msg.d[1] = h  % 10u;
                msg.d[2] = dot ? (m / 10u) : BLANK;
                msg.d[3] = dot ? (m % 10u) : BLANK;
                msg.dot  = false;
                break;

            case 3: // ustawianie godzin alarmu – godziny alarmu migają
                msg.d[0] = dot ? (alH / 10u) : BLANK;
                msg.d[1] = dot ? (alH % 10u) : BLANK;
                msg.d[2] = alM / 10u;
                msg.d[3] = alM % 10u;
                msg.dot  = false;
                break;

            case 4: // ustawianie minut alarmu – minuty alarmu migają
                msg.d[0] = alH / 10u;
                msg.d[1] = alH % 10u;
                msg.d[2] = dot ? (alM / 10u) : BLANK;
                msg.d[3] = dot ? (alM % 10u) : BLANK;
                msg.dot  = false;
                break;

            default:
                msg.d[0] = BLANK; msg.d[1] = BLANK;
                msg.d[2] = BLANK; msg.d[3] = BLANK;
                msg.dot  = false;
                break;
        }

        /* Wyślij dane do kolejki (nadpisz jeśli kolejka pełna – zawsze aktualne dane) */
        xQueueOverwrite(xDisplayQueue, &msg);

        /* ================================================
         * 6. Stałe opóźnienie 50 ms (polling rate przycisków i UI)
         *    vTaskDelayUntil zapewnia brak dryfu pętli.
         * ================================================ */
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(50));
    }
}
