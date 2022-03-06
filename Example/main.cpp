/**
 * @file main.cpp
 * @author Evandro Teixeira
 * @brief 
 * @version 0.1
 * @date 26-02-2022
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <Arduino.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <freertos/task.h>
#include "buzzer.hpp"

#define COLOR_BLACK         "\e[0;30m"
#define COLOR_RED           "\e[0;31m"
#define COLOR_GREEN         "\e[0;32m"
#define COLOR_YELLOW        "\e[0;33m"
#define COLOR_BLUE          "\e[0;34m"
#define COLOR_PURPLE        "\e[0;35m"
#define COLOR_CYAN          "\e[0;36m"
#define COLOR_WRITE         "\e[0;37m"
#define COLOR_RESET         "\e[0m"
#define BUZZER_FREQUENCY    30                  // Frequency 2000 Hz
#define BUZZER_DUTYCYCLE    75                  // Init DutyCycle 50%
#define BUZZER_CONFIG       (BuzzerPin_t){15,4} // Pin, Channel PWM
#define PIN_BUTTON          14                  // GPIO14 
#define LED_BOARD           2                   // Pino do LED
#define DEBOUNCE_BUTTON     50                  // Tempo do debounce do botão
#define SW_TIMER_PERIOD_PULSO    10             // 10 ms
#define SW_TIMER_PERIOD_BUZZER   500            // 500 ms
#define NUMBER_OF_ELEMENTS       8              // Número de elementos na fila

/**
 * @brief Cria objeto Buzzer
 * @return buzzer 
 */
buzzer Buzzer(BUZZER_CONFIG);

/**
 * @brief 
 */
SemaphoreHandle_t xSemaphore_BuzzerOff = NULL;
SemaphoreHandle_t xMutex_PulseTimerCounter = NULL;
TimerHandle_t TimerPulso;
TimerHandle_t TimerBuzzer;
QueueHandle_t QueuePulso;
uint32_t PulseTimerCounter;  


/**
 * @brief 
 * @param parameters 
 */
void Tarefa_Pulso(void *parameters);
void Tarefa_Buzzer(void *parameters);

uint32_t PulseTimerCounter_Get(void);
void PulseTimerCounter_Clear(void);
void PulseTimerCounter_Increment(void);

void Callback_TimerPulso(TimerHandle_t timer);
void Callback_TimerBuzzer(TimerHandle_t timer);

void setup() 
{
  // Inicializa a Serial 
  Serial.begin( 115200 );
  Serial.printf("\n\rFreeRTOS - Software Timer\n\r");

  // Inicializa o Buzzer 
  Buzzer.begin(BUZZER_FREQUENCY,BUZZER_DUTYCYCLE);
  Buzzer.stop();

  // Inicializa pino 14 como entrada
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // Inicializa pino do LED on Board
  pinMode(LED_BOARD,OUTPUT);
  digitalWrite(LED_BOARD,LOW);

  // Cria semafaro binario xSemaphore_Pulso
  vSemaphoreCreateBinary( xSemaphore_BuzzerOff );
  if(xSemaphore_BuzzerOff == NULL)
  {
    Serial.printf("\n\rFalha em criar o semafaro xSemaphore_BuzzerOff");
  }

  // Cria Mutex xMutex_PulseTimerCounter
  xMutex_PulseTimerCounter = xSemaphoreCreateMutex();
  if(xMutex_PulseTimerCounter == NULL)
  {
    Serial.printf("\n\rFalha em criar o Mutex para variavel global xMutex_PulseTimerCounter");
  }

  // Cria SoftwareTimer  TimerPulso
  TimerPulso = xTimerCreate("TIMER_PULSO",pdMS_TO_TICKS(SW_TIMER_PERIOD_PULSO),pdTRUE,NULL,Callback_TimerPulso);
  if(TimerPulso == NULL)
  {
    Serial.printf("\n\rFalha em criar SW_Timer TimerPulso");
  }

  // Cria SoftwareTimer  TimerBuzzer
  TimerBuzzer = xTimerCreate("TIMER_PULSO",pdMS_TO_TICKS(SW_TIMER_PERIOD_BUZZER),pdFALSE,NULL,Callback_TimerBuzzer);
  if(TimerBuzzer == NULL)
  {
    Serial.printf("\n\rFalha em criar SW_Timer TimerBuzzer");
  }

  // Cria Fila de mensagem QueuePulso
  QueuePulso = xQueueCreate( NUMBER_OF_ELEMENTS, sizeof(uint32_t) );
  if(QueuePulso == NULL)
  {
    Serial.printf("\n\rFalha em criar a fila QueuePulso");
  }

  // Cria tarefas da aplicação
  xTaskCreate(Tarefa_Pulso, "PULSO", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(Tarefa_Buzzer, "BUZZER", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
}

void loop() 
{
  Serial.printf("\n\rSupende tarefa LOOP");
  vTaskSuspend(NULL);
}

/**
 * @brief 
 * @param parameters 
 */
void Tarefa_Pulso(void *parameters)
{
  static int valueOld = 0xFF;
  int value = 0;
  uint32_t valuePulso;
  
  while (1)
  {
    // le o valor do botão 
    value = digitalRead(PIN_BUTTON);

    // Detecta borda de descida
    if((value != valueOld) && (value == LOW))
    {
      // Zera contador do tempo do pulso
      PulseTimerCounter_Clear();
      // Inicia TimerPulso
      xTimerStart(TimerPulso, 0 );
      // Aciona LED 
      digitalWrite(LED_BOARD,HIGH);
      Serial.print(COLOR_GREEN);
      Serial.printf("\n\rDetecta borda de descida - Aciona LED on Board");
      Serial.print(COLOR_RESET);
    }
    else 
    {
      // Detecta borda de subida 
      if((value != valueOld) && (value == HIGH))
      {
        // Encera o TimerPulso
        xTimerStop(TimerPulso, 0);
        // Pega o tempo do pulso
        valuePulso = PulseTimerCounter_Get();
        if(valuePulso != 0) 
        {
          // Publica na Fila de MSG o tempo do Pulso 
          if(xQueueSend(QueuePulso, &valuePulso, pdMS_TO_TICKS(10) ) != pdTRUE)
          {
            Serial.print(COLOR_GREEN);
            Serial.printf("\n\rFalha em enviar os dados da fila");
            Serial.print(COLOR_RESET);
          }
          // Desliga o LED
          digitalWrite(LED_BOARD,LOW);
          Serial.print(COLOR_GREEN);
          Serial.printf("\n\rDetecta borda de subida - Desliga LED on Board");
          Serial.printf("\n\rLargura Pulso: %d", valuePulso);
          Serial.print(COLOR_RESET);
        }
      }
    }
    // update 
    valueOld = value;

    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

/**
 * @brief 
 * @param parameters 
 */
void Tarefa_Buzzer(void *parameters)
{
  static bool buzzerSts = false;
  uint32_t valuePulso;

  while (1)
  {
    // Checar se há conteudo na Fila
    if(xQueueReceive(QueuePulso, &valuePulso, pdMS_TO_TICKS(10)) == pdPASS)
    {
      buzzerSts = true;

      Serial.print(COLOR_YELLOW);
      Serial.printf("\n\rvaluePulso: %d",valuePulso);
      Serial.print(COLOR_RESET);
      // Checa se o pulso esta entre 1 a 2 segundo
      if((valuePulso >= 1000) && (valuePulso < 2000))
      {
        // Liga Buzzer por 1 segundo
        Buzzer.start();
        Buzzer.note(Buzzer_Note_C,1);
        // Inicia TimerBuzzer com 1 segundo
        xTimerChangePeriod(TimerBuzzer,pdMS_TO_TICKS(1000), 0);
        xTimerStart(TimerBuzzer, 0 );
        Serial.print(COLOR_YELLOW);
        Serial.printf("\n\rLiga Buzzer por 1 segundo c/ Nota C");
        Serial.print(COLOR_RESET);
      }
      else 
      {
        // Checa se o pulso esta entre 2 a 3 segundo
        if((valuePulso >= 2000) && (valuePulso < 3000))
        {
          // Liga Buzzer por 2 segundo
          Buzzer.start();
          Buzzer.note(Buzzer_Note_E,2);
          // Inicia TimerBuzzer com 2 segundo
          xTimerChangePeriod(TimerBuzzer,pdMS_TO_TICKS(2000), 0);
          xTimerStart(TimerBuzzer, 0 );
          Serial.print(COLOR_YELLOW);
          Serial.printf("\n\rLiga Buzzer por 2 segundo c/ Nota E");
          Serial.print(COLOR_RESET);
        }
        else 
        {
          if((valuePulso > 1) && (valuePulso < 1000))
          {
            // Liga Buzzer 
            Buzzer.start();
            Buzzer.note(Buzzer_Note_F,0);
            // Inicia TimerBuzzer com 500 millessegundos
            xTimerChangePeriod(TimerBuzzer,pdMS_TO_TICKS(valuePulso), 0);
            xTimerStart(TimerBuzzer, 0 );
            Serial.print(COLOR_YELLOW);
            Serial.printf("\n\rLiga Buzzer por %d milissegundos c/ Nota F",valuePulso);
            Serial.print(COLOR_RESET);
          }
          else
          {
            // Liga Buzzer por 500 millessegundos
            Buzzer.start();
            Buzzer.note(Buzzer_Note_A,4);
            // Inicia TimerBuzzer com 500 millessegundos
            xTimerChangePeriod(TimerBuzzer,pdMS_TO_TICKS(500), 0);
            xTimerStart(TimerBuzzer, 0 );
            Serial.print(COLOR_YELLOW);
            Serial.printf("\n\rLiga Buzzer por 0,5 segundo c/ Nota A");
            Serial.print(COLOR_RESET);
          }
        }
      }
    }

    // Checa se o semaforo binario esta live
    if(xSemaphoreTake(xSemaphore_BuzzerOff, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      if(buzzerSts == true)
      {
        buzzerSts = false;
        // Encera o TimerBuzzer
        xTimerStop(TimerBuzzer, 0);
        // Desliga Buzzer
        Buzzer.stop();
        Serial.print(COLOR_YELLOW);
        Serial.printf("\n\rDesliga Buzzer");
        Serial.print(COLOR_RESET);
      }
    }

  }
}

/**
 * @brief 
 * @param timer 
 */
void Callback_TimerPulso(TimerHandle_t timer)
{
  if(timer == TimerPulso)
  {
    PulseTimerCounter_Increment();
  }
}

/**
 * @brief 
 * @param timer 
 */
void Callback_TimerBuzzer(TimerHandle_t timer)
{
  if(timer == TimerBuzzer)
  {
    xSemaphoreGiveFromISR(xSemaphore_BuzzerOff, (BaseType_t)(pdFALSE));
  }
}

/**
 * @brief 
 * @return uint32_t 
 */
uint32_t PulseTimerCounter_Get(void)
{
  uint32_t ret;
  // Obtem o Mutex Variavel Global xMutex_PulseTimerCounter
  xSemaphoreTake(xMutex_PulseTimerCounter,portMAX_DELAY );
  ret = PulseTimerCounter;
  // libera o Mutex Variavel Global xMutex_PulseTimerCounter
  xSemaphoreGive(xMutex_PulseTimerCounter);
  return ret;
}

/**
 * @brief 
 */
void PulseTimerCounter_Clear(void)
{
  // Obtem o Mutex Variavel Global xMutex_PulseTimerCounter
  xSemaphoreTake(xMutex_PulseTimerCounter,portMAX_DELAY );
  PulseTimerCounter = 0;
  // libera o Mutex Variavel Global xMutex_PulseTimerCounter
  xSemaphoreGive(xMutex_PulseTimerCounter);
}

/**
 * @brief  
 */
void PulseTimerCounter_Increment(void)
{
  // Obtem o Mutex Variavel Global xMutex_PulseTimerCounter
  xSemaphoreTake(xMutex_PulseTimerCounter,portMAX_DELAY );
  PulseTimerCounter += SW_TIMER_PERIOD_PULSO;
  // libera o Mutex Variavel Global xMutex_PulseTimerCounter
  xSemaphoreGive(xMutex_PulseTimerCounter);
}
