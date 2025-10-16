Button start              -> B13, Pull up

IRTransmitter             ->  A0




-----------
En STM2CubeIDE con un STM32F411 de una blackpill, quiero enviar un codigo RC-5 a traves de un emisor infra rojo



Ver razonamiento
¡Claro que sí! Para enviar un código RC-5 con un STM32F411 (Black Pill) usando STM32CubeIDE, necesitas combinar dos de sus periféricos TIMER. Uno generará la portadora de 36 kHz y el otro se encargará de la temporización y modulación de los bits según el protocolo RC-5 (codificación Manchester).

Aquí tienes una guía completa, desde la configuración hasta el código.

## 1. Entendiendo el Protocolo RC-5
Antes de empezar, es crucial entender dos conceptos clave del RC-5:

Portadora de 36 kHz: El emisor IR no emite una señal continua. Emite ráfagas de una onda cuadrada de 36 kHz. Cuando el protocolo dice "enviar un 1", en realidad significa "encender la portadora de 36 kHz". Cuando dice "enviar un 0", significa "apagar la portadora".

Codificación Manchester: RC-5 no envía simplemente niveles altos o bajos. Cada bit se codifica con una transición en el medio de su período de tiempo (que dura 1.778 ms).

Para enviar un '0': La señal pasa de BAJO a ALTO en mitad del bit.

Para enviar un '1': La señal pasa de ALTO a BAJO en mitad del bit.

Esto significa que nuestro programa debe controlar la portadora de 36 kHz en intervalos precisos de 889 µs (la mitad de un bit).

## 2. Configuración en STM32CubeIDE (.ioc)
Vamos a usar dos timers:

TIM2 (o similar): Para generar la portadora de 36 kHz en modo PWM.

TIM3 (o similar): Para controlar la temporización de los bits (interrupciones cada 889 µs).

Paso 2.1: Configurar el Timer de la Portadora (TIM2)
Busca un pin que tenga capacidad de salida PWM de un timer (por ejemplo, PA0 que puede usar TIM2_CH1).

En el panel de la izquierda, ve a Timers -> TIM2.

Configuración:

Clock Source: Internal Clock.

Channel1: PWM Generation CH1.

Parámetros (Pestaña Parameter Settings):

Aquí calculamos los valores para obtener 36 kHz. Suponiendo que tu APB1 Timer Clock es de 84 MHz (típico en un F411 a máxima velocidad):

Prescaler (PSC): 84 - 1. Esto reduce la frecuencia del contador a 84 MHz / 84 = 1 MHz.

Counter Period (ARR): 28 - 1. Esto define el período. Frecuencia = 1 MHz / 28 = ~35.7 kHz, lo cual es suficientemente cercano.

Pulse (CCR1): 14. Esto nos da un duty cycle del 50% (14 / 28), ideal para la portadora IR.

El resto de parámetros pueden quedarse por defecto.

Paso 2.2: Configurar el Timer de Temporización (TIM3)
Ve a Timers -> TIM3.

Configuración:

Clock Source: Internal Clock.

Parámetros:

Queremos una interrupción cada 889 µs (medio bit). Usando el mismo reloj de 84 MHz:

Prescaler (PSC): 84 - 1. De nuevo, para tener un "tick" de 1 µs.

Counter Period (ARR): 889 - 1.

Habilitar la Interrupción:

Ve a la pestaña NVIC Settings.

Marca la casilla TIM3 global interrupt.

¡Listo! Guarda el archivo .ioc y genera el código.

## 3. El Código en main.c
Ahora vamos a escribir la lógica para construir y enviar la trama RC-5 usando las interrupciones de TIM3 para controlar el PWM de TIM2.

Añade este código en la sección de USER CODE en tu main.c.

Paso 3.1: Variables Globales y Definiciones
C

/* USER CODE BEGIN PV */
#include <stdint.h>
#include <stdbool.h>

// --- Configuración RC-5 ---
#define RC5_HALF_BIT_TIME_US 889
#define RC5_BIT_COUNT 14

// --- Estado de la transmisión ---
volatile bool rc5_is_transmitting = false;
volatile uint16_t rc5_frame_buffer = 0;
volatile int8_t rc5_bit_index = 0;
volatile bool rc5_is_second_half = false; // ¿Estamos en la 1ª o 2ª mitad del bit?

/* USER CODE END PV */
Paso 3.2: Función para Enviar una Trama RC-5
Esta función construye la trama y activa la transmisión.

C

/* USER CODE BEGIN 4 */

/**
 * @brief Prepara y comienza la transmisión de una trama RC-5.
 * @param address: Dirección del dispositivo (0-31)
 * @param command: Comando a enviar (0-63)
 * @param toggle_bit: El bit de toggle (cambia con cada nueva pulsación)
 */
void RC5_SendFrame(uint8_t address, uint8_t command, bool toggle_bit) {
    if (rc5_is_transmitting) {
        return; // Ya hay una transmisión en curso
    }

    // Construcción de la trama de 14 bits
    // | S2 | S1 | T | A4 A3 A2 A1 A0 | C5 C4 C3 C2 C1 C0 |
    // S2 es un bit extendido (siempre '1' para el estándar RC-5)
    // S1 es el bit de start (siempre '1')
    rc5_frame_buffer = (1 << 13) | (1 << 12) | ((toggle_bit & 1) << 11) |
                       ((address & 0x1F) << 6) | (command & 0x3F);

    // Inicializar estado de transmisión
    rc5_bit_index = RC5_BIT_COUNT - 1; // Empezamos por el bit más significativo (S2)
    rc5_is_second_half = false;
    rc5_is_transmitting = true;

    // Arrancamos el timer de temporización. La ISR se encargará del resto.
    HAL_TIM_Base_Start_IT(&htim3);
}

// Apaga el LED y detiene la transmisión
void RC5_StopTransmission() {
    rc5_is_transmitting = false;
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Apaga la portadora
    HAL_TIM_Base_Stop_IT(&htim3); // Detiene las interrupciones de temporización
}

Paso 3.3: La Lógica en la Interrupción del Timer
Esta es la parte más importante. La interrupción de TIM3 se ejecutará cada 889 µs y decidirá si la portadora (PWM de TIM2) debe estar encendida o apagada.

C

/**
 * @brief Callback que se ejecuta cuando el timer de temporización (TIM3) se desborda.
 * Se llama cada 889us.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) { // Asegurarnos de que es nuestro timer
        if (!rc5_is_transmitting) {
            RC5_StopTransmission();
            return;
        }

        // Extraemos el bit actual que queremos transmitir
        bool current_bit = (rc5_frame_buffer >> rc5_bit_index) & 1;

        // Lógica de codificación Manchester
        // bit '0': Bajo -> Alto
        // bit '1': Alto -> Bajo
        if (current_bit == 0) { // Enviar un '0'
            if (!rc5_is_second_half) {
                // Primera mitad: BAJO (PWM apagado)
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            } else {
                // Segunda mitad: ALTO (PWM encendido)
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            }
        } else { // Enviar un '1'
            if (!rc5_is_second_half) {
                // Primera mitad: ALTO (PWM encendido)
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            } else {
                // Segunda mitad: BAJO (PWM apagado)
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            }
        }

        // Avanzamos al siguiente estado
        if (rc5_is_second_half) {
            rc5_bit_index--; // Hemos terminado un bit completo, pasamos al siguiente
        }
        rc5_is_second_half = !rc5_is_second_half; // Cambiamos de mitad de bit

        // Si ya hemos enviado todos los bits, terminamos
        if (rc5_bit_index < 0) {
            RC5_StopTransmission();
        }
    }
}
/* USER CODE END 4 */
Paso 3.4: Llamada en main()
Ahora, en tu bucle main, puedes llamar a la función RC5_SendFrame cuando quieras, por ejemplo, al pulsar un botón.

C

/* USER CODE BEGIN 2 */
// No es necesario iniciar los timers aquí, lo haremos bajo demanda.
bool toggle = false;
/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Ejemplo de envío: dirección 5, comando 23
    // El bit de toggle debería cambiar cada vez que envías un nuevo comando
    RC5_SendFrame(5, 23, toggle);
    toggle = !toggle; // Invertimos para la próxima vez

    // Esperamos un tiempo antes de enviar el siguiente para no saturar
    HAL_Delay(500);
}
/* USER CODE END 3 */
## 4. Conexión del Hardware
La conexión es muy sencilla:

Conecta el ánodo (pata larga) del emisor IR a VCC (3.3V).

Conecta el cátodo (pata corta) a una resistencia de limitación de corriente (e.g., 220Ω).

Conecta el otro extremo de la resistencia al pin que configuraste para la salida PWM (PA0 en nuestro ejemplo).

¡Importante! Un pin del STM32 no puede entregar suficiente corriente para un LED IR de alta potencia. Para mayor alcance, deberías usar el pin para controlar un transistor (como un BJT NPN o un MOSFET) que conmute el LED IR.

Resumen del Flujo de Trabajo
RC5_SendFrame(): Se llama para iniciar. Construye la trama de 14 bits, resetea las variables de estado y arranca el TIM3 con interrupciones.

ISR de TIM3 (cada 889 µs):

Lee el bit que corresponde transmitir.

Basado en el valor del bit y si es la primera o segunda mitad del período, enciende (HAL_TIM_PWM_Start) o apaga (HAL_TIM_PWM_Stop) la portadora generada por TIM2.

Avanza al siguiente estado (siguiente mitad de bit o siguiente bit).

Fin de la trama: Cuando se han enviado los 14 bits, la ISR detiene ambos timers y la transmisión finaliza.

Para depurar, un osciloscopio o un analizador lógico es increíblemente útil para visualizar tanto la portadora de 36 kHz como la trama modulada final. ¡Buena suerte! 🚀








Quiero el mismo ejemplo, pero con un interruptor (pull up) en el pin PB13



Ver razonamiento
¡Por supuesto! Aquí tienes la versión modificada del ejemplo anterior para que la transmisión del código RC-5 se active al presionar un interruptor en el pin PB13, configurado con una resistencia pull-up interna.

El principio de funcionamiento es el mismo: un timer genera la portadora y otro gestiona la temporización de los bits. Lo único que cambia es el disparador: en lugar de un bucle con retardo, ahora leeremos un pin de entrada.

## 1. Configuración en STM32CubeIDE (.ioc)
Los pasos para los Timers son idénticos al ejemplo anterior. Solo añadiremos la configuración del pin del botón.

Paso 1.1: Configurar Timers (Sin cambios)
TIM2: Configurado en PWM Generation CH1 en el pin PA0 para generar la portadora de ~36 kHz.

Prescaler (PSC): 84 - 1

Counter Period (ARR): 28 - 1

Pulse (CCR1): 14 (para un 50% de duty cycle).

TIM3: Configurado para generar una interrupción cada 889 µs.

Prescaler (PSC): 84 - 1

Counter Period (ARR): 889 - 1

Habilitar la interrupción global en la pestaña NVIC Settings.

Paso 1.2: Configurar el Pin del Botón (Nuevo)
Busca el pin PB13 en el chip y haz clic en él.

Selecciona la opción GPIO_Input.

Ahora, en el panel de la izquierda, ve a System Core -> GPIO.

Selecciona la fila correspondiente a PB13.

En los parámetros de configuración del pin, cambia GPIO pull-up/pull-down a Pull-up.

Esto significa que el pin estará en estado ALTO (1) por defecto. Cuando presiones el botón (conectándolo a tierra), el pin pasará a estado BAJO (0).

Guarda el archivo .ioc para que STM32CubeIDE genere el código de inicialización actualizado.

## 2. El Código en main.c
Las funciones de bajo nivel para RC-5 y la interrupción del timer no cambian. Solo modificaremos las variables globales ligeramente y el bucle principal while(1).

Paso 2.1: Variables Globales y Funciones RC-5 (Sin cambios)
Copia exactamente el mismo código del ejemplo anterior en la sección USER CODE BEGIN PV y USER CODE BEGIN 4. Esto incluye:

Las definiciones (#define).

Las variables volátiles para el estado de la transmisión.

La función RC5_SendFrame().

La función RC5_StopTransmission().

La función de callback HAL_TIM_PeriodElapsedCallback().

Paso 2.2: Modificar el Bucle Principal en main()
Aquí es donde está el cambio principal. Reemplaza el código del bucle infinito anterior por este. Esta nueva lógica se encarga de leer el botón, gestionar el antirrebote (debounce) y disparar la transmisión.

C

/* USER CODE BEGIN 2 */
// Variable para el bit de toggle del RC-5.
// Cambia de estado con cada nueva pulsación.
bool rc5_toggle_bit = false;
/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  // 1. Comprobar si el botón está presionado.
  // Como usamos PULL-UP, el estado presionado es GPIO_PIN_RESET (BAJO).
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
  {
    // 2. Antirrebote (debounce) - una pequeña pausa.
    // Esperamos un momento para asegurarnos de que no es un ruido eléctrico.
    HAL_Delay(50); // 50 ms es un valor común.

    // 3. Volver a comprobar si el botón sigue presionado.
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
      // ¡Botón confirmado!
      // Solo enviamos si no hay otra transmisión en curso.
      if (!rc5_is_transmitting)
      {
        // 4. Enviar la trama RC-5.
        // Ejemplo: Dirección 5, Comando 23.
        RC5_SendFrame(5, 23, rc5_toggle_bit);

        // 5. Invertir el bit de toggle para la próxima pulsación.
        rc5_toggle_bit = !rc5_toggle_bit;
      }
      
      // 6. Esperar a que el botón se suelte.
      // Esto evita que se envíen múltiples tramas si se mantiene pulsado.
      while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
      {
        // No hacer nada hasta que el pin vuelva a ALTO.
      }
    }
  }
}
/* USER CODE END 3 */
## 3. Conexión del Hardware Actualizada
Ahora tienes dos componentes que conectar:

Emisor Infrarrojo (en PA0):

Conecta el ánodo del LED IR a 3.3V.

Conecta el cátodo del LED IR a una resistencia de 220Ω.

Conecta el otro extremo de la resistencia al pin PA0.

Interruptor / Pulsador (en PB13):

Conecta una pata del pulsador al pin PB13.

Conecta la otra pata del pulsador a GND (tierra).

Resumen del Nuevo Flujo
El programa se inicia y entra en el bucle while(1), donde constantemente comprueba el estado del pin PB13.

Gracias a la resistencia pull-up, PB13 está en ALTO. El bucle no hace nada.

Cuando presionas el botón, conectas PB13 a tierra, y el pin pasa a estado BAJO.

El if detecta el estado BAJO. Espera 50 ms para filtrar rebotes y vuelve a comprobar.

Si el pin sigue en BAJO, llama a RC5_SendFrame() para iniciar la transmisión.

La lógica de los timers y las interrupciones se encarga de enviar la trama completa, exactamente igual que antes.

El código se queda esperando en un bucle while hasta que sueltes el botón (el pin vuelve a ALTO).

El ciclo se repite, listo para la siguiente pulsación.












Herramientas

Gemini puede cometer errores, incluso sobre personas, así que verific