//Hoverboard Manual Speed - Versão Modificada para 4 Motores sem Díodos
//Modificado para trocar pinos RX dinamicamente

#define _DEBUG      // debug output to first hardware serial port
//#define DEBUG_RX    // additional hoverboard-rx debug output
#define REMOTE_UARTBUS

#define SEND_MILLIS 100   // send commands to hoverboard every SEND_MILLIS millisesonds

#include <Arduino.h>
#include "util.h"
#include "hoverserial.h"

// --- CONFIGURAÇÃO FÍSICA ---
#ifndef oSerialHover
#define oSerialHover Serial2    // ESP32 usa Serial2 por defeito
#endif

// Mapa de Pinos RX para os 4 Motores
// [0] é ignorado, [1]=Motor1, [2]=Motor2, etc.
// Verifica se estes pinos estão livres no teu ESP32!
const int rx_pins_map[5] = {-1, 1, 2, 20, 21}; 
const int tx_pin_shared = 19; // Pino TX único ligado a todas as placas

// ---------------------------

#ifndef TOKEN_PASSING_ENABLED
#define TOKEN_PASSING_ENABLED 1
#endif

#if TOKEN_PASSING_ENABLED
#ifndef TOKEN_FRAME_TYPE
#define TOKEN_FRAME_TYPE 3
#endif
#ifndef TOKEN_MASTER_ID
#define TOKEN_MASTER_ID 0xFE
#endif
#ifndef TOKEN_STALE_TIMEOUT
#define TOKEN_STALE_TIMEOUT 80UL
#endif
#endif

#define input_serial

// --- CONFIGURAÇÃO DOS MOTORES ---
// Temos 4 motores no total
const size_t motor_count_total = 4;
// IDs que vamos procurar (1, 2, 3 e 4)
int motors_all[motor_count_total] = {1, 2, 3, 4};

// Separação lógica (se quiseres controlar esquerda/direita separadamente depois)
const size_t motor_count_left = 2;
int motors_left[motor_count_left] = {1, 3}; // Motores ímpares na esquerda
const size_t motor_count_right = 2;
int motors_right[motor_count_right] = {2, 4}; // Motores pares na direita

int motor_speed[10]; // array para guardar velocidades
int slave_state[10]; // array para guardar estados

// Variáveis Globais
uint32_t iLast = 0;
uint32_t iNext = 0;
uint32_t iTimeNextState = 0;
int iStep = 20;
int iSpeed = 0;
uint8_t wState = 1; // 1=Green, 2=Orange, 4=Red, 8=Up, 16=Down
int count = 0;

SerialHover2Server oHoverFeedback;
SerialHover2Server oHoverFeedbackAll[10]; // Guarda feedback de até 10 motores
uint32_t iHoverFeedbackAges[10];

// Protótipos
void ServiceTokenScheduler(uint32_t iNow);
void RecordFeedbackAge(uint8_t id, uint32_t iNow);

// ==========================================================
// SETUP
// ==========================================================
void setup() 
{
  Serial.begin(115200);
  Serial.println("Hoverboard Serial v2.0 - 4 Motor Logic (No Diodes)");

  // Inicializa a Serial do Hoverboard
  // Começamos por defeito a ouvir o Motor 1 (Pino 16)
  HoverSetupEsp32(oSerialHover, 19200, rx_pins_map[1], tx_pin_shared);
  
  // Limpa arrays
  for(int i=0; i<10; i++) {
    motor_speed[i] = 0;
    slave_state[i] = 0; // 0 = COM_VOLT mode usually
    iHoverFeedbackAges[i] = 0;
  }
}

// ==========================================================
// LOOP PRINCIPAL
// ==========================================================
void loop() 
{
  uint32_t iNow = millis();

  // 1. Processa inputs do Computador (USB)
  #ifdef input_serial
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      // Formato: hover|id|speed|state
      // Exemplo: hover|all|300|1  OU  hover|1|100|1
      
      if (command.startsWith("hover")) {
        int firstPipe = command.indexOf('|');
        int secondPipe = command.indexOf('|', firstPipe + 1);
        int thirdPipe = command.indexOf('|', secondPipe + 1);
        int fourthPipe = command.indexOf('|', thirdPipe + 1);

        String idStr = command.substring(firstPipe + 1, secondPipe);
        int speedVal = command.substring(secondPipe + 1, thirdPipe).toInt();
        int stateVal = command.substring(thirdPipe + 1).toInt();

        if (idStr == "all") {
           for (int i = 0; i < motor_count_total; i++) {
             motor_speed[motors_all[i]] = speedVal;
             slave_state[motors_all[i]] = stateVal;
           }
           Serial.println("All motors updated");
        } else if (idStr == "left") {
           for (int i = 0; i < motor_count_left; i++) {
             motor_speed[motors_left[i]] = speedVal;
             slave_state[motors_left[i]] = stateVal;
           }
        } else if (idStr == "right") {
           for (int i = 0; i < motor_count_right; i++) {
             motor_speed[motors_right[i]] = speedVal;
             slave_state[motors_right[i]] = stateVal;
           }
        } else if (idStr == "stop") {
           for(int i=0; i<10; i++) motor_speed[i] = 0;
           Serial.println("STOP");
        } else {
           int id = idStr.toInt();
           if(id > 0 && id < 10) {
             motor_speed[id] = speedVal;
             slave_state[id] = stateVal;
             Serial.print("Motor "); Serial.print(id); Serial.println(" updated");
           }
        }
      } else if (command.startsWith("stop")) {
           for(int i=0; i<10; i++) motor_speed[i] = 0;
           Serial.println("STOP ALL");
      }
    }
  #endif

  // 2. Recebe dados do Hoverboard (Leitura)
  boolean bReceived;   
  while (bReceived = Receive(oSerialHover, oHoverFeedback))
  {
    // Debug para ver se recebemos algo
    DEBUGT("RX ID", oHoverFeedback.iSlave);
    //DEBUGT("Speed", oHoverFeedback.iSpeed);
    HoverLog(oHoverFeedback);
    // Guarda os dados na cache global
    if (oHoverFeedback.iSlave >= 1 && oHoverFeedback.iSlave <= 4) {
        memcpy(&oHoverFeedbackAll[oHoverFeedback.iSlave], &oHoverFeedback, sizeof(SerialHover2Server));
    }

    #if TOKEN_PASSING_ENABLED
      RecordFeedbackAge(oHoverFeedback.iSlave, iNow);
    #endif
    iLast = iNow;
  }

  // 3. Envia Comandos de Velocidade (Broadcast)
  // Isto envia os comandos de velocidade periodicamente, independente do token
  if (iNow > iNext)
  {
    // Envia comandos para todos os motores definidos
    for (int i = 0; i < motor_count_total; i++) {
        int id = motors_all[i];
        
        // Constrói pacote de comando
        SerialServer2HoverMaster oCommand;
        oCommand.cStart = '/';
        oCommand.iDataType = 1; // Tipo 1 = Master Control
        oCommand.iSlave = id;
        oCommand.iSpeed = motor_speed[id];
        oCommand.iSteer = 0; // Não usamos steer neste modo, controlamos direto a velocidade
        oCommand.wState = slave_state[id]; // LED/Estado
        oCommand.wStateSlave = slave_state[id]; 
        oCommand.checksum = CalcCRC((uint8_t*)&oCommand, sizeof(oCommand)-2);

        // Envia para o barramento
        UART_Send((uint8_t*)&oCommand, sizeof(oCommand)); 
        
        // Pequeno delay para não engasgar o buffer de TX
        delayMicroseconds(500);
    }
    
    iNext = iNow + SEND_MILLIS;
  }

  // 4. Gestão do Token (Quem pode falar?)
  #if TOKEN_PASSING_ENABLED
    ServiceTokenScheduler(iNow);
  #endif
}

// ==========================================================
// FUNÇÕES AUXILIARES
// ==========================================================

void UART_Send(uint8_t *pBuffer, uint8_t size) {
    oSerialHover.write(pBuffer, size);
}

void SendTokenFrame(uint8_t recipient, uint16_t counterValue)
{
  SerialTokenPass oToken;
  oToken.cStart = '/';
  oToken.iDataType = TOKEN_FRAME_TYPE;
  oToken.iSlave = recipient;
  oToken.iSender = TOKEN_MASTER_ID;
  oToken.tokenCounter = counterValue;
  oToken.checksum = CalcCRC((uint8_t*) &oToken, sizeof(oToken) - 2);
  UART_Send((uint8_t*) &oToken, sizeof(oToken));
}

// --- GESTÃO DE TOKENS E MUDANÇA DE PINO ---
static uint8_t iNextTokenIndex = 0;
static uint32_t iNextTokenTime = 0;
static uint16_t iTokenCounter = 0;
static uint32_t iWaitResponseTimeout = 0;

// Tempo máximo para esperar resposta de um escravo (em ms)
// Aumenta se tiveres erros de CRC, diminui para mais velocidade
#define TOKEN_RETURN_TIMEOUT 40 

void ServiceTokenScheduler(uint32_t iNow)
{
  // Verifica se estamos à espera de uma resposta e se já passou o tempo (Timeout)
  if (iWaitResponseTimeout > 0 && iNow > iWaitResponseTimeout) {
      // Timeout! O motor não respondeu. Passa para o próximo.
      // DEBUGT("Timeout ID", motors_all[iNextTokenIndex]);
      
      iNextTokenIndex++;
      if (iNextTokenIndex >= motor_count_total) iNextTokenIndex = 0;
      
      iWaitResponseTimeout = 0; // Reseta timeout para permitir envio imediato
      iNextTokenTime = iNow;    // Envia próximo já
  }

  // Se estiver pronto para enviar novo token
  if (iWaitResponseTimeout == 0 && iNow >= iNextTokenTime)
  {
      uint8_t targetId = motors_all[iNextTokenIndex];
      
      // === A MÁGICA ACONTECE AQUI ===
      // Antes de pedir ao motor para falar, mudamos o ouvido (RX) do ESP32!
      if(targetId >= 1 && targetId <= 4) {
          // Muda o pino RX para o que está ligado ao motor alvo
          // Mantém o TX no pino partilhado (17)
          oSerialHover.setPins(rx_pins_map[targetId], tx_pin_shared); 
      }
      // ==============================

      // Envia o Token (Permissão para falar)
      SendTokenFrame(targetId, ++iTokenCounter);
      
      // Define o timeout: se ele não responder em X ms, desistimos dele
      iWaitResponseTimeout = iNow + TOKEN_RETURN_TIMEOUT;
  }
}

void RecordFeedbackAge(uint8_t id, uint32_t iNow) {
    // Esta função é chamada quando recebemos dados com sucesso
    if (id == motors_all[iNextTokenIndex]) {
        // Se recebemos do motor que esperávamos:
        // 1. Limpa o timeout (já não precisamos esperar)
        iWaitResponseTimeout = 0;
        
        // 2. Agenda o próximo motor imediatamente
        iNextTokenIndex++;
        if (iNextTokenIndex >= motor_count_total) iNextTokenIndex = 0;
        iNextTokenTime = iNow + 2; // Espera 2ms só para estabilidade elétrica
        
        // DEBUGT("Success ID", id);
    }
    
    if (id < 10) iHoverFeedbackAges[id] = iNow;
}