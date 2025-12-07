/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
////                                                                 ////
////                 Remote UARTBus protocol by robo                 ////
////                  (Corrected & Optimized)                        ////
////                                                                 ////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

#ifdef TARGET_MM32SPIN25
#include "HAL_device.h"
#else
#include "mm32_device.h"
#endif         
#include "math.h" 
#include "../Inc/pinout.h"
#include "../Inc/bldc.h"
#include "../Inc/uart.h"
#include "stdio.h"
#include "string.h"
#include "../Inc/remoteUartBus.h"
#include "hal_crc.h"
#include "../Inc/sim_eeprom.h"
#include "../Inc/calculation.h"

// --- Configurações Padrão ---
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
    #ifndef TOKEN_BROADCAST_ID
    #define TOKEN_BROADCAST_ID 0xFF
    #endif
    #ifndef TOKEN_DISCOVERY_INTERVAL
    #define TOKEN_DISCOVERY_INTERVAL 200
    #endif
    #ifndef TOKEN_RECOVER_TIMEOUT
    #define TOKEN_RECOVER_TIMEOUT 100
    #endif
    #ifndef TOKEN_MAX_HOLD_TIME
    #define TOKEN_MAX_HOLD_TIME 5
    #endif
#endif

// Certifique-se de que SERIAL_TIMEOUT esteja definido (geralmente no header)
#ifndef SERIAL_TIMEOUT
#define SERIAL_TIMEOUT 100 // Timeout padrão de segurança (ms)
#endif

#pragma pack(1)

extern uint32_t millis;
extern uint8_t sRxBuffer[10];

uint8_t bNeedTelemetry = 0;

// [CORREÇÃO 1] Inicializar com -1 para garantir a busca pelo caractere de início '/'
static int16_t iReceivePos = -1; 

extern int pwm;
extern int32_t speed;
extern uint8_t  wState;

extern int32_t iOdom;
extern int fvbat;              // global variable for battery voltage
extern int fitotal;            // global variable for current dc
extern int realspeed;          // global variable for real Speed

// --- Estruturas de Dados ---

typedef struct {
   uint8_t cStart;         //  = '/';
   uint8_t iDataType;      //  0
   uint8_t iSlave;
   int16_t iSpeed;
   uint8_t wState;
   uint16_t checksum;
} SerialServer2Hover;

typedef struct {
   uint8_t cStart;         //  = '/';
   uint8_t iDataType;      //  1
   uint8_t iSlave;
   int16_t iSpeed;
   int16_t iSteer;
   uint8_t wState;
   uint8_t wStateSlave;
   uint16_t checksum;
} SerialServer2HoverMaster;

typedef struct {
   uint8_t cStart;         //  = '/';
   uint8_t iDataType;      //  2
   uint8_t iSlave;
   float   fBattFull;
   float   fBattEmpty;
   uint8_t iDriveMode;
   int8_t  iSlaveNew;
   uint16_t checksum;
} SerialServer2HoverConfig;

#define RECEIVE_BUFFER_SIZE 255
static uint8_t aReceiveBuffer[RECEIVE_BUFFER_SIZE];
static uint16_t CalcCRC(uint8_t *ptr, int count);

#define START_FRAME         0xABCD
typedef struct {
   uint16_t cStart;
   uint8_t iSlave;
   int16_t iSpeed;    // 100* km/h
   uint16_t iVolt;    // 100* V
   int16_t iAmp;      // 100* A
   int32_t iOdom;     // hall steps
   uint16_t checksum;
} SerialHover2Server;

#if TOKEN_PASSING_ENABLED
typedef struct {
    uint8_t cStart;
    uint8_t iDataType;
    uint8_t iSlave;
    uint8_t iSender;
    uint16_t tokenCounter;
    uint16_t checksum;
} SerialTokenPass;

uint32_t iTimeLastRx = 0;

// Variáveis do Token
static uint8_t bHasToken = 0;
static uint32_t iTokenTimestamp = 0;
static uint16_t iTokenCounter = 0;
static uint8_t iLastTokenMaster = TOKEN_MASTER_ID;
static uint32_t iLastTokenRequest = 0;

// --- Implementação CRC ---
static uint16_t CalcCRC(uint8_t *ptr, int count){
  uint16_t  crc;
  uint8_t i;
  crc = 0;
  while (--count >= 0)
  {
    crc = crc ^ (uint16_t) *ptr++ << 8;
    i = 8;
    do
    {
      if (crc & 0x8000)
        crc = crc << 1 ^ 0x1021;
      else
        crc = crc << 1;
    } while(--i);
  }
  return (crc);
}

// --- Funções do Token ---

static void SendTokenFrame(uint8_t recipient, uint16_t counterValue)
{
    SerialTokenPass oToken;
    oToken.cStart = '/';
    oToken.iDataType = TOKEN_FRAME_TYPE;
    oToken.iSlave = recipient;
    oToken.iSender = SLAVE_ID;
    oToken.tokenCounter = counterValue;
    oToken.checksum = CalcCRC((uint8_t*) &oToken, sizeof(oToken) - 2);
    UART_Send_Group((uint8_t*) &oToken, sizeof(oToken));
}

static void ReturnTokenToMaster(void)
{
    uint8_t resolved = iLastTokenMaster ? iLastTokenMaster : TOKEN_MASTER_ID;
    SendTokenFrame(resolved, ++iTokenCounter);
    iTokenTimestamp = millis;
    bHasToken = 0;
}

static void RespondDiscovery(uint8_t targetId)
{
    uint8_t resolved = targetId ? targetId : TOKEN_MASTER_ID;
    SendTokenFrame(resolved, ++iTokenCounter);
    iTokenTimestamp = millis;
}

static void RequestTokenFromMaster(void)
{
    uint32_t now = millis;
    if ((now - iLastTokenRequest) < TOKEN_DISCOVERY_INTERVAL)
    {
        return;
    }
    uint8_t resolved = iLastTokenMaster ? iLastTokenMaster : TOKEN_MASTER_ID;
    SendTokenFrame(resolved, ++iTokenCounter);
    iLastTokenRequest = now;
}
#endif // TOKEN_PASSING_ENABLED

// --- Funções Principais ---

void AnswerMaster(void); // Forward declaration

void RemoteUpdate(void){
    
    // Safety: Stop motor if connection lost
    if (millis - iTimeLastRx > SERIAL_TIMEOUT){
        speed = 0;
    }

#if TOKEN_PASSING_ENABLED
    if (bHasToken && bNeedTelemetry)
    {
        AnswerMaster();
        bNeedTelemetry = 0;
        ReturnTokenToMaster();
    }
    else if (bHasToken && (millis - iTokenTimestamp) > TOKEN_MAX_HOLD_TIME)
    {
        // Timeout holding the token, give it back even if no telemetry needed
        ReturnTokenToMaster();
    }
    else if (!bHasToken && (bNeedTelemetry || (millis - iTokenTimestamp) > TOKEN_RECOVER_TIMEOUT))
    {
        // Lost token or need to speak? Ask for it.
        RequestTokenFromMaster();
    }
#else
    if (bNeedTelemetry)
    {
        AnswerMaster();
        bNeedTelemetry = 0;
    }
#endif
}

extern uint32_t steerCounter;
uint32_t iAnswerMaster = 0;

void AnswerMaster(void){
    
    SerialHover2Server oData;
    oData.cStart = START_FRAME;
    oData.iSlave = SLAVE_ID;
    oData.iVolt = (uint16_t) (fvbat);
    oData.iAmp = (int16_t)   (fitotal);
    oData.iSpeed = (int16_t) (realspeed * 10);
    oData.iOdom = (int32_t) iOdom;
    
    // Calc CRC excluding the CRC field itself (last 2 bytes)
    oData.checksum = CalcCRC((uint8_t*) &oData, sizeof(oData) - 2);

    UART_Send_Group((uint8_t*) &oData, sizeof(oData));
}

uint8_t iRxDataType;
uint8_t iRxDataSize;

// Process incoming UART bytes
void serialit(void){

    uint8_t cRead = sRxBuffer[0];
    
    // [CORREÇÃO 1] State Machine Initialization
    if (iReceivePos < 0)
    {
        if (cRead == '/')   // Start character detected
            iReceivePos = 0;
        else    
            return;         // Ignore garbage bytes
    }
    
    // [CORREÇÃO 2] Buffer Overflow Protection
    if (iReceivePos >= RECEIVE_BUFFER_SIZE) {
        iReceivePos = -1; // Reset if buffer overrun
        return;
    }

    aReceiveBuffer[iReceivePos++] = cRead;
    
    if (iReceivePos == 1)   // Just read '/', wait for more
            return;

    if (iReceivePos == 2)   // Read DataType
    {
        iRxDataType = aReceiveBuffer[1];
        switch (iRxDataType)
        {
            case 0: iRxDataSize = sizeof(SerialServer2Hover);       break;
            case 1: iRxDataSize = sizeof(SerialServer2HoverMaster); break;
            case 2: iRxDataSize = sizeof(SerialServer2HoverConfig); break;
#if TOKEN_PASSING_ENABLED
            case TOKEN_FRAME_TYPE: iRxDataSize = sizeof(SerialTokenPass); break;
#endif
            default:
                iReceivePos = -1; // Invalid packet type, reset
                return;
        }
        return;
    }
    
    // Wait until we have the full packet size
    if (iReceivePos < iRxDataSize)
        return;
        
    // Validate Checksum (Last 2 bytes are CRC)
    uint16_t iCRC = (aReceiveBuffer[iReceivePos-1] << 8) | aReceiveBuffer[iReceivePos-2];
    iReceivePos = -1; // Reset for next packet
    
    if (iCRC == CalcCRC(aReceiveBuffer, iRxDataSize - 2))
    {
        uint8_t bCommandHandled = 0;
#if TOKEN_PASSING_ENABLED
        if (iRxDataType == TOKEN_FRAME_TYPE)
        {
            SerialTokenPass* pToken = (SerialTokenPass*) aReceiveBuffer;
            
            // Only accept token logic if targeted or broadcast
            if (pToken->iSlave == SLAVE_ID || pToken->iSlave == TOKEN_BROADCAST_ID) {
                 iTokenCounter = pToken->tokenCounter;
                 iTokenTimestamp = millis;
                 
                 if (pToken->iSender) {
                     iLastTokenMaster = pToken->iSender;
                 }
                 
                 if (pToken->iSlave == SLAVE_ID) {
                     bHasToken = 1;
                     bNeedTelemetry = 1; // Trigger response
                 } else if (pToken->iSlave == TOKEN_BROADCAST_ID) {
                     RespondDiscovery(iLastTokenMaster);
                 }
                 iTimeLastRx = millis;
            }
            return;
        }
#endif
        // Regular Data Packet
        if (aReceiveBuffer[2] == SLAVE_ID)
        {
            iTimeLastRx = millis;
            
            switch (iRxDataType)
            {
                case 0:
                {
                    SerialServer2Hover* pData = (SerialServer2Hover*) aReceiveBuffer;
                    speed = pData->iSpeed;
                    wState = pData->wState;
                    bCommandHandled = 1;
                    break;
                }
                case 1: 
                {
                    SerialServer2HoverMaster* pData = (SerialServer2HoverMaster*) aReceiveBuffer;
                    speed = pData->iSpeed;
                    wState = pData->wState;
                    bCommandHandled = 1;
                    break;
                }
                case 2: 
                {
                    SerialServer2HoverConfig* pData = (SerialServer2HoverConfig*) aReceiveBuffer;
                    if ((pData->fBattFull > 0) && (pData->fBattFull < 60.0))
                    {
                        BAT_FULL = pData->fBattFull * 1000;
                    }
                    if ((pData->fBattEmpty > 0) && (pData->fBattEmpty < 60.0))
                    {
                        BAT_EMPTY = pData->fBattEmpty * 1000;
                    }
                    if (pData->iDriveMode <= SINE_SPEED)
                    {
                        DRIVEMODE = pData->iDriveMode;
                        PIDrst();
                        PID_Init();
                        TIMOCInit();
                    }
                    // [CORREÇÃO 3] Configuração de mudança de ID
                    // Se você quiser habilitar a troca remota de ID, remova o "&& 0" abaixo
                    if (pData->iSlaveNew >= 0 && 0) 
                    {   
                        SLAVE_ID = pData->iSlaveNew;
                        // Salva no EEPROM para persistir após reiniciar
                        EEPROM_Write((u8*)pinstorage, 2 * 64);
                    }
            
                    bCommandHandled = 1;
                    break;
                }
            }
            
            if (bCommandHandled)
            {
                bNeedTelemetry = 1;
#if TOKEN_PASSING_ENABLED
                if (!bHasToken)
                {
                    // Se recebemos um comando válido mas não temos o token, 
                    // solicitamos ao mestre na próxima oportunidade
                    RequestTokenFromMaster();
                }
#endif
            }
        }
    }
}