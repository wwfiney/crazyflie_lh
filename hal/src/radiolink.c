/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * radiolink.c: nRF24L01 implementation of the CRTP link
 */

#include <stdbool.h>
#include <errno.h>

#include "nrf24l01.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "debug.h"

static bool isInit;

#define RADIO_CONNECTED_TIMEOUT   M2T(2000)

/* Synchronisation */
xSemaphoreHandle dataRdy;
/* Data queue */
xQueueHandle txQueue;
xQueueHandle rxQueue;

static uint32_t lastPacketTick;

//Union used to efficiently handle the packets (Private type)
typedef union
{
  CRTPPacket crtp;
  struct {
    uint8_t size;
    uint8_t data[32];
  } __attribute__((packed)) raw;
} RadioPacket;

static struct {
  bool enabled;
} state;

static void interruptCallback()
{
  portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

  //To unlock RadioTask
  xSemaphoreGiveFromISR(dataRdy, &xHigherPriorityTaskWoken);

  if(xHigherPriorityTaskWoken)
    vPortYieldFromISR();
}

// 'Class' functions, called from callbacks
static int setEnable(bool enable)
{
  nrfSetEnable(enable);
  state.enabled = enable;

  return 0;
}

static int sendPacket(CRTPPacket * pk)
{
  if (!state.enabled)
    return ENETDOWN;
  xQueueSend( txQueue, pk, portMAX_DELAY);

  return 0;
}

static int receivePacket(CRTPPacket * pk)
{
  if (!state.enabled)
    return ENETDOWN;

  xQueueReceive( rxQueue, pk, portMAX_DELAY);

  return 0;
}

static int reset(void)
{
  xQueueReset(txQueue);
  nrfFlushTx();

  return 0;
}

static bool isConnected(void)
{
  if ((xTaskGetTickCount() - lastPacketTick) > RADIO_CONNECTED_TIMEOUT)
    return false;

  return true;
}

#if defined(__CC_ARM)
static struct crtpLinkOperations radioOp =
{
  setEnable,
  sendPacket,
  receivePacket,
  isConnected,
  reset,
};
#else
static struct crtpLinkOperations radioOp =
{
  .setEnable         = setEnable,
  .sendPacket        = sendPacket,
  .receivePacket     = receivePacket,
  .isConnected       = isConnected,
  .reset             = reset,
};
#endif

extern void updateRPY(float roll, float pitch, float yaw, uint16_t thrust);
/* Radio task handles the CRTP packet transfers as well as the radio link
 * specific communications (eg. Scann and ID ports, communication error handling
 * and so much other cool things that I don't have time for it ...)
 */
static void radiolinkTask(void * arg)
{
  unsigned char dataLen;
  static RadioPacket pk;
  static unsigned char recbuffer[32];
  int i;
  bool havedata = false;
    uint16_t thrust_test = 0;
    float roll_test = 0;
    float pitch_test = 0;
    float yaw_test = 0;

  memset(recbuffer, 0, 32);
  memset(recbuffer, 0x3c, 32);

  //Packets handling loop
  while(1)
  {
    ledseqRun(LED_GREEN, seq_linkup);

    xSemaphoreTake(dataRdy, portMAX_DELAY);
    lastPacketTick = xTaskGetTickCount();
    
    nrfSetEnable(false);

    //DEBUG_PRINT("st 0x%x\n", nrfRead1Reg(REG_STATUS));
    
    //Fetch all the data (Loop until the RX Fifo is NOT empty)
    while( !(nrfRead1Reg(REG_FIFO_STATUS)&0x01) )
    {
      dataLen = nrfRxLength(0);

      //DEBUG_PRINT("Get data %d bytes!\n", dataLen);

      if (dataLen>32)          //If a packet has a wrong size it is dropped
        nrfFlushRx();
      else                     //Else, it is processed
      {
        //Fetch the data
        pk.raw.size = dataLen-1;
        //nrfReadRX((char *)pk.raw.data, dataLen);
        nrfReadRX(recbuffer, dataLen);

    #define LH_ROLL_MIN     901
    #define LH_ROLL_MID     1387
    #define LH_ROLL_MAX     1901
    #define LH_ROLL_LIMIT   40.0

    #define LH_PITCH_MIN     945
    #define LH_PITCH_MID     1462
    #define LH_PITCH_MAX     1945
    #define LH_PITCH_LIMIT   40.0

    #define LH_YAW_MIN     937
    #define LH_YAW_MID     1487
    #define LH_YAW_MAX     1937
    #define LH_YAW_LIMIT   40
        thrust_test = recbuffer[6] + (recbuffer[7] << 8);
        roll_test = recbuffer[0] + (recbuffer[1] << 8);
        pitch_test = recbuffer[2] + (recbuffer[3] << 8);
        yaw_test = recbuffer[4] + (recbuffer[5] << 8);

        roll_test = (roll_test - LH_ROLL_MID) / ((LH_ROLL_MAX-LH_ROLL_MIN) / 2 / LH_ROLL_LIMIT);
        if(roll_test > LH_ROLL_LIMIT)
            roll_test = LH_ROLL_LIMIT;
        if(roll_test < -LH_ROLL_LIMIT)
            roll_test = -LH_ROLL_LIMIT;

        pitch_test = (pitch_test - LH_PITCH_MID) / ((LH_PITCH_MAX-LH_PITCH_MIN) / 2 / LH_PITCH_LIMIT);
        if(pitch_test > LH_PITCH_LIMIT)
            pitch_test = LH_PITCH_LIMIT;
        if(pitch_test < -LH_PITCH_LIMIT)
            pitch_test = -LH_PITCH_LIMIT;

#if 1
        yaw_test = (yaw_test - LH_YAW_MID) / ((LH_YAW_MAX-LH_YAW_MIN) / 2 / LH_YAW_LIMIT);
        if(yaw_test > LH_YAW_LIMIT)
            yaw_test = LH_YAW_LIMIT;
        if(yaw_test < -LH_YAW_LIMIT)
            yaw_test = -LH_YAW_LIMIT;

        //DEBUG_PRINT("y:%f\n", yaw_test);
#endif
        updateRPY(roll_test, pitch_test, yaw_test, thrust_test);

#if 0
        DEBUG_PRINT("t:%d\n", thrust_test);
        DEBUG_PRINT("r:%f\n", roll_test);
        DEBUG_PRINT("p:%f\n", pitch_test);
        DEBUG_PRINT("y:%f\n", yaw_test);
#endif

#if 0
        havedata = false;
       
        for(i = 0; i < dataLen; i++) {
            if(recbuffer[i] != 0) {
                DEBUG_PRINT("0x%x\n", recbuffer[i]);
                havedata = true;
            }
        }

        if(!havedata) {
            DEBUG_PRINT("all 0\n");
        }
#endif


        //nrfWriteAck(0, recbuffer, 8);

        //Push it in the queue (If overflow, the packet is dropped)
        //if (!CRTP_IS_NULL_PACKET(pk.crtp))  //Don't follow the NULL packets
        //  xQueueSend( rxQueue, &pk, 0);
      }
    }

    //Push the data to send (Loop until the TX Fifo is full or there is no more data to send)
    #if 0
    while( (uxQueueMessagesWaiting((xQueueHandle)txQueue) > 0) && !(nrfRead1Reg(REG_FIFO_STATUS)&0x20) )
    {
      xQueueReceive(txQueue, &pk, 0);
      pk.raw.size++;

      nrfWriteAck(0, (char*) pk.raw.data, pk.raw.size);
    }
    #endif

    //clear the interruptions flags
    nrfWrite1Reg(REG_STATUS, 0x70);
    
    //Re-enable the radio
    nrfSetEnable(true);
  }
}

static void radiolinkInitNRF24L01P(void)
{
  int i;
  char radioAddress[5] = {0x12,0x34,0x56,0x78,0xA0};//for Liehuo {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

  //Set the radio channel
  nrfSetChannel(configblockGetRadioChannel());
  //Set the radio data rate
  nrfSetDatarate(configblockGetRadioSpeed());
  //Set radio address
  nrfSetAddress(0, radioAddress);

  //Power the radio, Enable the DS interruption, set the radio in PRX mode
  nrfWrite1Reg(REG_CONFIG, 0x3F);
  vTaskDelay(M2T(2)); //Wait for the chip to be ready
  // Enable the dynamic payload size and the ack payload for the pipe 0
  //DEBUG_PRINT("Before 0x%x\n", nrfRead1Reg(REG_FEATURE));
  nrfWrite1Reg(REG_FEATURE, 0x06);
  //nrfWrite1Reg(REG_FEATURE, 0x04);
  //DEBUG_PRINT("After 0x%x\n", nrfRead1Reg(REG_FEATURE));
  nrfWrite1Reg(REG_DYNPD, 0x01);

  //Flush RX
  for(i=0;i<3;i++)
    nrfFlushRx();
  //Flush TX
  for(i=0;i<3;i++)
    nrfFlushTx();
}

/*
 * Public functions
 */

void radiolinkInit()
{
  if(isInit)
    return;

  nrfInit();

  nrfSetInterruptCallback(interruptCallback);

  vTaskSetApplicationTaskTag(0, (void*)TASK_RADIO_ID_NBR);

  /* Initialise the semaphores */
  vSemaphoreCreateBinary(dataRdy);

  /* Queue init */
  rxQueue = xQueueCreate(3, sizeof(RadioPacket));
  txQueue = xQueueCreate(3, sizeof(RadioPacket));

  radiolinkInitNRF24L01P();

    /* Launch the Radio link task */
  xTaskCreate(radiolinkTask, (const signed char * const)"RadioLink",
              configMINIMAL_STACK_SIZE+32, NULL, /*priority*/1, NULL);

  isInit = true;
}

bool radiolinkTest()
{
  return nrfTest();
}

struct crtpLinkOperations * radiolinkGetLink()
{
  return &radioOp;
}

void radiolinkReInit(void)
{
  if (!isInit)
    return;

  radiolinkInitNRF24L01P();
}
