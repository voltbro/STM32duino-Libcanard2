#define CANARD_ASSERT

#include <Libcanard2.h>
#include "F446_CAN.h"
#include "F446_HSE_init.h"

// UAVCAN related headers
#include <libcanard/canard.h>
#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/primitive/array/Real64_1_0.h>

CanardInstance   canard;   // This is the core structure that keeps all of the states and allocated resources of the library instance
CanardTxQueue   queue;    // Prioritized transmission queue that keeps CAN frames destined for transmission via one CAN interface

// Wrappers for using memory allocator with libcanard
static void *memAllocate(CanardInstance *const canard, const size_t amount);
static void memFree(CanardInstance *const canard, void *const pointer);

// Application-specific function prototypes
void process_canard_TX_queue(void);
void process_canard_receiption(void);

static uint8_t my_message_transfer_id = 0;

// Uptime counter variable for heartbeat message
uint32_t test_uptimeSec = 0;

// buffer for serialization of heartbeat message
size_t hbeat_ser_buf_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
uint8_t hbeat_ser_buf[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

CanardPortID const MSG_PORT_ID = 1620U;

void setup()
{
  // Serial interface initialization
  Serial.begin(115200);

  // CAN interface initialization
  bool ret = CANInit(CAN_1000KBPS, 2);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  if (!ret) while(true);

  // UAVCAN initialization
  canard = canardInit(&memAllocate, &memFree);  // Initialization of a canard instance
  canard.node_id = 96;

  queue = canardTxInit( 100,                      // Limit the size of the queue at 100 frames.
                        CANARD_MTU_CAN_CLASSIC);  // Set MTU = 64 bytes. There is also CANARD_MTU_CAN_CLASSIC.

  CanardRxSubscription subscription; // Transfer subscription state.

  if( canardRxSubscribe((CanardInstance *const)&canard,
                        CanardTransferKindMessage,
                        MSG_PORT_ID,
                        uavcan_primitive_array_Real64_1_0_EXTENT_BYTES_,
                        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                        &subscription) != 1 )
                        {
                          Serial.println("Failed to subscribe");
                        }                
}

void loop() 
{
  // Create a heartbeat message
  uavcan_node_Heartbeat_1_0 test_heartbeat = {
    .uptime = test_uptimeSec,
    .health = {uavcan_node_Health_1_0_NOMINAL},
    .mode = {uavcan_node_Mode_1_0_OPERATIONAL}};

  // Serialize the heartbeat message
  if (uavcan_node_Heartbeat_1_0_serialize_(&test_heartbeat, hbeat_ser_buf, &hbeat_ser_buf_size) < 0)
  {
    Serial.println("Failed to serialize");
  }

  // Create a transfer for the heartbeat message
  const CanardTransferMetadata transfer_metadata  = {
    .priority = CanardPriorityNominal,
    .transfer_kind = CanardTransferKindMessage,
    .port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
    .remote_node_id = CANARD_NODE_ID_UNSET,
    .transfer_id = my_message_transfer_id,
  };

  if( canardTxPush( &queue,                 // Call this once per redundant CAN interface (queue)
                    &canard,
                    0,                      // Zero if transmission deadline is not limited.
                    &transfer_metadata,
                    hbeat_ser_buf_size,     // Size of the message payload (see Nunavut transpiler)
                    hbeat_ser_buf) < 0 )
                    {
                      Serial.println("Failed to add message into queue");
                    }

  uint32_t timestamp = millis();

  // Block for a second before generating the next transfer, spin transmission and receiption
  while (millis() < timestamp + 1000)
  {
    process_canard_TX_queue();
    process_canard_receiption();
  }

  // Increment the transfer_id variable
  my_message_transfer_id++;

  // Increase uptime
  test_uptimeSec++;
}

// allocate dynamic memory of desired size in bytes
static void *memAllocate(CanardInstance *const canard, const size_t amount)
{
  (void)canard;
  return malloc(amount);
}

// free allocated memory
static void memFree(CanardInstance *const canard, void *const pointer)
{
  (void)canard;
  free(pointer);
}

void process_canard_TX_queue(void)
{
  // Look at top of the TX queue of individual CAN frames
  for (const CanardTxQueueItem* ti = NULL; (ti = canardTxPeek(&queue)) != NULL;)
  {
    if ((0U == ti->tx_deadline_usec) || (ti->tx_deadline_usec > micros()))  // Check the deadline.
    {
      uint8_t send_ch = 1;
      CAN_msg_t CAN_TX_msg;
      
      CAN_TX_msg.len = ti->frame.payload_size;
      CAN_TX_msg.type = DATA_FRAME;
      CAN_TX_msg.format = EXTENDED_FORMAT;
      CAN_TX_msg.id = ti->frame.extended_can_id;

      memcpy(&CAN_TX_msg.data, (uint8_t *)ti->frame.payload, ti->frame.payload_size);
      
      CANSend(send_ch, &CAN_TX_msg);

      Serial.print(CAN_TX_msg.id, HEX);
      Serial.print("  [");
      Serial.print(CAN_TX_msg.len, DEC);
      Serial.print("]  ");      
      //Serial.print("CAN frame is sent: ");
      for( int i = 0; i < CAN_TX_msg.len; i++)
      {
        Serial.print(CAN_TX_msg.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    // After the frame is transmitted or if it has timed out while waiting, pop it from the queue and deallocate:
    canard.memory_free(&canard, canardTxPop(&queue, ti));
  }

  return ;
}

void process_canard_receiption(void)
{
  uint8_t recv_ch = 1;
  if(CANMsgAvail(recv_ch))
  {
    CAN_msg_t CAN_RX_msg;
    CANReceive(recv_ch, &CAN_RX_msg);

    CanardFrame rxf;
  
    rxf.extended_can_id = CAN_RX_msg.id;
    rxf.payload_size = (size_t)CAN_RX_msg.len;
    rxf.payload = (void*)&CAN_RX_msg.data;
  
    CanardRxTransfer transfer;

    CanardRxSubscription dummy; // canardRxAccept fails without this struct

    if( canardRxAccept( (CanardInstance *const)&canard,
              micros(),
              &rxf,
              0,
              &transfer,
              (CanardRxSubscription** const)&dummy) != 1 )
              {
                return ; // the frame received is not a valid transfer
              }

    uavcan_primitive_array_Real64_1_0 array;
    size_t array_ser_buf_size = uavcan_primitive_array_Real64_1_0_EXTENT_BYTES_;
  
    if ( uavcan_primitive_array_Real64_1_0_deserialize_( &array, (uint8_t*)transfer.payload, &array_ser_buf_size) < 0)
    {
      Serial.println("Failed to deserialize");
    }
  
    canard.memory_free(&canard, transfer.payload);

    Serial.println("Received Real64 array message: ");
    for( size_t i = 0; i < array.value.count; i++)
    {
      Serial.print((float)array.value.elements[i], 2);
      Serial.print(" ");
    }    
    Serial.println();
  }

  return ;
}