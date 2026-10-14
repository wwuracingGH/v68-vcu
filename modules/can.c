/**************************************************************************************
 * Viking v68 VCU can.c
 *
 * Nicole Swierstra, Jackson Newman
 * 10/13/2026
 *
 * Function:
 * 		Holds CAN init and other various CAN related programs.
 ***************************************************************************************/
#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "can.h"
#include "stm32h5xx.h"
#include "vendor/FreeRTOS-Kernel/include/queue.h"
#include "vendor/FreeRTOS-Kernel/include/semphr.h"
#include <stdbool.h>
#define CAN_RX_BUFFER_LEN 256 // X
#define CAN_TX_LOCK_WAIT pdMS_TO_TICKS(2) //?

/* FreeRTOS */
static QueueHandle_t rx_queue[CAN_NUM_BUSES];   /* Two Queue Handles */
static SemaphoreHandle_t tx_mutex[CAN_NUM_BUSES];
static volatile uint32_t rx_dropped[CAN_NUM_BUSES];
static bool startup = true;

CAN_Message_t rx_buffer[CAN_RX_BUFFER_LEN];

/* Index of last CAN message put */
uint16_t rx_put_head = 0;
/* Index of where the last message read is */
uint16_t rx_get_head = 0;


/* TODO: implement this perhaps */
#define CAN_TX_BUFFER_LEN 32
CAN_Message_t tx_buffer[CAN_TX_BUFFER_LEN];

/* Index of last CAN message put */
uint16_t tx_put_head = 0;
/* Index of where the last message read is */
uint16_t tx_get_head = 0;

void CAN_init() {
    /* RTOS */
    /* If startup create all RTOS objects*/
    if (startup) {
        for (int i = 0; i < CAN_NUM_BUSES; i++) {
            rx_queue[i] = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(CAN_Message_t));
            tx_mutex[i] = xSemaphoreCreateMutex();
            configASSERT(rx_queue[i] != NULL && tx_mutex[i] != NULL);
        }
        startup = false;
    }

    /* Enable FDCAN clock */
    RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
    __DSB(); /* data sync -> wait for mem update stage in pipeline */

    /* clear sleep bit */
    FDCAN1->CCCR &= ~(FDCAN_CCCR_CSR);
    while (FDCAN1->CCCR & FDCAN_CCCR_CSA);

    /* Enter initialization mode */
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0) {
        /* Wait for INIT to set */
    }

    /* Enable the Configuration Change Enable (CCE) bit */
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;
    while ((FDCAN1->CCCR & FDCAN_CCCR_CCE) == 0);  /* wait for cce */

    /* TODO: get rid of this */
    FDCAN1->CCCR |= FDCAN_CCCR_DAR; /* disable automatic retransmission */

    /* clock in is at 125mhz (set in clock config), there are 5 divs and the smp is 75% */
    FDCAN1->NBTP = (CAN1_BAUD_PRES << FDCAN_NBTP_NBRP_Pos) |
                   (2 << FDCAN_NBTP_NTSEG1_Pos) |
                   (0 << FDCAN_NBTP_NTSEG2_Pos) |
                   (0 << FDCAN_NBTP_NSJW_Pos);

    /* Configure Global Acceptance Filtering to set blank filter */
    FDCAN1->RXGFC |= (0 << FDCAN_RXGFC_ANFS_Pos) |
                     (0 << FDCAN_RXGFC_ANFE_Pos) |
                     FDCAN_RXGFC_F0OM | FDCAN_RXGFC_F1OM;

    FDCAN1->TXBC &= ~(1 << FDCAN_TXBC_TFQM_Pos); /* clear queue bit */

    /* clear sleep bit */
    FDCAN2->CCCR &= ~(FDCAN_CCCR_CSR);
    while (FDCAN2->CCCR & FDCAN_CCCR_CSA);

    /* Enter initialization mode */
    FDCAN2->CCCR |= FDCAN_CCCR_INIT;
    while ((FDCAN2->CCCR & FDCAN_CCCR_INIT) == 0);

    /* Enable the Configuration Change Enable (CCE) bit */
    FDCAN2->CCCR |= FDCAN_CCCR_CCE;
    while ((FDCAN2->CCCR & FDCAN_CCCR_CCE) == 0);  /* wait for cce */

    /* TODO: get rid of this */
    FDCAN2->CCCR |= FDCAN_CCCR_DAR; /* disable automatic retransmission */

    /* clock in is at 125mhz (set in clock config), there are 4 divs and the first seg is 2 long */
    FDCAN2->NBTP = (24 << FDCAN_NBTP_NBRP_Pos) |
                   (2 << FDCAN_NBTP_NTSEG1_Pos) |
                   (0 << FDCAN_NBTP_NTSEG2_Pos) |
                   (0 << FDCAN_NBTP_NSJW_Pos);

    /* Configure Global Acceptance Filtering to set blank filter */
    FDCAN2->RXGFC |= (0 << FDCAN_RXGFC_ANFS_Pos) |
                     (0 << FDCAN_RXGFC_ANFE_Pos) |
                     FDCAN_RXGFC_F0OM | FDCAN_RXGFC_F1OM;

    FDCAN2->TXBC &= ~(1 << FDCAN_TXBC_TFQM_Pos); /* clear queue bit */

    /* setup interrupts */
    FDCAN1->IE |= FDCAN_IE_RF0NE | FDCAN_IE_RF1NE; 
    FDCAN2->IE |= FDCAN_IE_RF0NE | FDCAN_IE_RF1NE;
    FDCAN1->ILS |= FDCAN_ILS_RXFIFO1;
    FDCAN2->ILS |= FDCAN_ILS_RXFIFO1;
    FDCAN1->ILE |= 3;
    FDCAN2->ILE |= 3;

    /* Set interrupt priorities CAN1>CAN2*/
    NVIC_SetPriority(FDCAN1_IT0_IRQn, CAN1_IRQ_PRIORITY);
    NVIC_SetPriority(FDCAN1_IT1_IRQn, CAN1_IRQ_PRIORITY);
    NVIC_SetPriority(FDCAN2_IT0_IRQn, CAN2_IRQ_PRIORITY);
    NVIC_SetPriority(FDCAN2_IT1_IRQn, CAN2_IRQ_PRIORITY);

    /* Enable interrupts */
    NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
    NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
    
    /* Enable Operations*/
    FDCAN1->CCCR |= FDCAN_CCCR_FDOE;
    FDCAN2->CCCR |= FDCAN_CCCR_FDOE;

    /* Exit Initialize Mode */
    FDCAN1->CCCR &= ~(FDCAN_CCCR_INIT);
    while (FDCAN1->CCCR & FDCAN_CCCR_INIT) {
        /* Wait */
    }

    /* Exit Initialize Mode */
    FDCAN2->CCCR &= ~(FDCAN_CCCR_INIT);
    while (FDCAN2->CCCR & FDCAN_CCCR_INIT) {
        /* Wait */
    }
}

void CAN_reset() {
    /* Disable interrupts */
    NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
    NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    NVIC_DisableIRQ(FDCAN2_IT1_IRQn);

    RCC->APB1HENR &= ~RCC_APB1HENR_FDCANEN;
    __DSB();
    RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
    __DSB();
    RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;
    __DSB();

    CAN_init();
}

void memcpy_32(uint32_t* dest, uint32_t* src, uint32_t l){
    uint32_t * d = dest;
    uint32_t * s = src;
    
    while (l > 0){
        *d = *s;
        d++;
        s++;
        l--;
    }
}

uint32_t _get_id(FDCAN_Rx_FIFO_Element_Typedef* msg) {
    return (msg->H0 & FDCAN_RXBH0_XTD_Msk) ? (msg->H0 & FDCAN_RXBH0_EXTID_Msk) :
            (msg->H0 & FDCAN_RXBH0_STDID_Msk) >> FDCAN_RXBH0_STDID_Pos;
}

uint8_t _get_dlc(FDCAN_Rx_FIFO_Element_Typedef* msg) {
    return (msg->H1 & FDCAN_RXBH1_DLC_Msk) >> FDCAN_RXBH1_DLC_Pos;
}

uint32_t* _get_data(FDCAN_Rx_FIFO_Element_Typedef* msg) {
    return msg->data;
}

int CAN_rxCount() {
    int l = rx_put_head - rx_get_head;
    if (l < 0) l += CAN_RX_BUFFER_LEN;
    return l;
}

int CAN_bytesFromDLC(int dlc) {
    const int bytes[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
    return bytes[dlc]; 
}
int CAN_DLCFromBytes(int bytes) {
    if (bytes <= 8) return bytes;
    if (bytes <= 24) return (bytes + 27) >> 2;
    return (bytes + 191) >> 4;
}

/* FreeRTOS */
void CAN_sendmessage(FDCAN_GlobalTypeDef* FDCAN, uint16_t id, uint8_t length, uint8_t* data) {
    SemaphoreHandle_t lock; /* Mutex Selector */

    /* Select which mutex */
    if (FDCAN == FDCAN1) {
        lock = tx_mutex[CAN1_BUS];
    } else if (FDCAN ==FDCAN2) {
        lock = tx_mutex[CAN2_BUS];
    } else {
        return;
    }

    /* 
     * Take one of the mutexs if failed to acquire bail 
     * xSemaphoreTake outputs a BaseType_t of pdTRUE or pdFALSE
     * which equates to mutex acquired or failed to acquire mutex
     */
    if (xSemaphoreTake(lock, CAN_TX_LOCK_WAIT) != pdTRUE) return;
    
    /*
     * FDCAN->TXFQS & FDCAN_TXFQS_TFQF_Msk:
     * Tx FIFO/QUEUE Status Reg the mask looks at only the FIFO/QUEUE FULL
     * bit. Waits for the queue to not be full then exits.
     * If time out, release mutex and return.
     */
    int timeout = 1600000;
    while ((FDCAN->TXFQS & FDCAN_TXFQS_TFQF_Msk) && timeout) timeout--;
    if (FDCAN->TXFQS & FDCAN_TXFQS_TFQF_Msk) {
        xSemaphoreGive(lock);
        return;
    }
    /* first open buffer slot */
    int j = (FDCAN->TXFQS & FDCAN_TXFQS_TFQPI_Msk) >> FDCAN_TXFQS_TFQPI_Pos;

    /* Find the indexed buffer */
    FDCAN_SRAM_Typedef * fd_ram = (FDCAN == FDCAN1) ? FDCAN1_RAM : FDCAN2_RAM;
    FDCAN_Tx_FIFO_Element_Typedef *msg = &fd_ram->tx_buffers[j];

    msg->H0 = 0; /* 11 bit ID goes to T0 */
    msg->H0 |= ((uint32_t)id << FDCAN_TXBH0_STDID_Pos); /* STDID */

    msg->H1  = 0;
    msg->H1 |= ((uint32_t)length << FDCAN_TXBH1_DLC_Pos); /* DLC */

    msg->data[0] = *((uint32_t*)data);
    msg->data[1] = *(((uint32_t*)data) + 1);

    FDCAN->TXBAR |= (1UL << j); /* Trigger message by writing to TX buffer */
    xSemaphoreGive(lock);   /* Release mutex */
}

static void CAN_receiveMessage(uint32_t id, uint16_t bus,uint16_t len, uint32_t* data, BaseType_t* xHigherPriorityTaskWoken) {
    /* Create a CAN message */
    CAN_Message_t msg = {id, len, bus, {data[0], data[1]}};

    /* 
     * Sends the message to its respective queue, sets xHigher... to true if
     * the task it interrupted had lower priority and returns if it succeeded or...
     */
    if (xQueueSendFromISR(rx_queue[bus], &msg, xHigherPriorityTaskWoken) != pdTRUE) rx_dropped[bus]++;
}

/*******************************************************************************
 * static void CAN_FIFO_Process(FDCAN_GlobalTypeDef* FDCAN, CAN_Bus_t bus...
 *
 * 
 * 
 *******************************************************************************/
static void CAN_FIFO_Process(FDCAN_GlobalTypeDef* FDCAN, CAN_Bus_t bus, int fifo, BaseType_t* xHigherPriorityTaskWoken) {
    /*
     * Find where the bus's recieved frames are stored. Each CAN peripheral has
     * its own block of message in RAM, (ID, Len, data), Bus 0 = CAN1, Bus
     * 1 = CAN2, Additionally there are two FIFO blocks (0 and 1) that represent 
     * the two mail boxes for each CAN bus. We must know the bus and which FIFO block.
     */
    FDCAN_SRAM_Typedef* fd_ram = (bus == CAN1_BUS) ? FDCAN1_RAM : FDCAN2_RAM;

    /*
     * Select the status register of FDCAN RX FIFO x.
     * Bits [3:0] fill level or number of elements stored
     * Bits [9:8] index pointer of oldest frame to read
     * Bits [16-17] index to next frame to fill/ write
     */
    volatile uint32_t* status = fifo ? &FDCAN->RXF1S : &FDCAN->RXF0S;
    volatile uint32_t* ack    = fifo ? &FDCAN->RXF1A : &FDCAN->RXF0A;
    FDCAN_Rx_FIFO_Element_Typedef* elements = fifo ? fd_ram->rx_fifo1 : fd_ram->rx_fifo0;

    /* RXF0S and RXF1S share a layout, so the FIFO0 masks work for both */
    while (*status & FDCAN_RXF0S_F0FL_Msk) {
        uint32_t idx = (*status & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
        FDCAN_Rx_FIFO_Element_Typedef* msg = elements + idx;
        CAN_receiveMessage(_get_id(msg), bus, CAN_bytesFromDLC(_get_dlc(msg)), _get_data(msg), xHigherPriorityTaskWoken);
        *ack = idx;
    }

}

/*******************************************************************************
 * fdcanX_itY_handler()
 * 
 * Activates when the FIFO interrupt triggers. Sends it to CAN_FIFO_Process...
 * 
 *******************************************************************************/
void fdcan1_it0_handler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    FDCAN1->IR = FDCAN_IR_RF0N;    /* Clear new message */
    CAN_FIFO_Process(FDCAN1, CAN1_BUS, 0, &xHigherPriorityTaskWoken);
    /* If the CAN_rxTask has higher PRIO than what it interrupted, switch*/
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void fdcan1_it1_handler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    FDCAN1->IR = FDCAN_IR_RF1N;    /* Clear new message */
    CAN_FIFO_Process(FDCAN1, CAN1_BUS, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void fdcan2_it0_handler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    FDCAN2->IR = FDCAN_IR_RF0N;    /* Clear new message */
    CAN_FIFO_Process(FDCAN2, CAN2_BUS, 0, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void fdcan2_it1_handler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    FDCAN2->IR = FDCAN_IR_RF1N;    /* Clear new message */
    CAN_FIFO_Process(FDCAN2, CAN2_BUS,1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool CAN_getMessage(uint16_t bus, CAN_Message_t* msg, TickType_t wait) {
    return xQueueReceive(rx_queue[bus], msg, wait) == pdTRUE;
}
/*******************************************************************************
 * fdcanX_itY_handler()
 *
 * Activates when the FIFO interrupt triggers. Sends it to CAN_FIFO_Process...
 *
 *******************************************************************************/
uint32_t CAN_rxDropped(uint16_t bus) {
    return rx_dropped[bus];
}
