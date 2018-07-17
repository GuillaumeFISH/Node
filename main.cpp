#include "radio.h"

#if defined(SX127x_H)
    #define BW_KHZ              125
    #define SPREADING_FACTOR    7
    #define CF_HZ               915000000
    #define TX_DBM              20
#elif defined(SX126x_H)
    #define BW_KHZ              125
    #define SPREADING_FACTOR    7
    #define CF_HZ               915000000
    #define TX_DBM              (Radio::chipType == CHIP_TYPE_SX1262 ? 20 : 14) 
#elif defined(SX128x_H)
    #define BW_KHZ              200
    #define SPREADING_FACTOR    7
    #define CF_HZ               2487000000
    #define TX_DBM              6
#endif

/**********************************************************************/
volatile bool txDone;

void txDoneCB()
{
    txDone = true;
}

void rxDoneCB(uint8_t size, float rssi, float snr)
{
}

const RadioEvents_t rev = {
    /* Dio0_top_half */     NULL,
    /* TxDone_topHalf */    NULL,
    /* TxDone_botHalf */    txDoneCB,
    /* TxTimeout  */        NULL,
    /* RxDone  */           rxDoneCB,
    /* RxTimeout  */        NULL,
    /* RxError  */          NULL,
    /* FhssChangeChannel  */NULL,
    /* CadDone  */          NULL
};

int main()
{
    uint8_t seq = 0;

    printf("\r\nreset-tx ");

    Radio::Init(&rev);

    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, SPREADING_FACTOR, 1);
    Radio::SetChannel(CF_HZ);

    Radio::set_tx_dbm(TX_DBM);

               // preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(8, false, true, false);

    for (;;) {
        Radio::radio.tx_buf[0] = seq;  /* set payload */
        txDone = false;
        Radio::Send(1, 0, 0, 0);   /* begin transmission */

        printf("sent\r\n");
        while (!txDone) {
            Radio::service();
        }

        printf("got-tx-done\r\n");

        wait(0.5);  /* throttle sending rate */
        seq++;  /* change payload */
    }
}

