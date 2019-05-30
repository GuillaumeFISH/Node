/*
What Node does as of May 29 2019
1. Mode is set to receive indefinetely
2. Once a transmission is received, message is printed out
3. Sends out a transmission which includes gps information
4. Mode is set to receive indefinetely after successful transmission (in Send_transmission)
*/
#include "radio.h"
#include "XNucleoIKS01A2.h"
#include "MBed_Adafruit_GPS.h"

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

#define CODE_RATE 1

/**********************************************************************/

//Sensors
float temp1, temp2, humid1, humid2;
char buffer1[32], buffer2[32], buffer3[32], buffer4[32];
int32_t axes1[3], axes2[3], axes3[3], axes4[3];

//Timing
int64_t usTime1 = 0, usTime2 = 0, usDeltaTime = 0;
int int_time=0;
bool set_rtc = true;

//Payload
uint8_t Buffer[32];
uint8_t id;

//Configure GPS
Serial * gps_Serial = new Serial(D1,D0); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here

// Defines the queues
EventQueue TransmitQueue;
EventQueue eventQueue;
EventQueue GPSQueue;

/* Defines the timer */
Timer t;
time_t whattime;


/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGRAccSensor *accelerometer = mems_expansion_board->accelerometer;


volatile bool txDone;

/* Converts standard time into Epoch time. Could delete this if no longer needed.*/
time_t asUnixTime(int year, int mon, int mday, int hour, int min, int sec) {
    struct tm   t;
    t.tm_year = year - 1900;
    t.tm_mon =  mon - 1;        // convert to 0 based month
    t.tm_mday = mday;
    t.tm_hour = hour;
    t.tm_min = min;
    t.tm_sec = sec;
    t.tm_isdst = -1;            // Is Daylight saving time on? 1 = yes, 0 = no, -1 = unknown
 
    return mktime(&t);          // returns seconds elapsed since January 1, 1970 (begin of the Epoch)
}


/* Reads the sensor board sensors */
/* Reads the current board time */
/* Compares the current time to the last time it was measured */
void Read_Sensors() {
  // this runs in the normal priority thread
  hum_temp->get_temperature(&temp1);
  hum_temp->get_humidity(&humid1);
  press_temp->get_temperature(&temp2);
  press_temp->get_pressure(&humid2);
  magnetometer->get_m_axes(axes1);
  accelerometer->get_x_axes(axes2);
  acc_gyro->get_x_axes(axes3);
  acc_gyro->get_g_axes(axes4);
}


//Currently using to send radio transmission. May 24
// this runs in the lower priority thread
void Send_transmission() {

    //Building the payload
    //Microcontroller time
    Radio::radio.tx_buf[0] = (int)((whattime >> 24) & 0xFF) ; 
    Radio::radio.tx_buf[1] = (int)((whattime >> 16) & 0xFF) ;
    Radio::radio.tx_buf[2] = (int)((whattime >> 8) & 0XFF);
    Radio::radio.tx_buf[3] = (int)((whattime & 0XFF));

    //Some sensor data for fun
    uint16_t u16_temp1 = (uint16_t)(temp1*100);
    Radio::radio.tx_buf[4] = (u16_temp1 >> 8) & 0xFF ; 
    Radio::radio.tx_buf[5] = u16_temp1 & 0xFF ;

    /*If you have a fix you can fill more of the buffer with location data*/
    //myGPS.fix is commented out for testing, perhaps not even needed
    //If the if statement is used, you must redefine the payload length as it is currently hardcoded
    //if (myGPS.fix) {
        //Hardcoded gps data for testing.
        myGPS.latitude = 9000.00;
        myGPS.longitude = 18000.00;
        myGPS.lat = 'N';
        myGPS.lon = 'E';
        myGPS.altitude = 99999.99;
        printf("Location: %5.2f%c, %5.2f%c\r\n", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
        printf("Altitude: %5.2f\r\n", myGPS.altitude);

        //Converting to a non float format for transmission
        unsigned int uint_latitude = (int)(myGPS.latitude * 100);
        unsigned int uint_longitude = (int)(myGPS.longitude * 100);
        unsigned int uint_altitude = (int)(myGPS.altitude * 100);

        //Encoding scheme for hemishere indicator (to avoid sending chars and save 4 bits)
        unsigned int uint_latlon = -1; //-1 = error
        if(myGPS.lat == 'S'){
            if(myGPS.lon == 'E')
                uint_latlon = 0; //N-E
            else 
                uint_latlon = 1; //N-W
        }
        else if(myGPS.lon == 'E')
                uint_latlon = 2; //S-E

        else
            uint_latlon = 3; //S-W

        //All 20 bits of latitude and first 12 of longitude
        unsigned int uint_positioningtop = (uint_latitude << 12) | (uint_longitude >> 12);
        //Last 12 bits of longitude and first 20 of altitude
        unsigned int uint_positioningmid = (uint_longitude << 20) | (uint_altitude >> 4);
        //Last 4 bits of altitude and the 4 for lat/long hemisphere
        unsigned int uint_positioningbot = ((uint_altitude << 4) & 0xF0) | (uint_latlon);

        Radio::radio.tx_buf[6] = (uint_positioningtop >> 24) & 0xFF;
        Radio::radio.tx_buf[7] = (uint_positioningtop >> 16) & 0xFF;
        Radio::radio.tx_buf[8] = (uint_positioningtop >> 8) & 0xFF;
        Radio::radio.tx_buf[9] = uint_positioningtop & 0xFF;

        Radio::radio.tx_buf[10] = (uint_positioningmid >> 24) & 0xFF;
        Radio::radio.tx_buf[11] = (uint_positioningmid >> 16) & 0xFF;
        Radio::radio.tx_buf[12] = (uint_positioningmid >> 8) & 0xFF;
        Radio::radio.tx_buf[13] = uint_positioningmid & 0xFF;

        Radio::radio.tx_buf[14] = uint_positioningbot & 0xFF;
    //}
    unsigned int uint_deviceid = 1917;
    Radio::radio.tx_buf[15] = uint_deviceid & 0xFF;


    txDone = false;
    //The first parameter indicates the size of the payload in bytes, dont forget this.
    Radio::Send(16, 0, 0, 0);   /* begin transmission */
    printf("Packet sent\r\n");
    while (!txDone) {
        Radio::service();
    }

    printf("Done servicing\r\n");
    printf("-------END OF CYCLE-------\r\n\r\n");
    Radio::Rx(0);
}

//Collects and parses GPS data
//This runs in the high priority thread
void GPS_data() {
    int received = 0;
    do{
        c = myGPS.read();   //queries the GPS
        //if (c) { pc.printf("%c", c); } //this line will echo the GPS data if not paused
        
        //check if we recieved a new message from GPS, if so, attempt to parse it,
        if ( myGPS.newNMEAreceived() ) {
            if ( !myGPS.parse(myGPS.lastNMEA()) ) {
                continue;
            }
            else
                received++;    
        }
    } while(received != 2);

    int_time = asUnixTime(myGPS.year+2000, myGPS.month, myGPS.day, myGPS.hour, myGPS.minute, myGPS.seconds);  
    
    //set rtc when called for the first time (sync to gps on startup) 
    if (set_rtc){
        set_time(int_time);
        set_rtc = false;
    } 
    //Updating the time just because this is the high priority thread
    usTime2 = usTime1;
    usTime1 = t.read_high_resolution_us();
    usDeltaTime = usTime1 - usTime2;
    whattime = time(NULL); 
}

void txDoneCB()
{
    txDone = true;
}

void rxDoneCB(uint8_t size, float rssi, float snr)
{
    printf("\r\n-------START OF CYCLE------\r\n");
    printf("Received Query: %0X\r\n", Radio::radio.rx_buf[0]);
    Send_transmission();
    //Radio::Rx(0); //Temporary while Send_transmission is commented out
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
    myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                    //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf
    
    myGPS.sendCommand(PMTK_AWAKE);
    printf("Wake Up GPS...\r\n");

    wait(1);
    //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);

    //Intialize transmitter settings
    Radio::Init(&rev);
    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, SPREADING_FACTOR, CODE_RATE);
    Radio::SetChannel(CF_HZ);
    Radio::set_tx_dbm(TX_DBM);
               // preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(8, false, true, false);

    printf("Radio Initialized\r\n");

    /* resets and starts the timer */
    t.reset();
    t.start();
    usTime1 = t.read_high_resolution_us();
  
    /* Enable all sensors */
    hum_temp->enable();
    press_temp->enable();
    magnetometer->enable();
    accelerometer->enable();
    acc_gyro->enable_x();
    acc_gyro->enable_g();
    wait(1.5);

    //High priority thread for gps
    Thread gpsThread(osPriorityHigh);
    gpsThread.start(callback(&GPSQueue, &EventQueue::dispatch_forever));
    
    // normal priority thread for other events
    Thread eventThread(osPriorityNormal);
    eventThread.start(callback(&eventQueue, &EventQueue::dispatch_forever));
  
    // low priority thread for calling Send_transmission()
    Thread TransmitThread(osPriorityLow);
    TransmitThread.start(callback(&TransmitQueue, &EventQueue::dispatch_forever));
    
    // call read_sensors 1 every second, automatically defering to the eventThread
    Ticker GPSTicker;
    Ticker ReadTicker;
    Ticker TransmitTicker;

    GPSTicker.attach(GPSQueue.event(&GPS_data), 1.0f);
    ReadTicker.attach(eventQueue.event(&Read_Sensors), 3.0f);
    //Print ticker currently triggers the radio transmission.
    //TransmitTicker.attach(TransmitQueue.event(&Send_transmission), 3.0f);

    Radio::Rx(0);
    //Service interrupts
    for (;;) { 
        Radio::service();
    }
    //Should never reach this point
    printf("Bad news");
    wait(osWaitForever);
}

