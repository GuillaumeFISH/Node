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
#define M_PI           3.141592  /* pi */
#define PPS_PIN        PC_2      /*PA1*/
#define NODEID         6
#define DELAY          600     //[ms]
#define US_CORRECTION  33      //Correcting us timer error with offset
/**********************************************************************/

//Variable to help keep track if a transmission is over
volatile bool txDone;

//Define sensor variables
float temp1, temp2, humid1, humid2;
char buffer1[32], buffer2[32], buffer3[32], buffer4[32];
int32_t axes1[3], axes2[3], axes3[3], axes4[3];

//Define timing variables
int int_time=0;         //GPS epoch time
int set_rtc = 0;        //Counter to sync the uc RTC to GPS a number of times defined in GPS_data()
int received_time;      //Holds a timestamp of us between seconds
Timer t;                //us timer to achieve greater resolution between seconds, reset and logged every PPS
time_t whattime;        //Used to timestamp uc RTC

//Configure GPS
Serial * gps_Serial = new Serial(D1,D0); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here

// Defines the queues to assign execution priority
EventQueue Read_Sensors_Queue;
EventQueue GPS_data_Queue;

//Configure PPS pin
InterruptIn PPS(PA_1, PullNone);

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGRAccSensor *accelerometer = mems_expansion_board->accelerometer;

// DM_to_DD
// Converts the input from decimal minutes format to decimal degrees
// 
// Inputs:
//     DM = Geodetic lat/long in decimal minutes format ie. 30deg 30.18min
// 
// Returns:
//     Decimal degree representation of DM, ie. 30.503 deg
double DM_to_DD(double DM){
    double minutes, degrees;

    DM /= 100;
    minutes = std::modf(DM, &degrees);
    minutes *= 100;

    return (degrees + minutes/60);
}

// Converts standard time into Epoch time.
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


// Reads and stores sensor data
void Read_Sensors() {
  printf("Got to read_sensors\r\n\r\n");
  hum_temp->get_temperature(&temp1);
  hum_temp->get_humidity(&humid1);
  press_temp->get_temperature(&temp2);
  press_temp->get_pressure(&humid2);
  magnetometer->get_m_axes(axes1);
  accelerometer->get_x_axes(axes2);
  acc_gyro->get_x_axes(axes3);
  acc_gyro->get_g_axes(axes4);
}


//Sends a message incluing time and positioning data
void Send_transmission() {

    //Check if gps has a fix, otherwise fill gps data with hardcoded stuff for testing
    /* Commented out for outdoor testing
    if (!myGPS.fix) {
        //Hardcoded gps data for testing.
        myGPS.latitude = 5052.03; //loc 1: 5051.93 loc 2: 5048.59 loc 3: 5052.03 loc 5: 5054.31
        myGPS.longitude = 10644.94; //loc 1: 10631.26 loc 2: 10634.48 loc 3: 10644.94 loc 5: 10639.18
        myGPS.lat = 'N';
        myGPS.lon = 'W';
        myGPS.altitude = 1120;

        //Manually setting whattime for testing purposes
        whattime = 3699225; //loc 1: 1647806 loc 2: 211146 loc 3: 3699225 loc 5: 2054754

        printf("Location: %5.2f%c, %5.2f%c\r\n", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
        printf("Altitude: %5.2f\r\n", myGPS.altitude);
    }
    */

    //Converting to a non float format for transmission
    unsigned int uint_latitude = (int)(myGPS.latitude * 100);       //Holds node latitude
    unsigned int uint_longitude = (int)(myGPS.longitude * 100);     //Holds node longitude

    //Encoding scheme for hemishere indicator (to avoid sending chars and save 4 bits)
    int8_t int8_latlon = -1; //-1 = error
    if(myGPS.lat == 'N'){
        if(myGPS.lon == 'E')
            int8_latlon = 0; //N-E
        else 
            int8_latlon = 1; //N-W
    }
    else if(myGPS.lon == 'E')
        int8_latlon = 2; //S-E

    else
        int8_latlon = 3; //S-W

    //Hardcoded device identifier
    unsigned int uint_deviceid = NODEID;

    //Building the payload
    //Microcontroller time
    Radio::radio.tx_buf[0] = (int)((whattime >> 24) & 0xFF) ; 
    Radio::radio.tx_buf[1] = (int)((whattime >> 16) & 0xFF) ;
    Radio::radio.tx_buf[2] = (int)((whattime >> 8) & 0XFF);
    Radio::radio.tx_buf[3] = (int)((whattime & 0XFF));

    //Latitude
    Radio::radio.tx_buf[4] = (uint_latitude >> 24) & 0xFF;
    Radio::radio.tx_buf[5] = (uint_latitude >> 16) & 0xFF;
    Radio::radio.tx_buf[6] = (uint_latitude >> 8) & 0xFF;
    Radio::radio.tx_buf[7] =  uint_latitude & 0xFF;

    //Longitude
    Radio::radio.tx_buf[8] = (uint_longitude >> 24) & 0xFF;
    Radio::radio.tx_buf[9] = (uint_longitude >> 16) & 0xFF;
    Radio::radio.tx_buf[10] =(uint_longitude >> 8) & 0xFF;
    Radio::radio.tx_buf[11] = uint_longitude & 0xFF;
    
    //Device ID
    Radio::radio.tx_buf[12] = uint_deviceid & 0xFF;

    //Microsecond timer timestamp from when query was received
    Radio::radio.tx_buf[13] = (received_time >> 16) & 0xFF;
    Radio::radio.tx_buf[14] = (received_time >> 8) & 0xFF;
    Radio::radio.tx_buf[15] = received_time & 0xFF;

    //Lat/lon
    Radio::radio.tx_buf[16] = int8_latlon & 0xFF;

    txDone = false;
    //The first parameter indicates the size of the payload in bytes, dont forget this.
    Radio::Send(17, 0, 0, 0);   /* begin transmission */
    printf("Packet sent\r\n");
    while (!txDone) {
        Radio::service();
    }

    printf("Done servicing\r\n");
    printf("-------END OF CYCLE-------\r\n\r\n");
    Radio::Rx(0);
}

//Collects and parses GPS data
void GPS_data() {
    int d = t.read_us();
    t.reset(); //reset us timer every second according to PPS
    printf("Got to gps_data\r\n");
    do{
        c = myGPS.read();   //queries the GPS
        //if (c) { pc.printf("%c", c); } //this line will echo the GPS data if not paused
        //check if we recieved a new message from GPS, if so, attempt to parse it,
        if (myGPS.newNMEAreceived() == true)
        {
            if (myGPS.parse(myGPS.lastNMEA()) == true)
            {
                int_time = asUnixTime(myGPS.year+2000, myGPS.month, myGPS.day, myGPS.hour, myGPS.minute, myGPS.seconds);  
                //Will update uC RTC first 4 times GPS fetches data to ensure uC RTC is correct
                if (set_rtc < 4){
                     set_time(int_time);
                    set_rtc++;
                }              
                break;
            }
        }
    } while(myGPS.newNMEAreceived() == false);

    int_time = asUnixTime(myGPS.year+2000, myGPS.month, myGPS.day, myGPS.hour, myGPS.minute, myGPS.seconds);  
}

//Callback for when a transmission is complete
void txDoneCB()
{
    txDone = true;
}

//Callback for when a message has finished receiving
void rxDoneCB(uint8_t size, float rssi, float snr)
{
    //Time that message was received
    received_time = t.read_us() + US_CORRECTION;
    whattime = time(NULL);

    //If query has correct key, send back positioning data
    if(Radio::radio.rx_buf[0] == 0xAB){
        printf("\r\n-------START OF CYCLE------\r\n");
        printf("Received Query: %0X\r\n", Radio::radio.rx_buf[0]);
        wait_ms(DELAY);
        Send_transmission();
    }
    Radio::Rx(0);
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
    //GPS Settings
    myGPS.sendCommand(PMTK_STANDBY);
    wait(1);
    myGPS.sendCommand(PMTK_AWAKE);
    wait(1);
    myGPS.sendCommand(PMTK_SET_BAUD_57600);
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
    myGPS.sendCommand(PGCMD_NOANTENNA);

    //Intialize tranceiver settings
    Radio::Init(&rev);
    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, SPREADING_FACTOR, CODE_RATE);
    Radio::SetChannel(CF_HZ);
    Radio::set_tx_dbm(TX_DBM);
               // preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(8, false, true, false);
    printf("Radio Initialized\r\n");

    /* resets and starts the us timer */
    t.reset();
    t.start();
  
    /* Enable all sensors */
    hum_temp->enable();
    press_temp->enable();
    magnetometer->enable();
    accelerometer->enable();
    acc_gyro->enable_x();
    acc_gyro->enable_g();
    wait(1.5);

    //High priority thread for GPS_data()
    Thread GPS_data_Thread(osPriorityHigh);
    GPS_data_Thread.start(callback(&GPS_data_Queue, &EventQueue::dispatch_forever));
    
    // normal priority thread for Read_Sensors()
    Thread Read_Sensors_Thread(osPriorityNormal);
    Read_Sensors_Thread.start(callback(&Read_Sensors_Queue, &EventQueue::dispatch_forever));
    
    // call read_sensors 1 every second, automatically defering to the eventThread
    //Tickers are only used for testing in the absence of PPS
    Ticker GPSTicker;
    Ticker ReadTicker;
    GPSTicker.attach(GPS_data_Queue.event(&GPS_data), 1.0f);
    //ReadTicker.attach(Read_Sensors_Queue.event(&Read_Sensors), 1.0f);

    //Attaches PPS rise and fall events to GPS and Sensor data fetching threads
    //PPS.rise(GPSQueue.event(&GPS_data));
    //PPS.fall(eventQueue.event(&Read_Sensors));

    Radio::Rx(0);
    //Service interrupts
    for (;;) { 
        Radio::service();
    }
    wait(osWaitForever);
}
