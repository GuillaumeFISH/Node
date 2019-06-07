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
#define M_PI           3.141592  /* pi */
/**********************************************************************/

//Sensors
float temp1, temp2, humid1, humid2;
char buffer1[32], buffer2[32], buffer3[32], buffer4[32];
int32_t axes1[3], axes2[3], axes3[3], axes4[3];

//Timing
int64_t usTime1 = 0;
int int_time=0;
bool set_rtc = true;
int received_time;

//Payload
uint8_t Buffer[32];
uint8_t id;

//Configure GPS
Serial * gps_Serial = new Serial(D1,D0); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here

// Defines the queues
EventQueue eventQueue;
EventQueue GPSQueue;

/* Defines the timer */
Timer t;
time_t whattime;

//Configure PPS
InterruptIn PPS(A1, PullNone);

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGRAccSensor *accelerometer = mems_expansion_board->accelerometer;


volatile bool txDone;

class Coord {
    const double a = 6378137.0;              //WGS-84 semi-major axis
    const double e2 = 6.6943799901377997e-3;  //WGS-84 first eccentricity squared
    const double a1 = 4.2697672707157535e+4;  //a1 = a*e2
    const double a2 = 1.8230912546075455e+9;  //a2 = a1*a1
    const double a3 = 1.4291722289812413e+2;  //a3 = a1*e2/2
    const double a4 = 4.5577281365188637e+9;  //a4 = 2.5*a2
    const double a5 = 4.2840589930055659e+4;  //a5 = a1+a3
    const double a6 = 9.9330562000986220e-1;  //a6 = 1-e2
    double zp,w2,w,r2,r,s2,c2,s,c,ss;
    double g,rg,rf,u,v,m,f,p,x,y,z;
    double n,lat,lon,alt;
    
    public:
    //Convert Earth-Centered-Earth-Fixed (ECEF) to lat, Lon, Altitude
    //Input is a three element array containing x, y, z in meters
    //Returned array contains lat and lon in radians, and altitude in meters
        double * ecef_to_geo( double *ecef ){
            static double geo[3];   //Results go here (Lat, Lon, Altitude)
            x = ecef[0];
            y = ecef[1];
            z = ecef[2];
            zp = std::fabs( z );
            w2 = x*x + y*y;
            w = std::sqrt( w2 );
            r2 = w2 + z*z;
            r = std::sqrt( r2 );
            geo[1] = std::atan2( y, x );       //Lon (final)
            s2 = z*z/r2;
            c2 = w2/r2;
            u = a2/r;
            v = a3 - a4/r;
            if( c2 > 0.3 ){
                s = ( zp/r )*( 1.0 + c2*( a1 + u + s2*v )/r );
                geo[0] = std::asin( s );      //Lat
                ss = s*s;
                c = std::sqrt( 1.0 - ss );
            }
            else{
                c = ( w/r )*( 1.0 - s2*( a5 - u - c2*v )/r );
                geo[0] = std::acos( c );      //Lat
                ss = 1.0 - c*c;
                s = std::sqrt( ss );
            }
            g = 1.0 - e2*ss;
            rg = a/std::sqrt( g );
            rf = a6*rg;
            u = w - rg*c;
            v = zp - rf*s;
            f = c*u + s*v;
            m = c*v - s*u;
            p = m/( rf/g + f );
            geo[0] = geo[0] + p;      //Lat
            geo[2] = f + m*p/2.0;     //Altitude
            if( z < 0.0 ){
                geo[0] *= -1.0;     //Lat
            }
            return( geo );    //Return Lat, Lon, Altitude in that order
        }
        
        //Convert Lat, Lon, Altitude to Earth-Centered-Earth-Fixed (ECEF)
        //Input is a three element array containing lat, lon (rads) and alt (m)
        //Returned array contains x, y, z in meters
        double * geo_to_ecef( double *geo ) {
            static double ecef[3];  //Results go here (x, y, z)
            lat = geo[0]*M_PI/180;
            lon = geo[1]*M_PI/180;
            alt = geo[2]*M_PI/180;
            n = a/std::sqrt( 1 - e2*std::sin( lat )*std::sin( lat ) );
            ecef[0] = ( n + alt )*std::cos( lat )*std::cos( lon );    //ECEF x
            ecef[1] = ( n + alt )*std::cos( lat )*std::sin( lon );    //ECEF y
            ecef[2] = ( n*(1 - e2 ) + alt )*std::sin( lat );          //ECEF z
            return( ecef );     //Return x, y, z in ECEF
        }
}coords;

//Converts from decimal minutes format to decimal degrees
double DM_to_DD(double DM){
    double minutes, degrees;

    DM /= 100;
    minutes = std::modf(DM, &degrees);
    minutes *= 100;

    return (degrees + minutes/60);
}

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


// this runs in the lower priority thread
void Send_transmission() {

    //Building the payload
    //Microcontroller time
    whattime = time(NULL);
    Radio::radio.tx_buf[0] = (int)((whattime >> 24) & 0xFF) ; 
    Radio::radio.tx_buf[1] = (int)((whattime >> 16) & 0xFF) ;
    Radio::radio.tx_buf[2] = (int)((whattime >> 8) & 0XFF);
    Radio::radio.tx_buf[3] = (int)((whattime & 0XFF));

    //Check if gps has a fix, otherwise fill gps data with hardcoded stuff for testing
    if (!myGPS.fix) {
        //Hardcoded gps data for testing.
        myGPS.latitude = 9000.00;
        myGPS.longitude = 18000.00;
        myGPS.lat = 'N';
        myGPS.lon = 'E';
        myGPS.altitude = 99999.99;
        printf("Location: %5.2f%c, %5.2f%c\r\n", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
        printf("Altitude: %5.2f\r\n", myGPS.altitude);
    }

    //Converting to a non float format for transmission
    unsigned int uint_latitude = (int)(myGPS.latitude * 100);
    unsigned int uint_longitude = (int)(myGPS.longitude * 100);
    unsigned int uint_altitude = (int)(myGPS.altitude * 100);

    //Encoding scheme for hemishere indicator (to avoid sending chars and save 4 bits)
    unsigned int uint_latlon = -1; //-1 = error
    if(myGPS.lat == 'N'){
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

    Radio::radio.tx_buf[4] = (uint_positioningtop >> 24) & 0xFF;
    Radio::radio.tx_buf[5] = (uint_positioningtop >> 16) & 0xFF;
    Radio::radio.tx_buf[6] = (uint_positioningtop >> 8) & 0xFF;
    Radio::radio.tx_buf[7] = uint_positioningtop & 0xFF;

    Radio::radio.tx_buf[8] = (uint_positioningmid >> 24) & 0xFF;
    Radio::radio.tx_buf[9] = (uint_positioningmid >> 16) & 0xFF;
    Radio::radio.tx_buf[10] = (uint_positioningmid >> 8) & 0xFF;
    Radio::radio.tx_buf[11] = uint_positioningmid & 0xFF;

    Radio::radio.tx_buf[12] = uint_positioningbot & 0xFF;
    
    unsigned int uint_deviceid = 55;
    Radio::radio.tx_buf[13] = uint_deviceid & 0xFF;

    Radio::radio.tx_buf[14] = (received_time >> 16) & 0xFF;
    Radio::radio.tx_buf[15] = (received_time >> 8) & 0xFF;
    Radio::radio.tx_buf[16] = received_time & 0xFF;

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
//This runs in the high priority thread
void GPS_data() {
    t.reset(); //reset us timer every second
    int received = 0;
    do{
        c = myGPS.read();   //queries the GPS
        //if (c) { printf("%c", c); } //this line will echo the GPS data if not paused

        //check if we recieved a new message from GPS, if so, attempt to parse it,
        //ie. Got one whole sentence
        if ( myGPS.newNMEAreceived() ) {
            //Parsed the message succesfully (had all info present)
            if ( !myGPS.parse(myGPS.lastNMEA()) ) {
                continue;
            }
            //Incomplete sentence
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
}

void txDoneCB()
{
    txDone = true;
}

void rxDoneCB(uint8_t size, float rssi, float snr)
{
    //Time that can be sent back to gateway for TDOA analysis
    received_time = t.read_us();
    if(Radio::radio.rx_buf[0] == 0xAB){
        printf("\r\n-------START OF CYCLE------\r\n");
        printf("Received Query: %0X\r\n", Radio::radio.rx_buf[0]);
        wait_ms(5000);
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
   

    myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                    //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf
    
    //myGPS.sendCommand(PMTK_SET_BAUD_57600);
    myGPS.sendCommand(PMTK_AWAKE);
    printf("Wake Up GPS...\r\n");

    //Geodetic to cartesian testing
    /*
    double geodetic[3] = {0.67874351881,-1.34475873537,130.049};
    double * cartesian = coords.geo_to_ecef(geodetic);
    printf("%lf %lf %lf\r\n", cartesian[0], cartesian[1], cartesian[2]);
    printf("%lf\r\n", DM_to_DD(4916.45));
    printf("%lf\r\n", DM_to_DD(12311.12));
    double * geodeticagain = coords.ecef_to_geo(cartesian);
    printf("%lf %lf %lf\r\n", geodeticagain[0], geodeticagain[1], geodeticagain[2]);
    */
    wait(1);
    //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_NOANTENNA);

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
    
    // call read_sensors 1 every second, automatically defering to the eventThread
    Ticker GPSTicker;
    Ticker ReadTicker;

    GPSTicker.attach(GPSQueue.event(&GPS_data), 1.0f);
    ReadTicker.attach(eventQueue.event(&Read_Sensors), 1.0f);

    //PPS.rise(GPSQueue.event(&GPS_data));
    //PPS.fall(eventQueue.event(&Read_Sensors));

    Radio::Rx(0);
    //Service interrupts
    for (;;) { 
        Radio::service();
    }
    wait(osWaitForever);
}
