// Lab21_OPT3101_TestMain.c

//Cyber-Phys Autonomous Racing Challenge
// Team Groot

// Standard includes
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "msp.h"
#include "driverlib.h"
#include "simplelink.h"
#include "sl_common.h"
#include "MQTTClient.h"
#include "Motor.h"
#include "PWM.h"
#include "Bump.h"
#include "LaunchPad.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Tachometer.h"
#include "../inc/CortexM.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTick.h"
#include "../inc/I2CB1.h"
#include "../inc/opt3101.h"
#include "UART0.h"
#include "SSD1306.h"
#include "FFT.h"
#include "Reflectance.h"
#include "LPF.h"

// Select one of the following three output possibilities
// define USENOKIA
#define USEOLED 1
//#define USEUART

#ifdef USENOKIA
// this batch configures for LCD
#include "../inc/Nokia5110.h"
#define Init Nokia5110_Init
#define Clear Nokia5110_Clear
#define SetCursor Nokia5110_SetCursor
#define OutString Nokia5110_OutString
#define OutChar Nokia5110_OutChar
#define OutUDec Nokia5110_OutUDec
#define OutSDec Nokia5110_OutSDec
#endif

#ifdef USEOLED
// this batch configures for OLED

void OLEDinit(void){SSD1306_Init(SSD1306_SWITCHCAPVCC);}
#define Init OLEDinit
#define Clear SSD1306_Clear
#define SetCursor SSD1306_SetCursor
#define OutChar SSD1306_OutChar
#define OutString SSD1306_OutString
#define OutUDec SSD1306_OutUDec
#define OutSDec SSD1306_OutSDec
#endif

#ifdef USEUART
// this batch configures for UART link to PC
#include "UART0.h"
void UartSetCur(uint8_t newX, uint8_t newY){
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec
#define OutSDec UART0_OutSDec
#endif

uint16_t avg(uint16_t *array, int length)
{
  int i;
  uint32_t sum = 0;
  for(i=0; i<length; i=i+1){
    sum = sum + array[i];
  }
  return (sum/length);
}

uint32_t max(uint32_t a, uint32_t b){
    return a>b? a:b;
}

// Distance Sensor Stuff
void UartSetCur(uint8_t newX, uint8_t newY)
{
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}

uint32_t Distances[3];
long left0, center1, right2;
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // msec

bool pollDistanceSensor(void)
{
  if(OPT3101_CheckDistanceSensor())
  {
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}


// calibrated for 500mm track
// right is raw sensor data from right sensor
// return calibrated distance from center of Robot to right wall
int32_t Right(int32_t right){
        return  (right*(59*right + 7305) + 2348974)/32768;
}
// left is raw sensor data from left sensor
// return calibrated distance from center of Robot to left wall
int32_t Left(int32_t left){
        return (1247*left)/2048 + 22;
}

#define N 1024
uint32_t Data[N];
#define M 1024
uint16_t Histogram[M];
uint32_t Sum;      // sum of data
uint32_t Sum2;     // sum of (data-average)^2
uint32_t Average;  // average of data = sum/N
uint32_t Variance; // =sum2/(N-1)
uint32_t Sigma;    // standard deviation = sqrt(Variance)
uint8_t DataR;
int modeFlag = 0;
int bumpCount = 0;

// assumes track is 500mm
int32_t Mode = 1; // 0 stop, 1 run
int32_t Error, LeftError, RightError;
int32_t Ki=30;  // integral controller gain
int32_t Kd=20;
int32_t PrevError, PrevLeftError, PrevRightError;


float Kpc=7.5; //proportional controller gain //was 4
float Kpl=5;
float Kpr=4.6; // was 3.7
float Kdl=2.8;
float Kdr=2; // was 1
int32_t UR, UL;  // PWM duty 0 to 14,998


#define TOOCLOSE 200 //was 200
volatile int DESIRED_L=350; //was 250
volatile int DESIRED_R=350; //was 250
int32_t SetPoint = 250; // mm //was 250
int32_t LeftDistance,CenterDistance,RightDistance; // mm
#define TOOFAR 400 // was 400
#define RIGHT 1
#define LEFT 0
volatile int BIAS = RIGHT; //1= right, 0 = left


#define PWMNOMINAL 6000 // was 2500
#define SWING 1000 //was 1000
#define PWMMIN (PWMNOMINAL-SWING)
#define PWMMAX (PWMNOMINAL+SWING)

uint32_t RWallDist = 100;
uint32_t latch = 1;

void Controller_New(void){

    if(latch == 1){
        Motor_Forward(6000, 6000);

        if((BIAS == RIGHT) && (RightDistance < 100))latch = 0;
        else if((BIAS == LEFT) && (LeftDistance <  100)) latch = 0;
        return;
    }else{

    if((BIAS == RIGHT) && (RightDistance > max(480, DESIRED_R))) Motor_Forward(6000, 3500);
    else if((BIAS == LEFT) && (LeftDistance > max(480, DESIRED_L))) Motor_Forward(3500, 6000);


    else if(RightDistance > DESIRED_R || LeftDistance > DESIRED_L){
            DESIRED_L = (RightDistance + LeftDistance)*(4/5);
            DESIRED_R = (RightDistance + LeftDistance)/5;

    RightError = RightDistance - DESIRED_R;
    LeftError = LeftDistance - DESIRED_L;

    int32_t basePWM = Kpc * CenterDistance + 4000;
    int32_t leftProp = Kpl * (RightError);
    int32_t rightProp = Kpr * (LeftError);

    int32_t leftD = Kdl * (RightError - PrevRightError);
    int32_t rightD = Kdr * (LeftError - PrevLeftError);


    PrevLeftError = LeftError;
    PrevRightError = RightError;


    UL = basePWM + leftProp + leftD;
    UR = basePWM + rightProp + rightD;

    Motor_Forward(UL, UR);
    }
    }
}

void Pause(void){int i;
  uint8_t bumpVal = Bump_Read();

  for(i=5;i>0;i=i-1){
    Clock_Delay1ms(50);LaunchPad_Output(0); // off
    Clock_Delay1ms(50); LaunchPad_Output(2); // green
  }
  // restart Jacki
  UR = UL = PWMNOMINAL;    // reset parameters
  if (bumpVal <= 0x04)
      Motor_Backward(5000, 0);
  else if (bumpVal == 0x0C)
      Motor_Backward(5000, 5000);
  else
      Motor_Backward(0, 5000);
  Clock_Delay1ms(750);
  bumpCount++;
  Mode = 1;

}

// MSP432 memory limited to q=11, N=2048
#define q   8       /* for 2^8 points */
#define NN   (1<<q)  /* 256-point FFT, iFFT */
complex_t a[NN], scratch[NN];
uint32_t PlotOffset,PlotData;

/*
 * Values for below macros shall be modified per the access-point's (AP) properties
 * SimpleLink device will connect to following AP when the application is executed
 */
#define SSID_NAME       "Oday"       /* Access point name to connect to. */
#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2     /* Security type of the Access piont */
#define PASSKEY         "Oday1234"   /* Password in case of secure AP */
#define PASSKEY_LEN     pal_Strlen(PASSKEY)  /* Password length in case of secure AP */

/*
 * MQTT server and topic properties that shall be modified per application
 */
#define MQTT_BROKER_SERVER  "broker.hivemq.com"
#define SUBSCRIBE_TOPIC "odaySub/"
#define LRDCSUBSCRIBE_TOPIC "odayLRDCSub/"
#define LRPMPUBLISH_TOPIC "grootLSpeed/"
#define RRPMPUBLISH_TOPIC "grootRSpeed/"
#define LDPUBLISH_TOPIC "grootLWall/"
#define CDPUBLISH_TOPIC "grootCWall/"
#define RDPUBLISH_TOPIC "grootRWall/"
#define BMPPUBLISH_TOPIC "grootBump/"
#define GOPUBLISH_TOPIC "grootGo/"

// MQTT message buffer size
#define BUFF_SIZE 32

// IDs for Different MQTT Topics
#define Control 0
#define LDC 1
#define RDC 2
#define LD 3
#define CD 4
#define RD 5

int LDuC = 5000, RDuC = 5000;


#define APPLICATION_VERSION "1.0.0"

#define MCLK_FREQUENCY 48000000
#define PWM_PERIOD 255

#define SL_STOP_TIMEOUT        0xFF

#define SMALL_BUF           32
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   1024

/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

#define min(X,Y) ((X) < (Y) ? (X) : (Y))


/*
 * GLOBAL VARIABLES -- Start
 */
/* Button debounce state variables */
volatile unsigned int S1buttonDebounce = 0;
volatile unsigned int S2buttonDebounce = 0;
volatile int publishID = 0;

unsigned char macAddressVal[SL_MAC_ADDR_LEN];
unsigned char macAddressLen = SL_MAC_ADDR_LEN;

char macStr[18];        // Formatted MAC Address String
char uniqueID[9];       // Unique ID generated from TLV RAND NUM and MAC Address
int Dir = 0;

Network n;
Client hMQTTClient;     // MQTT Client

_u32  g_Status = 0;
struct{
    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
    _u8 SendBuff[MAX_SEND_BUF_SIZE];

    _u8 HostName[SMALL_BUF];
    _u8 CityName[SMALL_BUF];

    _u32 DestinationIP;
    _i16 SockID;
}g_AppData;

/* Port mapper configuration register */
const uint8_t port_mapping[] =
{
    //Port P2:
    PM_TA0CCR1A, PM_TA0CCR2A, PM_TA0CCR3A, PM_NONE, PM_TA1CCR1A, PM_NONE, PM_NONE, PM_NONE
};

/* TimerA UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_8,          // SMCLK/8 = 6MHz
        90000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

/*
 * GLOBAL VARIABLES -- End
 */


/*
 * STATIC FUNCTION DEFINITIONS -- Start
 */
static _i32 establishConnectionWithAP();
static _i32 configureSimpleLinkToDefaultState();
static _i32 initializeAppVariables();
static void displayBanner();
static void messageArrived(MessageData*);
static void messageArrivedLRDC(MessageData*);
static void generateUniqueID();


void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");

    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write(" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write(" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_Write(" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /*
             * Information about the connected AP's IP, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */
        }
        break;

        default:
        {
            CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    CLI_Write(" [HTTP EVENT] Unexpected event \n\r");
}

void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    CLI_Write(" [GENERAL EVENT] \n\r");
}

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
        CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");

    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        {
            switch( pSock->EventData.status )
            {
                case SL_ECLOSE:
                    CLI_Write(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
                break;


                default:
                    CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
                break;
            }
        }
        break;

        default:
            CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
        break;
    }
}
/*
 * ASYNCHRONOUS EVENT HANDLERS -- End
 */

// Tachometer Stuff
uint16_t ActualL;                        // actual rotations per minute measured by tachometer
uint16_t ActualR;                        // actual rotations per minute measured by tachometer
#define TACHBUFF 10
uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)

/*
 * Application's entry point
 */
int main(int argc, char** argv)
{
    int i = 0, j = 0, k = 0, g = 0;
    uint32_t channel = 1;
    BIAS = RIGHT;
    Clock_Init48MHz();
    SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
    SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts
    Bump_Init();
    Motor_Init();
    UART0_Init();
    Reflectance_Init();
    LaunchPad_Init(); // built-in switches
    Tachometer_Init();
    Motor_Stop(); // initialize and stop
    Mode = 1;
    I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
    Init();
    Clear();
    OutString("OPT3101");
    SetCursor(0, 1);
    OutString("L=");
    SetCursor(0, 2);
    OutString("C=");
    SetCursor(0, 3);
    OutString("R=");
    SetCursor(0, 4);
    OutString("Wall follow");
    SetCursor(0, 5);
    OutString("SP=");
    SetCursor(0, 6);
    OutString("Er=");
    SetCursor(0, 7);
    OutString("U =");
    OPT3101_Init();
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();
    //OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
    TxChannel = 3;
    OPT3101_StartMeasurementChannel(channel);
    LPF_Init(100,8);
    LPF_Init2(100,8);
    LPF_Init3(100,8);
    UR = UL = PWMNOMINAL; //initial power
    //Pause();
    EnableInterrupts();

    char command;
    char comm = 'S';
    Motor_Init();

    _i32 retVal = -1;

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    /* Stop WDT and initialize the system-clock of the MCU */
    stopWDT();
    initClk();


    /* Confinguring P1.1 & P1.4 as an input and enabling interrupts */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);


    /* Configuring TimerA1 for Up Mode */
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);

    Interrupt_enableInterrupt(INT_TA1_0);
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableMaster();

    /* Configure command line interface */
    CLI_Configure();

    displayBanner();

    retVal = configureSimpleLinkToDefaultState();
    if(retVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == retVal)
            CLI_Write(" Failed to configure the device in its default state \n\r");

        LOOP_FOREVER();
    }

    CLI_Write(" Device is configured in default state \n\r");

    /*
     * Assumption is that the device is configured in station mode already
     * and it is in its default state
     */
    retVal = sl_Start(0, 0, 0);
    if ((retVal < 0) ||
        (ROLE_STA != retVal) )
    {
        CLI_Write(" Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Device started as STATION \n\r");

    /* Connecting to WLAN AP */
    retVal = establishConnectionWithAP();
    if(retVal < 0)
    {
        CLI_Write(" Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Connection established w/ AP and IP is acquired \n\r");

    // Obtain MAC Address
    sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);

    // Print MAC Addres to be formatted string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            macAddressVal[0], macAddressVal[1], macAddressVal[2], macAddressVal[3], macAddressVal[4], macAddressVal[5]);

    // Generate 32bit unique ID from TLV Random Number and MAC Address
    generateUniqueID();

    int rc = 0;
    unsigned char buf[100];
    unsigned char readbuf[100];

    NewNetwork(&n);
    rc = ConnectNetwork(&n, MQTT_BROKER_SERVER, 1883);

    if (rc != 0) {
        CLI_Write(" Failed to connect to MQTT broker \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Connected to MQTT broker \n\r");

    MQTTClient(&hMQTTClient, &n, 1000, buf, 100, readbuf, 100);
    MQTTPacket_connectData cdata = MQTTPacket_connectData_initializer;
    cdata.MQTTVersion = 3;
    cdata.clientID.cstring = uniqueID;
    rc = MQTTConnect(&hMQTTClient, &cdata);

    if (rc != 0) {
        CLI_Write(" Failed to start MQTT client \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Started MQTT client successfully \n\r");

    rc = MQTTSubscribe(&hMQTTClient, SUBSCRIBE_TOPIC, QOS0, messageArrived);
    rc = MQTTSubscribe(&hMQTTClient, LRDCSUBSCRIBE_TOPIC, QOS0, messageArrivedLRDC);
    if (rc != 0) {
        CLI_Write(" Failed to subscribe to /msp/cc3100/demo topic \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Subscribed to /msp/cc3100/demo topic \n\r");

    rc = MQTTSubscribe(&hMQTTClient, uniqueID, QOS0, messageArrived);

    if (rc != 0) {
        CLI_Write(" Failed to subscribe to uniqueID topic \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Subscribed to uniqueID topic \n\r");
    LaunchPad_LED(0);

    while(1){
        LaunchPad_LED(1);
        rc = MQTTYield(&hMQTTClient, 10);
        if (rc != 0) {
            CLI_Write(" MQTT failed to yield \n\r");
            LOOP_FOREVER();
        }
        command = UART0_InChar();
        if(command == 'G' || command == 'S') comm = command;
        else if(command == 'L') BIAS = LEFT;
        else if(command == 'R') BIAS = RIGHT;

        if(Bump_Read()){ // collision
          Mode = 0;
          Motor_Stop();
          Pause();
        }

        DataR = Reflectance_Read(1000);
        if (DataR == 0x00 && modeFlag == 0)
            modeFlag = 1;
        if (DataR &=~ 0xFF != 0x00 && modeFlag == 1)
        {
            Motor_Stop();
            modeFlag = 0;

            char bufG[1];
            int m = 0;
            ltoa (m, bufG, 10);
            MQTTMessage msgG;
            msgG.dup = 0;
            msgG.id = 0;
            msgG.payload = bufG;
            msgG.payloadlen = 1;
            msgG.qos = QOS0;
            msgG.retained = 0;

            rc = MQTTPublish(&hMQTTClient, GOPUBLISH_TOPIC, &msgG);

            uint8_t pBmp;
            char bufBmp[3];
            if (bumpCount >= 10)
                pBmp = 2;
            else
                pBmp = 1;
            ltoa(bumpCount, bufBmp, 10);
            int rc = 0;
            MQTTMessage msgBmp;
            msgBmp.dup = 0;
            msgBmp.id = 0;
            msgBmp.payload = bufBmp;
            msgBmp.payloadlen = pBmp;
            msgBmp.qos = QOS0;
            msgBmp.retained = 0;

            rc = MQTTPublish(&hMQTTClient, BMPPUBLISH_TOPIC, &msgBmp);

            break;
        }

        if(comm == 'S') {Motor_Stop();}

        else if(comm == 'G'){


            if(g == 0){
                //Publish
                char bufG[1];
                int m = 1;
                ltoa (m, bufG, 10);
                MQTTMessage msgG;
                  msgG.dup = 0;
                  msgG.id = 0;
                  msgG.payload = bufG;
                  msgG.payloadlen = 1;
                  msgG.qos = QOS0;
                  msgG.retained = 0;

                  // Sending Tachometer Values
                  rc = MQTTPublish(&hMQTTClient, GOPUBLISH_TOPIC, &msgG);
                g++;
            }


            if(TxChannel <= 2){ // 0,1,2 means new data
              if(TxChannel==0){
                if(Amplitudes[0] > 1000){
                  LeftDistance = FilteredDistances[0] = Left(LPF_Calc(Distances[0]));
                }else{
                  LeftDistance = FilteredDistances[0] = 500;
                }
              }else if(TxChannel==1){
                if(Amplitudes[1] > 1000){
                  CenterDistance = FilteredDistances[1] = LPF_Calc2(Distances[1]);
                }else{
                  CenterDistance = FilteredDistances[1] = 500;
                }
              }else {
                if(Amplitudes[2] > 1000){
                  RightDistance = FilteredDistances[2] = Right(LPF_Calc3(Distances[2]));
                }else{
                  RightDistance = FilteredDistances[2] = 500;
                }
              }
              SetCursor(2, TxChannel+1);
              OutUDec(FilteredDistances[TxChannel]); OutChar(','); OutUDec(Amplitudes[TxChannel]);
              TxChannel = 3; // 3 means no data
              channel = (channel+1)%3;
              OPT3101_StartMeasurementChannel(channel);
              j = j + 1;
            }
            Controller_New();
            if(j >= 100){
              j = 0;
              SetCursor(3, 5);
              OutUDec(SetPoint);
              SetCursor(3, 6);
              OutSDec(Error);
              SetCursor(3, 7);
              OutUDec(UL); OutChar(','); OutUDec(UR);
            }
            WaitForInterrupt();
          }
        //Use hardware timers to capture wheel encoder outputs
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);

        //Buffer loop index
        i = i + 1;

        //Take the running average of the past TACHBUFF # of samples
        //Check to see if the Tachometer buffer is full
         if(i >= TACHBUFF)
         {
           //Reset the buffer index
           i = 0;

           //Compute the Average Revolutions Per Minute over the most recent  TACHBUFF # of Samples
           // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
           ActualL = 2000000/avg(LeftTach, TACHBUFF);
           ActualR = 2000000/avg(RightTach, TACHBUFF);

         }

         if(pollDistanceSensor())
         {
           TimeToConvert = ((StartTime-SysTick->VAL)&0x00FFFFFF)/48000; // msec
           if(TxChannel <= 2)
           {
             SetCursor(6, TxChannel+1);

             left0 = Distances[0];
             center1 = Distances[1];
             right2 = Distances[2];
             if (left0 > 2000)
                 left0 = 2000;
             if (center1 > 2000)
                 center1 = 2000;
             if (right2 > 2000)
                 right2 = 2000;

             OutUDec(Distances[TxChannel]);
           }
           channel = (channel+1)%3;
           OPT3101_StartMeasurementChannel(channel);
           StartTime = SysTick->VAL;
         }
         if (k == 30)
         {
             uint8_t payloadL, payloadR, pLD, pCD, pRD;
             char bufL[3];
             char bufR[3];
             if (ActualL >= 100)
                 payloadL = 3;
             else if (ActualL >= 10 && ActualL <100)
                 payloadL = 2;
             else
                 payloadL = 1;

             if (ActualR >= 100)
                  payloadR = 3;
              else if (ActualR >= 10 && ActualR <100)
                  payloadR = 2;
              else
                  payloadR = 1;

             ltoa(ActualL, bufL, 10);
             ltoa(ActualR, bufR, 10);

             char bufLD[3];
             char bufCD[3];
             char bufRD[3];

              if (left0 >= 1000)
                  pLD = 4;
              else if (left0 >= 100 && left0 < 1000)
                  pLD = 3;
              else if (left0 >= 10 && left0 < 100)
                  pLD = 2;
              else
                  pLD = 1;

              if (center1 >= 1000)
                  pCD = 4;
              else if (center1 >= 100 && center1 < 1000)
                  pCD = 3;
              else if (center1 >= 10 && center1 < 100)
                  pCD = 2;
              else
                  pCD = 1;

              if (right2 >= 1000)
                  pRD = 4;
              else if (right2 >= 100 && right2 < 1000)
                  pRD = 3;
              else if (right2 >= 10 && right2 < 100)
                  pRD = 2;
              else
                  pRD = 1;

             ltoa(left0, bufLD, 10);
             ltoa(center1, bufCD, 10);
             ltoa(right2, bufRD, 10);

             int rc = 0;
             MQTTMessage msgL;
             msgL.dup = 0;
             msgL.id = 0;
             msgL.payload = bufL;
             msgL.payloadlen = payloadL;
             msgL.qos = QOS0;
             msgL.retained = 0;

             MQTTMessage msgR;
              msgR.dup = 0;
              msgR.id = 0;
              msgR.payload = bufR;
              msgR.payloadlen = payloadR;
              msgR.qos = QOS0;
              msgR.retained = 0;

              MQTTMessage msgLD;
              msgLD.dup = 0;
              msgLD.id = 0;
              msgLD.payload = bufLD;
              msgLD.payloadlen = pLD;
              msgLD.qos = QOS0;
              msgLD.retained = 0;

              MQTTMessage msgCD;
              msgCD.dup = 0;
              msgCD.id = 0;
              msgCD.payload = bufCD;
              msgCD.payloadlen = pCD;
              msgCD.qos = QOS0;
              msgCD.retained = 0;

              MQTTMessage msgRD;
              msgRD.dup = 0;
              msgRD.id = 0;
              msgRD.payload = bufRD;
              msgRD.payloadlen = pRD;
              msgRD.qos = QOS0;
              msgRD.retained = 0;

              // Sending Tachometer Values
              rc = MQTTPublish(&hMQTTClient, LRPMPUBLISH_TOPIC, &msgL);
              rc = MQTTPublish(&hMQTTClient, RRPMPUBLISH_TOPIC, &msgR);
              // Clock_Delay1ms(100);

              // Sending Distance Sensor Values
              rc = MQTTPublish(&hMQTTClient, LDPUBLISH_TOPIC, &msgLD);
              rc = MQTTPublish(&hMQTTClient, CDPUBLISH_TOPIC, &msgCD);
              rc = MQTTPublish(&hMQTTClient, RDPUBLISH_TOPIC, &msgRD);
              // Clock_Delay1ms(100);
              k = 0;
         }
         else
             k++;
        //Delay(10);
    }
}

static void generateUniqueID() {
    CRC32_setSeed(TLV->RANDOM_NUM_1, CRC32_MODE);
    CRC32_set32BitData(TLV->RANDOM_NUM_2);
    CRC32_set32BitData(TLV->RANDOM_NUM_3);
    CRC32_set32BitData(TLV->RANDOM_NUM_4);
    int i;
    for (i = 0; i < 6; i++)
    CRC32_set8BitData(macAddressVal[i], CRC32_MODE);

    uint32_t crcResult = CRC32_getResult(CRC32_MODE);
    sprintf(uniqueID, "%06X", crcResult);
}

//****************************************************************************
//
//!    \brief MQTT message received callback - Called when a subscribed topic
//!                                            receives a message.
//! \param[in]                  data is the data passed to the callback
//!
//! \return                        None
//
//****************************************************************************
static void messageArrived(MessageData* data) {
    char buf[BUFF_SIZE];

    // Check for buffer overflow
    if (data->topicName->lenstring.len >= BUFF_SIZE) {
//      UART_PRINT("Topic name too long!\n\r");
        return;
    }
    if (data->message->payloadlen >= BUFF_SIZE) {
//      UART_PRINT("Payload too long!\n\r");
        return;
    }

    strncpy(buf, data->topicName->lenstring.data,
        min(BUFF_SIZE, data->topicName->lenstring.len));
    buf[data->topicName->lenstring.len] = 0;



    strncpy(buf, data->message->payload,
        min(BUFF_SIZE, data->message->payloadlen));
    buf[data->message->payloadlen] = 0;

    if (strcmp(buf, "GO") == 0)
    {
        Motor_Forward(LDuC, RDuC);
        Dir = 1;
    }
    if (strcmp(buf, "SLOW") == 0)
    {
        int tLDC = LDuC, tRDC = RDuC;
        while (tLDC > 0 && tRDC > 0)
        {
            if (Dir == 1)
                Motor_Forward(tLDC, tRDC);
            else if(Dir == 2)
                Motor_Right(tLDC, tRDC);
            else if(Dir == 3)
                Motor_Left(tLDC, tRDC);
            else
                Motor_Stop();
            tLDC -= 100;
            tRDC -= 100;
            Clock_Delay1ms(50);
        }
        Motor_Stop();
        Dir = 0;
    }
    if (strcmp(buf, "STOP") == 0)
    {
        Motor_Stop();
        Dir = 0;
    }
    if (strcmp(buf, "RIGHT") == 0)
    {
        Motor_Right(LDuC, RDuC);
        Dir = 2;
    }
    if (strcmp(buf, "LEFT") == 0)
    {
        Motor_Left(LDuC, RDuC);
        Dir = 3;
    }

    return;
}

//
static void messageArrivedLRDC(MessageData* data) {
    char buf[BUFF_SIZE];
    float ldc;
    float rdc;
    // Check for buffer overflow
    if (data->topicName->lenstring.len >= BUFF_SIZE) {
//      UART_PRINT("Topic name too long!\n\r");
        return;
    }
    if (data->message->payloadlen >= BUFF_SIZE) {
//      UART_PRINT("Payload too long!\n\r");
        return;
    }

    strncpy(buf, data->topicName->lenstring.data,
        min(BUFF_SIZE, data->topicName->lenstring.len));
    buf[data->topicName->lenstring.len] = 0;



    strncpy(buf, data->message->payload,
        min(BUFF_SIZE, data->message->payloadlen));
    buf[data->message->payloadlen] = 0;

    ldc = atoi(buf);
    rdc = atoi(buf);
    LDuC = ldc/100 * 15000;
    RDuC = rdc/100 * 15000;

    switch(Dir)
        {
        case 1:
            Motor_Forward(LDuC, RDuC);
            break;
        case 2:
            Motor_Right(LDuC, RDuC);
            break;
        case 3:
            Motor_Left(LDuC, RDuC);
            break;
        }

    return;
}

void PORT1_IRQHandler(void)
{
    uint32_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (status & GPIO_PIN1)
    {
        if (S1buttonDebounce == 0)
        {
            S1buttonDebounce = 1;

            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

            // Publish the unique ID
            publishID = 1;

            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
    if (status & GPIO_PIN4)
    {
        if (S2buttonDebounce == 0)
        {
            S2buttonDebounce = 1;

            CLI_Write(" MAC Address: \n\r ");
            CLI_Write(macStr);
            CLI_Write("\n\r");

            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
}

void TA1_0_IRQHandler(void)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    if (P1IN & GPIO_PIN1)
    {
        S1buttonDebounce = 0;
    }
    if (P1IN & GPIO_PIN4)
    {
        S2buttonDebounce = 0;
    }

    if ((P1IN & GPIO_PIN1) && (P1IN & GPIO_PIN4))
    {
        Timer_A_stopTimer(TIMER_A1_BASE);
    }
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


/*!
    \brief This function configure the SimpleLink device in its default state. It:
           - Sets the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregisters mDNS services
           - Remove all filters

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
*/
static _i32 configureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;

    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}

/*!
    \brief Connecting to a WLAN Access point

    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address

    \param[in]  None

    \return     0 on success, negative error-code on error

    \note

    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
*/
static _i32 establishConnectionWithAP()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = PASSKEY_LEN;
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

/*!
    \brief This function initializes the application variables

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
static _i32 initializeAppVariables()
{
    g_Status = 0;
    pal_Memset(&g_AppData, 0, sizeof(g_AppData));

    return SUCCESS;
}

/*!
    \brief This function displays the application's banner

    \param      None

    \return     None
*/
static void displayBanner()
{
    CLI_Write("\n\r\n\r");
    CLI_Write(" MQTT STUFF ");
    CLI_Write(APPLICATION_VERSION);
    CLI_Write("\n\r*******************************************************************************\n\r");
}
