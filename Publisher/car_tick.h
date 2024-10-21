#ifndef _CAR_TICK_H
#define _CAR_TICK_H

/* Type definitions */
typedef unsigned char   UINT8;
typedef int             INT32;
typedef size_t          UINT64;
typedef char            CHAR8;
typedef unsigned char   UCHAR8;
typedef long long       TIME64;

/* Configurable data structure */
typedef struct car_tick_conf_data{
    CHAR8 *serial_device;       /* Serial device connected to OBD2 */
    CHAR8 *mqtt_broker_ip;      /* IP of the broker */
    INT32 mqtt_broker_port;     /* Port of the broker (1883 standard, 8883 for TLS) */
    CHAR8 *car_model;           /* Full name of the car model */
} Conf_Data;

/* For debug purposes */
#define DBG_LOG_INT(x,y)    fflush(stdin); printf("DEBUG::%s::%d\n", x, y);
#define DBG_LOG_CHAR(x,y)   fflush(stdin); printf("DEBUG::%s::%s\n", x, y);

/* Error codes */
#define CAR_ERR_SUCCESS 0
#define CAR_ERR_FAILURE -1

/* Program variables */
#define CLIENT_ID       "car_data_pulisher"
#define SLEEP_TIME      0.20
#define TRANSMIT_DELAY  600   
#define DELAY_S         1
#define DELAY_MAX       30
#define DELAY_EXP       true

/* PIDs definition */
#define PID_ENG_LOAD    "0104"
#define PID_RPM         "010C"
#define PID_SPEED       "010D"
#define PID_RUN_TIME    "011F"
#define PID_CMD_EGR     "012C"
#define PID_AMB_TEMP    "0146"
#define PID_ERR_CODES   "03"

/* Processing options */
#define OPT_ENG_LOAD    0
#define OPT_RPM         1
#define OPT_SPEED       2
#define OPT_RUN_TIME    3
#define OPT_CMD_EGR     4
#define OPT_AMB_TEMP    5
#define OPT_ERR_CODES   6

/* Request frequencys in milliseconds */
#define PID_FREQ_1000   950
#define PID_FREQ_5000   5000

#define START_DATA_INDEX_STD        8  /* 8 If ECHO is enabled in ELM327, if not set to 4 */
#define START_DATA_INDEX_ERR        4  /* 8 If ECHO is enabled in ELM327, if not set to 4 */
#define START_DATA_INDEX_NON_STD    12 /* 12 If ECHO is enabled in ELM327, if not set to 6 */

/* MQTT Options */
#define QOS_0   0
#define QOS_1   1
#define QOS_2   2

#endif  /* _CAR_TICK_H */