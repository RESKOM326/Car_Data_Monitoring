#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif  /* _GNU_SOURCE */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <ctype.h>
#include <time.h>

#include "car_tick.h"
#include <mosquitto.h>

/* Function prototypes */

/*
*  Detect signal SIGINT (Ctrl-C) to stop the program safely
*  PARAMETERS: None
*  RETURNS: Nothing, this call is always successful
*/
void SIGINT_Handler(INT32);

/*
*  Get configurable data from car_tick.conf file
*  PARAMETERS: Pointer to Conf_Data struct with memory already allocated
*  RETURNS: CAR_ERR_SUCCESS if the given Conf_Data has been filled. CAR_ERR_FAILURE otherwise
*/
INT32 Get_Conf_Data(Conf_Data*);

/*
*  Initialize serial data port to request and read OBD2 data from car (OBD2 -> USB Serial)
*  PARAMETERS: None
*  RETURNS: CAR_ERR_SUCCESS on success, CAR_ERR_FAILURE on any failure
*/
INT32 Init_Serial_Read(void);

/*
*  Initialize mosquitto instance and establish connection with MQTT Broker
*  PARAMETERS: None
*  RETURNS: CAR_ERR_SUCCESS on success, CAR_ERR_FAILURE on any failure
*/
INT32 Init_MQTT_Client(void);

/*
*  Sends a data request to the OBD2
*  PARAMETERS: PID request (SERVICE_MODE + PID)
*  RETURNS: On success, total number of bytes written (including CR). CAR_ERR_FAILURE otherwise
*/
INT32 Send_PID_Request(const CHAR8*);

/*
*  Reads the OBD2 response to the last request
*  PARAMETERS: ->Buffer to store read data. ->Size of the expected response. ->Response timeout in seconds
*  RETURNS: On success, number of bytes read. CAR_ERR_FAILURE otherwise
*/
INT32 Read_OBD2_Response(CHAR8*, UINT64, INT32);

/*
*  Parses the OBD2 response (ASCII buffer representing bytes) into a readable value and publishes it into MQTT
*  PARAMETERS: ->PID to process. ->OBD2 response received. ->QoS of transmission
*  RETURNS: CAR_ERR_SUCCESS on success, CAR_ERR_FAILURE on any failure
*/
INT32 Parse_Publish_OBD2_Response(INT32, CHAR8*, INT32);

/*
*  Removes spaces from an ASCII buffer
*  PARAMETERS: ->Source buffer. ->Processed buffer
*  RETURNS: Nothing
*/
void Remove_Spaces(const CHAR8*, CHAR8*);

/*
*  Free all resources used by serial port reading and mosquitto client
*  PARAMETERS: None
*  RETURNS: Nothing, this call is always successful
*/
void Clean_UP(void);

/* Global variables */
struct mosquitto    *mosq;          /* Mosquitto instance */
INT32               obd2_fd;        /* File descriptor of USB serial port */
Conf_Data           *conf_data;     /* Structure of the configurable data */
struct timespec     cur_time;       /* Struct to store current time */
TIME64              t_current_time; /* Current time */
TIME64              t_timemark_1s;  /* Time mark for 1s frequency publish */
TIME64              t_timemark_5s;  /* Time mark for 5s frequency publish */

/* Used to stop the program with Ctrl-C, cleaning up everything */
static volatile int keepRunning = 1;

void SIGINT_Handler(__attribute__((unused)) INT32 dummy) {
    printf("CAR_TICK > Stopping program...\n");
    keepRunning = 0;
}

INT32 Get_Conf_Data(Conf_Data *conf_data)
{
    FILE *conf;
    CHAR8 *line = NULL;
    size_t n_line = 0;
    ssize_t n_read = 0;

    const CHAR8 delim[1] = ":";

    conf = fopen("car_tick.conf", "r");
    if(conf == NULL)
    {
        perror("CAR_TICK > open_conf_data: Unable to open file");
        return CAR_ERR_FAILURE;
    }

    while((n_read = getline(&line, &n_line, conf)) != -1)
    {
        if(line[0] == '#') continue;
        CHAR8 *token = strtok(line, delim);
        /* The line feed character is read, so it must be deleted */
        if(strcmp("SERIAL_DEVICE", token) == 0)
        {
            token = strtok(NULL, delim);
            if(token[strlen(token)-1] == '\n') token[(strlen(token)-1)] = '\0';
            conf_data->serial_device = strdup(token);
        }
        if(strcmp("MQTT_BROKER_IP", token) == 0)
        {
            token = strtok(NULL, delim);
            if(token[strlen(token)-1] == '\n') token[(strlen(token)-1)] = '\0';
            conf_data->mqtt_broker_ip = strdup(token);
        }
        if(strcmp("MQTT_BROKER_PORT", token) == 0)
        {
            token = strtok(NULL, delim);
            if(token[strlen(token)-1] == '\n') token[(strlen(token)-1)] = '\0';
            conf_data->mqtt_broker_port = atoi(token);
        }
        if(strcmp("CAR_MODEL", token) == 0)
        {
            token = strtok(NULL, delim);
            if(token[strlen(token)-1] == '\n') token[(strlen(token)-1)] = '\0';
            conf_data->car_model = strdup(token);
        }
    }
    return CAR_ERR_SUCCESS;
}

INT32 Init_Serial_Read()
{
    struct termios options;

    INT32 ret_value = CAR_ERR_SUCCESS;

    /* Open Serial Port in non-blocking mode with R/W permissions */
    obd2_fd = open(conf_data->serial_device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (obd2_fd == -1) 
    {
        perror("CAR_TICK > open_port: Unable to open serial port");
        return CAR_ERR_FAILURE;
    }
    /* Configure serial port file descriptor in non-blocking mode
       This is done so the call to write/read does not return immediately
       if no data is available at the exact moment of the call */
    fcntl(obd2_fd, F_SETFL, 0);

    /* Configure port parameters */
    ret_value = tcgetattr(obd2_fd, &options);
    if(ret_value != CAR_ERR_SUCCESS)
    {
        perror("CAR_TICK > configure_port: Unable to get attributes");
        return CAR_ERR_FAILURE;
    }
    ret_value = cfsetispeed(&options, B115200);   /* Baud rate for reading */
    if(ret_value != CAR_ERR_SUCCESS)
    {
        perror("CAR_TICK > configure_port: Unable to configure input baud rate");
        return CAR_ERR_FAILURE;
    }
    ret_value = cfsetospeed(&options, B115200);   /* Baud rate for writing */
    if(ret_value != CAR_ERR_SUCCESS)
    {
        perror("CAR_TICK > configure_port: Unable to configure output baud rate");
        return CAR_ERR_FAILURE;
    }
    options.c_cflag |= (CLOCAL | CREAD);        /* Enable port control and reading */
    options.c_cflag &= ~PARENB;                 /* No parity */
    options.c_cflag &= ~CSTOPB;                 /* 1 bit STOP */
    options.c_cflag &= ~CSIZE;                  /* Clean data size */
    options.c_cflag |= CS8;                     /* 8 bit data */
    options.c_cflag &= CRTSCTS;                 /* Enable HW flow control if available */
    options.c_iflag &= ~(IXON | IXOFF | IXANY); /* Disable SW flow control to avoid extra bytes in raw mode */

    /* Configure serial port for raw mode operation */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Disable canonical entry and echo */
    options.c_oflag &= ~OPOST;                          /* Disable output processing */

    /* Apply configuration */
    ret_value = tcsetattr(obd2_fd, TCSANOW, &options);
    if(ret_value != CAR_ERR_SUCCESS)
    {
        perror("CAR_TICK > configure_port: Unable to set attributes");
        return CAR_ERR_FAILURE;
    }

    /* Flush serial port */
    tcflush(obd2_fd, TCIOFLUSH);

    return CAR_ERR_SUCCESS;
}

INT32 Init_MQTT_Client()
{
    INT32 ret_value = MOSQ_ERR_SUCCESS;

    ret_value = mosquitto_lib_init();
    if(ret_value != MOSQ_ERR_SUCCESS)
    {
        printf("CAR_TICK > mosquitto_lib_init: %s\n", strerror(errno));
        return CAR_ERR_FAILURE;
    }

    mosq = mosquitto_new(CLIENT_ID, true, NULL);
    if(mosq == NULL)
    {
        printf("CAR_TICK > mosquitto_new: %s\n", strerror(errno));
        return CAR_ERR_FAILURE;
    }

    /* Last will message in case of unexpected disconnection. QoS 1 ensures delivery, and the message
    is retained so the subscribers can know the last status of the publisher even if it is offline */
    CHAR8 *last_will_msg = "Car Data Publisher connection crashed!";
    ret_value = mosquitto_will_set(mosq, "LastWill", strlen(last_will_msg), last_will_msg, QOS_1, true);

    ret_value = mosquitto_connect(mosq, conf_data->mqtt_broker_ip, conf_data->mqtt_broker_port, 10);
    if(ret_value != MOSQ_ERR_SUCCESS)
    {
        printf("CAR_TICK > mosquitto_connect: %s\n", strerror(errno));
        return CAR_ERR_FAILURE;
    }

    return CAR_ERR_SUCCESS;
}

INT32 Send_PID_Request(const CHAR8 *request) 
{
    INT32 n_written = 0;
    INT32 total = 0;
    
    /* Many systems require to write separately data and ending character
       The ending character for ELM327 is carriage return, so,
       first the request is transmitted and then the carriage return is sent */
    n_written = write(obd2_fd, request, strlen(request));
    if(n_written < 0)
    {
        perror("CAR_TICK > send_pid_request: Failure writing PID request to OBD2");
        return CAR_ERR_FAILURE;
    }
    total += n_written;
    n_written = write(obd2_fd, "\r", 1);
    if(n_written != 1)
    {
        perror("CAR_TICK > send_pid_request: Failure writing carriage return to OBD2");
        return CAR_ERR_FAILURE;
    }
    total += n_written;
    return total;
}

INT32 Read_OBD2_Response(CHAR8 *buffer, UINT64 size, INT32 timeout) 
{
    /* Clear buffer */
    memset(buffer, 0, size);

    fd_set readfds;
    struct timeval tv;
    INT32 ret_value = 0;

    /* Add USB serial port descriptor to fd set */
    FD_ZERO(&readfds);
    FD_SET(obd2_fd, &readfds);

    /* Time structure used to check for timeouts */
    tv.tv_sec = timeout;
    tv.tv_usec = 0;

    /* Check if OBD2 file descriptor is ready to read data */
    ret_value = select(obd2_fd + 1, &readfds, NULL, NULL, &tv);
    if(ret_value == -1) 
    {
        perror("CAR_TICK > read_obd_response: Failure in select function");
        return CAR_ERR_FAILURE;
    } 
    else if(ret_value == 1) 
    {
        INT32 n_read = read(obd2_fd, buffer, size-1);
        if(n_read < 0) 
        {
            perror("CAR_TICK > read_obd_response: Failure reading OBD response");
            return CAR_ERR_FAILURE;
        }
        /* Set the first invalid byte to NULL to ignore the rest of the buffer */
        buffer[n_read] = '\0';
        DBG_LOG_CHAR("read_response", buffer);
        return n_read;
    } 
    else 
    {
        printf("CAR_TICK > read_obd_response: Timeout reached\n");
        return CAR_ERR_SUCCESS;
    }
}

INT32 Parse_Publish_OBD2_Response(INT32 option, CHAR8 *obd2_response, INT32 qos)
{
    INT32 ret_value = CAR_ERR_SUCCESS;

    INT32 scalar_value = 0;             /* Numerical values such as speed or RPM */

    CHAR8 no_space_response[256] = {0}; /* Buffer for response after removal of blanks */
    CHAR8 response_no_header[64] = {0}; /* Buffer for response after removal of header */

    CHAR8 topic[256] = {0};             /* Name of the topic */
    CHAR8 to_pub[64] = {0};             /* Human-readable data to be published in ASCII format */
    INT32 payload_len = 0;              /* Length of the message to be published */

    Remove_Spaces(obd2_response, no_space_response);

    switch(option)
    {
    case OPT_ENG_LOAD:
        memcpy(response_no_header, &no_space_response[START_DATA_INDEX_STD], 2);
        response_no_header[2] = '\0';
        DBG_LOG_CHAR("engine_load_buf", response_no_header);
        scalar_value = (INT32)strtol(response_no_header, NULL, 16);
        DBG_LOG_INT("scalar_eng_ld", scalar_value);
        snprintf(topic, 256, "%s", "RealTime/EngineLoad");
        break;
    case OPT_SPEED:
        memcpy(response_no_header, &no_space_response[START_DATA_INDEX_STD], 2);
        response_no_header[2] = '\0';
        DBG_LOG_CHAR("speed_buf", response_no_header);
        scalar_value = (INT32)strtol(response_no_header, NULL, 16);
        DBG_LOG_INT("scalar_speed", scalar_value);
        snprintf(topic, 256, "%s", "RealTime/Speed");
        break;
    case OPT_CMD_EGR:
        memcpy(response_no_header, &no_space_response[START_DATA_INDEX_STD], 2);
        response_no_header[2] = '\0';
        DBG_LOG_CHAR("EGR_buf", response_no_header);
        scalar_value = (INT32)strtol(response_no_header, NULL, 16);
        DBG_LOG_INT("scalar_EGR", scalar_value);
        snprintf(topic, 256, "%s", "RealTime/ExhaustGasRecirculation");
        break;
    case OPT_RPM:
        memcpy(response_no_header, &no_space_response[START_DATA_INDEX_STD], 4);
        response_no_header[4] = '\0';
        DBG_LOG_CHAR("rpm_buf", response_no_header);
        scalar_value = (INT32)strtol(response_no_header, NULL, 16);
        DBG_LOG_INT("scalar_rpm", scalar_value);
        snprintf(topic, 256, "%s", "RealTime/RPM");
        break;
    case OPT_RUN_TIME:
        memcpy(response_no_header, &no_space_response[START_DATA_INDEX_STD], 4);
        response_no_header[4] = '\0';
        DBG_LOG_CHAR("time_buf", response_no_header);
        scalar_value = (INT32)strtol(response_no_header, NULL, 16);
        DBG_LOG_INT("scalar_time", scalar_value);
        snprintf(topic, 256, "%s", "RealTime/Time");
        break;
    case OPT_AMB_TEMP:
        memcpy(response_no_header, &no_space_response[START_DATA_INDEX_STD], 2);
        response_no_header[2] = '\0';
        DBG_LOG_CHAR("air_temp_buf", response_no_header);
        scalar_value = (INT32)strtol(response_no_header, NULL, 16);
        DBG_LOG_INT("scalar_air_temp", scalar_value);
        snprintf(topic, 256, "%s", "RealTime/AirTemperature");
        break;
    case OPT_ERR_CODES:
        memcpy(response_no_header, &no_space_response[START_DATA_INDEX_ERR], 2);
        response_no_header[2] = '\0';
        DBG_LOG_CHAR("err_buf", response_no_header);
        scalar_value = (INT32)strtol(response_no_header, NULL, 16);
        DBG_LOG_INT("err_codes", scalar_value);
        snprintf(topic, 256, "%s", "Errors");
        break;
    default:
        return CAR_ERR_FAILURE;
    }

    snprintf(to_pub, 64, "%d", scalar_value);
    DBG_LOG_CHAR("buffer_to_pub", to_pub);
    payload_len = strlen(to_pub);

    /* Publish message with no retain option */
    ret_value = mosquitto_publish(mosq, NULL, topic, payload_len, to_pub, qos, false);
    if(ret_value != MOSQ_ERR_SUCCESS)
    {
        printf("CAR_TICK > mosquitto_publish: %s\n", strerror(errno));
    }

    /* If only mosquitto_loop_start is used when multiple publishes are done rapidly, there is a chance
        that some messages may not be sent immediately. Using mosquitto_loop after all messages
        are published ensures that that all of them are correctly handled before continuing to the
        next iteration of the loop */
    mosquitto_loop(mosq, -1, -1);

    return ret_value;
}

void Remove_Spaces(const CHAR8 *raw, CHAR8 *no_spaces)
{
    while(*raw)
    {
        if(!isspace((UCHAR8)*raw))
        {
            *no_spaces++ = *raw;
        }
        raw++;
    }
    *no_spaces = '\0';
}

void Clean_UP()
{
    free(conf_data->serial_device);
    free(conf_data->mqtt_broker_ip);
    free(conf_data->car_model);
    free(conf_data);

    close(obd2_fd);

    mosquitto_disconnect(mosq);

    mosquitto_loop_stop(mosq, false);

    mosquitto_destroy(mosq);

    mosquitto_lib_cleanup();
}

INT32 main() 
{
    /* Set SIGINT_Handler function on SIGINT reception */
    signal(SIGINT, SIGINT_Handler);

    INT32 ret_value = CAR_ERR_SUCCESS;

    t_current_time = 0;
    t_timemark_1s = 0;
    t_timemark_5s = 0;

    conf_data = malloc(sizeof(Conf_Data));

    ret_value = Get_Conf_Data(conf_data);
    if(ret_value != CAR_ERR_SUCCESS)
    {
        return CAR_ERR_FAILURE;
    }

    ret_value = Init_Serial_Read();
    if(ret_value != CAR_ERR_SUCCESS)
    {
        return CAR_ERR_FAILURE;
    }

    ret_value = Init_MQTT_Client();
    if(ret_value != CAR_ERR_SUCCESS)
    {
        return CAR_ERR_FAILURE;
    }

    ret_value = mosquitto_loop_start(mosq);
    if(ret_value != MOSQ_ERR_SUCCESS)
    {
        printf("CAR_TICK > mosquitto_loop_start: %s\n", strerror(errno));
        return CAR_ERR_FAILURE;
    }

    ret_value = mosquitto_reconnect_delay_set(mosq, DELAY_S, DELAY_MAX, DELAY_EXP);
    if(ret_value != MOSQ_ERR_SUCCESS)
    {
        printf("CAR_TICK > mosquitto_reconn_delay_set: %s\n", strerror(errno));
        return CAR_ERR_FAILURE;
    }

    CHAR8 obd2_response[256];

    /* First, transmit static data. This is called only once with QoS 1 to ensure delivery. Duplicates allowed */
    ret_value = mosquitto_publish(mosq, NULL, "Static/Model", strlen(conf_data->car_model), 
                                  conf_data->car_model, QOS_1, false);
    if(ret_value != MOSQ_ERR_SUCCESS)
    {
        printf("CAR_TICK > mosquitto_publish: %s\n", strerror(errno));
        /* Do not exit program, this is not a critical failure */
    }

    while(keepRunning)
    {
        /* At 115.2K bauds, a byte takes around 95us to transmit, so we wait as necessary before reading
           A request is composed of 4 chars (for standard request) + CR + \0, making a total of 6 chars,
           hence 600us to transmit. Then a timeout in reading is set to take care of possible delays */

        clock_gettime(CLOCK_MONOTONIC, &cur_time);
        t_current_time = (TIME64)cur_time.tv_sec * 1000 + cur_time.tv_nsec / 1000000;

        Send_PID_Request(PID_SPEED);
        usleep(TRANSMIT_DELAY);
        Read_OBD2_Response(obd2_response, sizeof(obd2_response), 1);
        DBG_LOG_CHAR("Speed", obd2_response);
        Parse_Publish_OBD2_Response(OPT_SPEED, obd2_response, QOS_0);

        Send_PID_Request(PID_RPM);
        usleep(TRANSMIT_DELAY);
        Read_OBD2_Response(obd2_response, sizeof(obd2_response), 1);
        DBG_LOG_CHAR("RPM", obd2_response);
        Parse_Publish_OBD2_Response(OPT_RPM, obd2_response, QOS_0);

        Send_PID_Request(PID_ENG_LOAD);
        usleep(TRANSMIT_DELAY);
        Read_OBD2_Response(obd2_response, sizeof(obd2_response), 1);
        DBG_LOG_CHAR("Eng_Load", obd2_response);
        Parse_Publish_OBD2_Response(OPT_ENG_LOAD, obd2_response, QOS_0);

        /* 1 second frequency for elapsed time and commanded EGR */
        if ((t_current_time - t_timemark_1s) >= PID_FREQ_1000)
        {
            Send_PID_Request(PID_RUN_TIME);
            usleep(TRANSMIT_DELAY);
            Read_OBD2_Response(obd2_response, sizeof(obd2_response), 1);
            DBG_LOG_CHAR("Time", obd2_response);
            Parse_Publish_OBD2_Response(OPT_RUN_TIME, obd2_response, QOS_0);

            Send_PID_Request(PID_CMD_EGR);
            usleep(TRANSMIT_DELAY);
            Read_OBD2_Response(obd2_response, sizeof(obd2_response), 1);
            DBG_LOG_CHAR("EGR", obd2_response);
            Parse_Publish_OBD2_Response(OPT_CMD_EGR, obd2_response, QOS_0);

            t_timemark_1s = t_current_time;
        }

        /* 5 second frequency for ambient air temperature and error codes */
        if ((t_current_time - t_timemark_5s) >= PID_FREQ_5000)
        {
            Send_PID_Request(PID_AMB_TEMP);
            usleep(TRANSMIT_DELAY);
            Read_OBD2_Response(obd2_response, sizeof(obd2_response), 1);
            DBG_LOG_CHAR("Amb_Temp", obd2_response);
            Parse_Publish_OBD2_Response(OPT_AMB_TEMP, obd2_response, QOS_1);

            Send_PID_Request(PID_ERR_CODES);
            usleep(TRANSMIT_DELAY);
            Read_OBD2_Response(obd2_response, sizeof(obd2_response), 1);
            DBG_LOG_CHAR("Err_Codes", obd2_response);
            Parse_Publish_OBD2_Response(OPT_ERR_CODES, obd2_response, QOS_1);
            
            t_timemark_5s = t_current_time;
        }

        sleep(SLEEP_TIME);
    }

    Clean_UP();
    
    return CAR_ERR_SUCCESS;
}
