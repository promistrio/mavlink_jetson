#include <stdio.h>
#include <string.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include<common/mavlink.h>
#include <ardupilotmega/mavlink.h>

int
set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        printf ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void
set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf ("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        printf ("error %d setting term attributes", errno);
}

int main()
{

    char portname[20] = "/dev/ttyACM0";
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {
        printf ("error %d opening %s: %s", errno, portname, strerror (errno));
        return -1;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking

    //write (fd, "hello!\n", 7);           // send 7 character greeting

    //usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
    // receive 25:  approx 100 uS per char transmit
    /*char buf [100];
    int n = read (fd, buf, sizeof buf);

    printf ("number of chars: %d", n);*/



    while (1)
    {
        mavlink_message_t message;
        mavlink_status_t status;

        char buf [200];  //Buffer to store the data received

        int n = read(fd, buf, sizeof buf);
        //printf ("number of chars: %d", n);

        for (int pos = 0; pos < n; pos++)
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)buf[pos],
                                   &message, &status))
            {
                if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
                {
                    printf("\n\n  ");

                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&message, &heartbeat);

                    printf("Heartbeat received, system type:%d System status: %d", heartbeat.type, heartbeat.system_status);
                    printf("\n +----------------------------------+\n");
                }

                if (message.msgid == MAVLINK_MSG_ID_OPT_RAW)
                {
                    printf("\n\n  ");

                    mavlink_opt_raw_t m_opt_raw;
                    mavlink_msg_opt_raw_decode(&message, &(m_opt_raw));

                    printf("lat:%f lon: %f", m_opt_raw.lat , m_opt_raw.lon);
                    printf("\n +----------------------------------+\n");
                    break;
                }
            }
        }
    }


    close(fd); /* Close the serial port */
    return 0;

}

//#include <stdio.h>
//#include <fcntl.h>   /* File Control Definitions           */
//#include <termios.h> /* POSIX Terminal Control Definitions */
//#include <unistd.h>  /* UNIX Standard Definitions 	   */
//#include <errno.h>   /* ERROR Number Definitions           */
//#include<common/mavlink.h>
//#include <ardupilotmega/mavlink.h>

//int main()
//{
//    int fd;/*File Descriptor*/

//    printf("\n +----------------------------------+");
//    printf("\n |        Serial Port Read          |");
//    printf("\n +----------------------------------+");

//    /*------------------------------- Opening the Serial Port -------------------------------*/

//    /* Change /dev/ttyUSB0 to the one corresponding to your system */

//    fd = open("/dev/ttyACM0",O_RDWR | O_NOCTTY);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
//    /* O_RDWR   - Read/Write access to serial port       */
//    /* O_NOCTTY - No terminal will control the process   */
//    /* Open in blocking mode,read will wait              */



//    if(fd == -1)						/* Error Checking */
//    {
//        printf("\n  Error! in Opening serial");
//        close(fd);
//        return -1;
//    }
//    else
//        printf("\n  serial Opened Successfully ");


//    /*---------- Setting the Attributes of the serial port using termios structure --------- */

//    struct termios SerialPortSettings;	/* Create the structure                          */

//    tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

//    /* Setting the Baud rate */
//    cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 9600                       */
//    cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 9600                       */

//    /* 8N1 Mode */
//    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
//    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
//    SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
//    SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

//    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
//    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */


//    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
//    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

//    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

//    /* Setting Time outs */
//    SerialPortSettings.c_cc[VMIN] = 10; /* Read at least 10 characters */
//    SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */


//    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0){ /* Set the attributes to the termios structure*/
//        printf("\n  ERROR ! in Setting attributes");
//        close(fd);
//        return -2;
//    }
//    else
//        printf("\n  BaudRate = 9600 \n  StopBits = 1 \n  Parity   = none");


//    /*------------------------------- Read data from serial port -----------------------------*/

//    //tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */



//    mavlink_message_t message;
//    mavlink_status_t status;




//    char read_buffer[32];   //Buffer to store the data received
//    int  bytes_read = 0;    // Number of bytes read by the read() system call

//    bytes_read = read(fd,&read_buffer,32);

//    printf("\n\n  Bytes Rxed -%d", bytes_read);

//    for (int pos = 0; pos < bytes_read; ++pos)
//    {


//        if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)read_buffer[pos],
//                               &message, &status))
//        {
//            if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
//            {
//                printf("\n\n  ");
//                mavlink_heartbeat_t heartbeat;
//                mavlink_msg_heartbeat_decode(&message, &heartbeat);

//                printf("Heartbeat received, system type:%d System status: %d", heartbeat.type, heartbeat.system_status);
//                printf("\n +----------------------------------+\n\n\n");
//            }
//        }
//    }
//    //printf("\n\n  Bytes Rxed -%d", bytes_read);

//    //for(i=0;i<bytes_read;i++)
//    //    printf("%c",read_buffer[i]);


//    close(fd); /* Close the serial port */
//    return 0;

//}
