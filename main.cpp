#include <stdio.h>
#include <string.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include<common/mavlink.h>
#include <ardupilotmega/mavlink.h>
/*
 *
 * Важно не забыть выставить верный порт и скорость для USB - 115200 а для telem1,2 - 57600
 *
 * */

int
set_interface_attribs (int fd, int speed, int parity) // В это вдаваться не нужно. Я сам не понимаю почему именно этот код в линуксе работает отменно
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
set_blocking (int fd, int should_block) // В этом тоже не нужно разбираться
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

void
send_pack (int fd, float p_lat, float p_lon) // Отправка пакета
{
    // Общий тип для каждого mavlink сообщения. Используется для хранения неструктурированного массива данных
    mavlink_message_t msg;
    // это кастомное сообщение, которое я создал. Лежит в папке mavlink/ardupilotmega
    mavlink_opt_raw_t packet;

    // просто заполняем float нужными величинами
    packet.lat = p_lat;
    packet.lon = p_lon;

    //mavlink_msg_opt_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_opt_raw_t* opt_raw) так выглядит объявление функции
    // заполняем 255 это наземная станция 1 - id устройства на которое отправляются данные
    mavlink_msg_opt_raw_encode(255, 1, &msg, &packet);

    // просто отправка буффера
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint8_t len = mavlink_msg_to_send_buffer(buf, &msg);

    write(fd, ( char * ) ( buf ), len);
}

int main()
{
    // <Не интересно>
    char portname[20] = "/dev/ttyACM0";
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {
        printf ("error %d opening %s: %s", errno, portname, strerror (errno));
        return -1;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking

    // </ Не интересно>


    int w = 0; // счетчик итераций цикла ниже
    int max_skip = 200; // Кол-во итераций циклов, которое должно быть пропущено, чтобы не спамить автопилот(см ниже)


    while (1)
    {
        mavlink_message_t message;
        mavlink_status_t status;

        char buf [200];
        int n = read(fd, buf, sizeof buf); // просто читаем не более 200 байт, которые нам прислали

        for (int pos = 0; pos < n; pos++) // обрабатываем каждый байт. Этот цикл для функции mavlink_parse_char. Она принимает по одному байту.
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)buf[pos],
                                   &message, &status))
            {
                // Если после вновь добавленного байта получилось целостное сообщение, то возвращает true и можно парсить

                //Обрабатываем только нужные нам сообщения
                if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) // сердцебиение
                {
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&message, &heartbeat);

                    printf("\n\n  ");
                    printf("Heartbeat received, system type:%d System status: %d", heartbeat.type, heartbeat.system_status);
                    printf("\n +----------------------------------+\n");
                }

                // это то самое новое сообщение. Здесь происходит разбор принятого пакета
                if (message.msgid == MAVLINK_MSG_ID_OPT_RAW)
                {
                    mavlink_opt_raw_t m_opt_raw;
                    mavlink_msg_opt_raw_decode(&message, &(m_opt_raw));

                    // По сути это и есть ЭХО. Выведет ранее отправленные широту и долготу (функция send_pack ниже это как раз и делает)
                    // Если send_pack ниже закомментировать будут те значения, которые я инициализировал в автопилоте. Т.е. нули.
                    printf("\n\n  ");
                    printf("lat:%f lon: %f", m_opt_raw.lat , m_opt_raw.lon);
                    printf("\n +----------------------------------+\n");

                    //Если этого не достаточно, можно подключиться к автопилоту по USB и через telem1. И проверять выход их другого порта
                    //В данном случае проще позвонить мне, я расскажу.

                    break;
                }

                //Ну и для интереса стандартные пакеты с положением аппарата
                //GPS
                if (message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT)
                {
                    mavlink_gps_raw_int_t gps_raw;
                    mavlink_msg_gps_raw_int_decode(&message, &(gps_raw));
                    //qDebug() <<"lat: " << gps_raw.lat / 1.0e7 << " lon: " << gps_raw.lon / 1.0e7 << " alt: " << gps_raw.alt;

                    float m_lat = gps_raw.lat / 1.0e7f; // Э-Экономия. В этом пакете широта и долгота хранится в shortint
                    float m_lon = gps_raw.lon / 1.0e7f;

                    printf("\n\n  ");
                    printf("GPS lat:%f lon: %f", m_lat , m_lon);
                    printf("\n +----------------------------------+\n");
                }

                //Крен тангаж рысканье
                //Ниже на qt.
                /*
                if (message.msgid == MAVLINK_MSG_ID_ATTITUDE && message.sysid != 0)
                            {
                                mavlink_attitude_t attitude;
                                mavlink_msg_attitude_decode(&message, &attitude);

                                QVariant pitch(qRadiansToDegrees(attitude.pitch));
                                QVariant roll(qRadiansToDegrees(attitude.roll));
                                QVariant yaw(qRadiansToDegrees(attitude.yaw));
                                m_pitch = pitch;
                                m_roll = roll;
                                m_yaw = yaw;
                }
                */


            }
        }

        if(w == max_skip){
            // чтобы слишком часто не отправлять команду пришлось сделать такой костыль. Нужно как-то это дело в отдельный поток.
            // могу помочь сделать это на qt

            //Если закомментировать, то ответное сообщение будет содержать нули

            send_pack(fd, 5.0f, 20.0f); // отправляете рандомные float значения для ЭХО.

            //printf("sent");
        }
        else
            w++;

    }


    close(fd);
    return 0;

}
