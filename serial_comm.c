#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "serial_comm.h"

#define ENC_LEFT_FACTOR  1.0f
#define ENC_RIGHT_FACTOR 1.0f

static const char *commport = "/dev/ttyUSB0";
static int commfd = -1;

int motor_l_speed = 0;
int motor_r_speed = 0;

void init_comm() {
    commfd = open(commport, O_RDWR);
    if (commfd < 0) {
        printf("Couldn't open %s\n", commport);
        exit(1);
    }

    struct termios tio;
    memset(&tio, 0, sizeof(struct termios));
    tio.c_cflag = CS8 | CREAD | CLOCAL;
    tio.c_iflag = IXON | IXOFF | IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VTIME] = 0;
    tio.c_cc[VMIN] = 1;
    cfsetospeed(&tio, B115200);
    cfsetispeed(&tio, B115200);
    
    tcsetattr(commfd, TCSANOW, &tio);
    tcsetattr(commfd, TCSAFLUSH, &tio);

    printf("Initializing... (takes 2 seconds)\n");
    sleep(2);
}

void comm_send_msg(char *msg) {
    //if (IS_SIM) return;

    write(STDOUT_FILENO, msg, strlen(msg));
    write(commfd, msg, strlen(msg));
}

int comm_read_msg(char *buf, int size) {
    int bytes = 0;

    do {
        read(commfd, buf+bytes, 1);
        bytes++;
    } while (buf[bytes-1]!='\n' && bytes<size);

    return bytes;
}

void set_motors(int left, int right) {
    char msg[16];
    sprintf(msg, "IN %d %d\r\n", right, left);
    comm_send_msg(msg);
}

void set_motor_l(int speed)
{
    if(speed < -100) {
        printf("WARNING: Speed out of bounds.\n");
        speed = -100;
    } else if (speed > 100) {
        printf("WARNING: Speed out of bounds.\n");
        speed = 100;
    }

    motor_l_speed = speed;
    set_motors(motor_l_speed, motor_r_speed);
}

void set_motor_r(int speed)
{
    if(speed < -100) {
        printf("WARNING: Speed out of bounds.\n");
        speed = -100;
    } else if (speed > 100) {
        printf("WARNING: Speed out of bounds.\n");
        speed = 100;
    }

    motor_r_speed = speed;
    set_motors(motor_l_speed, motor_r_speed);
}

void read_encoders(encoders *enc) {
    char buf[32];

    comm_send_msg("OUT\r\n");
    comm_read_msg(buf, 32);

    sscanf(buf, "%d %d", &enc->left, &enc->right);
    enc->left = (int)(enc->left*ENC_LEFT_FACTOR);
    enc->right = (int)(enc->right*ENC_RIGHT_FACTOR);
}
 
