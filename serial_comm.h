#ifndef _SERIAL_COMM_
#define _SERIAL_COMM_

typedef struct encoders {
    int left, right;
} encoders;

void init_comm();

void comm_send_msg(char *msg);
int comm_read_msg(char *buf, int size);

void set_motors(int left, int right);
void set_motor_l(int speed);
void set_motor_r(int speed);

void read_encoders(encoders *enc);

#endif /* _SERIAL_COMM_ */
