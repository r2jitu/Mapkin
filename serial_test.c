#include <stdio.h>

#include "serial_comm.h"

int main() {
    encoders enc;

    init_comm();
    
    set_motors(100, 100);

    while (1) {
        char mode = getchar();
        if (mode == 'q') {
            break;
        }
        //sleep(1);

        read_encoders(&enc);
        printf("LEFT: %10d\tRIGHT: %10d\n", enc.left, enc.right);
    }

    set_motors(0, 0);
}

