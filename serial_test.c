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
        } else if (mode>='0' && mode<='9') {
            set_motors((mode-'0')*10, (mode-'0')*10);
        }
        //sleep(1);

        read_encoders(&enc);
        printf("LEFT: %10d\tRIGHT: %10d\n", enc.left, enc.right);
    }

    set_motors(0, 0);
}

