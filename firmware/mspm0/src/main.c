#include "robot_app.h"

int main(void)
{
    robot_app_t app;

    robot_app_init(&app);

    while (1) {
        robot_app_poll_uart(&app);
        robot_app_tick(&app);
    }
}
