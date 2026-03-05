#ifndef ECVT_CONTROLLER_H
#define ECVT_CONTROLLER_H
#include <odrive.h>
#include <sys/time.h>
#include <telemetry.h>
class ECVTController {
public:
    ECVTController() {}

private:
    ODrive ecvt_odrive;
    ODrive centerlock_odrive;
    void control_loop();
    Telemetry* telem;

};

#endif // ECVT_CONTROLLER_H