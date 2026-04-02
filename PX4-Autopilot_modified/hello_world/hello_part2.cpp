#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/debug_value.h>

// FMU output pins (0-based, so pin 1 = 0)
#define DC_MOTOR        0 //output pin 1 
#define SERVO           1 //output pin 2

// motor: 0.5 = stop, above 0.5 = forward
#define MOTOR_STOP      0.5f
#define MOTOR_SLOW      0.58f
#define MOTOR_NORMAL    0.65f

// servo: 0.5 = center, 0.25 = left, 0.75 = right
#define SERVO_LEFT      0.25f
#define SERVO_CENTER    0.5f
#define SERVO_RIGHT     0.75f

// distance thresholds from ultrasonic (cm)
#define DIST_STOP       15.0f //15cm, stop 
#define DIST_SLOW       50.0f //50cm, slowdown 

extern "C" __EXPORT int hello_world_main(int argc, char *argv[]);

// helper to publish a test_motor command without repeating all the fields
static void publish_motor(uORB::Publication<test_motor_s> &pub, uint32_t motor_num, float value)
{
    test_motor_s msg{};
    msg.timestamp       = hrt_absolute_time();
    msg.motor_number    = motor_num; //which output to control 
    msg.value           = value; //command value to send 
    msg.action          = test_motor_s::ACTION_RUN;
    msg.driver_instance = 0;
    msg.timeout_ms      = 0;
    pub.publish(msg);
}

int hello_world_main(int argc, char *argv[])
{
    // wait for MAVLink and uORB to finish starting up
    px4_sleep(2);

    // subscribe to debug_value - this is what MAVLink maps incoming debug messages to
    debug_value_s debug_data{};
    int debug_handle = orb_subscribe(ORB_ID(debug_value));
    orb_set_interval(debug_handle, 100); // 1000/100 = 10 Hz, ask for updates 10 times per seocnd 

    //create publisher for the test_motor topic 
    uORB::Publication<test_motor_s> motor_pub{ORB_ID(test_motor)};

    // start safe
    publish_motor(motor_pub, DC_MOTOR, MOTOR_STOP);
    publish_motor(motor_pub, SERVO,    SERVO_CENTER);

    while (1)
    {
        //copies the latest debug_value message into debug_data
        //after this point debug_data contains latest info sent from raspberry pi 
        orb_copy(ORB_ID(debug_value), debug_handle, &debug_data);

        // RPi sends: ind = direction (0=left, 1=forward, 2=right), value = distance in cm
        int   direction = (int)debug_data.ind; //direction from camera 
        float dist_cm   = debug_data.value; //distrance from ultrasonic

        // slow down or stop based on distance
        float motor_val;
        if (dist_cm < DIST_STOP)
            motor_val = MOTOR_STOP;
        else if (dist_cm < DIST_SLOW)
            motor_val = MOTOR_SLOW;
        else
            motor_val = MOTOR_NORMAL;

        // steer based on camera direction
        float servo_val;
        if (direction == 0)
            servo_val = SERVO_LEFT;
        else if (direction == 2)
            servo_val = SERVO_RIGHT;
        else
            servo_val = SERVO_CENTER;

        //publish commands
        publish_motor(motor_pub, DC_MOTOR, motor_val);
        publish_motor(motor_pub, SERVO,    servo_val);

        PX4_INFO("dist: %.1f cm | dir: %d | motor: %.2f | servo: %.2f",
                 (double)dist_cm, direction, (double)motor_val, (double)servo_val);

        px4_usleep(100000); // loop runs at about 10 Hz
    }

    return 0;
}
