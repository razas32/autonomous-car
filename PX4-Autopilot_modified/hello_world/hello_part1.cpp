#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/test_motor.h>

// FMU output pins (0-based, so pin 1 = 0)
#define DC_MOTOR        0 //connected to output pin 1 
#define SERVO           1 //connected to output pin 2 

// FlySky RC channels (0-based)
#define RC_THROTTLE_CH  2   // CH3: left stick vertical, array index 2 
#define RC_STEERING_CH  0   // CH1: right stick horizontal, array index 0 

// RC signal range in microseconds
#define RC_MIN  1000
#define RC_MAX  2000

extern "C" __EXPORT int hello_world_main(int argc, char *argv[]);

// converts RC value (1000-2000) to a float between out_min and out_max
static float rc_map(uint16_t rc_val, float out_min, float out_max)
{
    //(rc_value -1000)/ (2000-1000)
    float t = (float)(rc_val - RC_MIN) / (float)(RC_MAX - RC_MIN);
    //clamp
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return out_min + t * (out_max - out_min);
}

int hello_world_main(int argc, char *argv[])
{
    //create publisher for test_motor & subscriber to the input_rc
    uORB::Publication<test_motor_s> motor_pub{ORB_ID(test_motor)};
    uORB::Subscription rc_sub{ORB_ID(input_rc)};

    input_rc_s   rc_data{}; //stores latest RC input message
    test_motor_s motor_cmd{}; //hold DC motor command before publishing 
    test_motor_s servo_cmd{}; //hold servo command before publishing 

    PX4_INFO("RC motor/servo control started.");
    PX4_INFO("Throttle -> CH%d | Steering -> CH%d", RC_THROTTLE_CH + 1, RC_STEERING_CH + 1);
    PX4_INFO("Make sure RC receiver is connected and RC controller is on.");

    while (1)
    {
        if (rc_sub.update(&rc_data)) // new RC message has arrived 
        {
            if (rc_data.rc_lost)
            {
                PX4_WARN("RC signal lost - holding last command");
            }
            else
            {
                // throttle: 1000=full reverse, 1500=stop, 2000=full forward
                float motor_val = rc_map(rc_data.values[RC_THROTTLE_CH], 0.0f, 1.0f);

                // steering: 1000=full left, 1500=center, 2000=full right
                float servo_val = rc_map(rc_data.values[RC_STEERING_CH], 0.0f, 1.0f);

                //after rc_map() computes motor_val and servo_val, they are written 
                //into a test_motor_s struct and published 
                
                // send motor speed
                motor_cmd.timestamp       = hrt_absolute_time();
                motor_cmd.motor_number    = DC_MOTOR;
                motor_cmd.value           = motor_val;
                motor_cmd.action          = test_motor_s::ACTION_RUN;
                motor_cmd.driver_instance = 0;
                motor_cmd.timeout_ms      = 0;
                motor_pub.publish(motor_cmd);

                // send servo position (same topic, different motor_number)
                servo_cmd.timestamp       = hrt_absolute_time();
                servo_cmd.motor_number    = SERVO;
                servo_cmd.value           = servo_val;
                servo_cmd.action          = test_motor_s::ACTION_RUN;
                servo_cmd.driver_instance = 0;
                servo_cmd.timeout_ms      = 0;
                motor_pub.publish(servo_cmd);

                PX4_INFO("Motor: %.2f  Servo: %.2f  (raw: throttle=%d steering=%d)",
                         (double)motor_val, (double)servo_val,
                         rc_data.values[RC_THROTTLE_CH],
                         rc_data.values[RC_STEERING_CH]);
            }
        }

        px4_usleep(200000); // 5 Hz, sleeps for 0.2 seconds, loop runs about 5 times per second 
    }

    // stop everything on exit
    PX4_INFO("Stopping motor and centering servo");

    motor_cmd.timestamp       = hrt_absolute_time();
    motor_cmd.motor_number    = DC_MOTOR;
    motor_cmd.value           = 0.5f;
    motor_cmd.action          = test_motor_s::ACTION_RUN;
    motor_cmd.driver_instance = 0;
    motor_cmd.timeout_ms      = 0;
    motor_pub.publish(motor_cmd);

    servo_cmd.timestamp       = hrt_absolute_time();
    servo_cmd.motor_number    = SERVO;
    servo_cmd.value           = 0.5f;
    servo_cmd.action          = test_motor_s::ACTION_RUN;
    servo_cmd.driver_instance = 0;
    servo_cmd.timeout_ms      = 0;
    motor_pub.publish(servo_cmd);

    return 0;
}
