
/////////////////////////////////////////////////
// KSS Drone Controller Configuration
// This file contains configuration parameters for the KSS Drone Controller.
// Adjust these parameters based on your drone's characteristics and your personal preferences.

/*
1. EXPO (Exponential) Configuration


expo down -> instantaneous response
expo up   -> smoother response around center, more aggressive at ends

static constexpr float EXPO_ROLL  = 0.4f;
static constexpr float EXPO_PITCH = 0.4f;
static constexpr float EXPO_YAW   = 0.2f;

recommended expo values for drone control (these are just starting points and can be adjusted based on personal preference and flight characteristics):
- Roll: 0.3 to 0.5
- Pitch: 0.3 to 0.5
- Yaw: 0.1 to 0.3
*/


/*


2. Deadband Configuration

static constexpr int DEAD_BAND = 5;

recommended deadband values for drone control (these are just starting points and can be adjusted based on personal preference and flight characteristics):
- Roll: 5
- Pitch: 5
- Yaw: 5
*/


/*
3. Maximum Output Configuration

static constexpr float MAX_ANGLE = 0.52f;
static constexpr float MAX_YAW_RATE = 2.0f;

recommended maximum output values for drone control (these are just starting points and can be adjusted based on personal preference and flight characteristics):
- MAX_ANGLE = 0.35~0.7(20~40 degrees)
- MAX_YAW_RATE = 1.5~4.0 (rad/s)

- Roll: 0.5 to 0.6
- Pitch: 0.5 to 0.6
- Yaw: 0.2 to 0.3



*/


/*
4. LPF (Low Pass Filter) Configuration
static constexpr float STICK_ALPHA = 0.7f;
static constexpr float THROTTLE_ALPHA = 0.5f;

values up -> more smoothing, less responsive
values down -> less smoothing, more responsive

recommended LPF alpha values for drone control (these are just starting points and can be adjusted based on personal preference and flight characteristics):
- Stick (Roll, Pitch, Yaw): 0.6 to 0.8
- Throttle: 0.4 to 0.6  

*/


/*
5. Slew Rate Limiter Configuration

static constexpr float STICK_RATE = 4.0f;
static constexpr float THROTTLE_RATE = 2.0f;

recommended slew rate limiter values for drone control (these are just starting points and can be adjusted based on personal preference and flight characteristics):
- Stick (Roll, Pitch, Yaw): 3.0 to 6.0 (rad/s 느낌)
- Throttle: 1.0 to 3.0 (per second) 

*/


/*
6. Task Period Configuration

static constexpr int TASK_PERIOD_MS = 10;

recommended task period values for drone control (these are just starting points and can be adjusted based on personal preference and flight characteristics):
- 5 to 20 ms (higher frequency = more responsive, but also more CPU intensive)

*/

/*
7. Arming Configuration


static constexpr uint32_t ARM_HOLD_MS = 500;
static constexpr float ARM_THROTTLE_MAX = 0.05f;

recommended arming configuration values for drone control (these are just starting points and can be adjusted based on personal preference and flight characteristics):
- ARM_HOLD_MS: 300 to 1000 ms (time required to hold the arming command before the controller arms)
- ARM_THROTTLE_MAX: 0.03 to 0.1 (maximum throttle value allowed during arming, helps prevent accidental arming with
*/
/*
8. Link Loss Configuration

static constexpr uint32_t FAIL_THRESHOLD = 10;

recommended link loss configuration values for drone control (these are just starting points and can be adjusted based on personal preference and flight characteristics):
- FAIL_THRESHOLD: 5 to 20 (number of consecutive failed transmissions before considering the link lost)
*/

/////////////////////////////////////////////////////////
//EspNowInterface Configuration
// This section contains configuration parameters for the EspNowInterface, which handles wireless communication between the controller and the drone. Adjust these parameters based on your specific use case and requirements.

/*
1. send period configuration

controlled by kss_controller.cpp -> TASK_PERIOD_MS


*/


/*
2. Timeout and Retry Configuration
TIUMEOUT_US = 1000000; // 1 second
MAX_RETRY = 5;

recommended timeout and retry configuration values for EspNowInterface (these are just starting points and can be adjusted based on your specific use case and requirements):
- TIMEOUT_US: 500000 to 2000000 (microseconds, time to wait for an acknowledgment before considering the transmission failed)
- MAX_RETRY: 3 to 10 (number of retry attempts for failed transmissions)

*/

/*
3. channel and encryption configuration

peer.channel = 1;
peer.encrypt = false;

recommended channel and encryption configuration values for EspNowInterface (these are just starting points and can be adjusted based on your specific use case and requirements):
- CHANNEL: 1, 6 or 11 (Wi-Fi channel to use for communication)
- ENCRYPT: true or false (enable or disable encryption for the communication)

*/


////////////////////////////////////////////////
//ContollerInput Configuration
// This section contains configuration parameters for the ControllerInput class, which handles reading input from the PS2 controller. Adjust these parameters based on your specific use case and requirements.

/*
1. Task Period Configuration

polling period for reading controller input

static constexpr uint32_t TASK_PERIOD_MS = 10; // 100Hz
*/


/////////////////////////////
/*
1. -> DEAD_BAND, STICK_ALPHA : Prevent stick sloppiness around center (deadband) and smooth stick input (LPF)
2. -> EXPO_ROLL, EXPO_PITCH, EXPO_YAW : Apply exponential scaling to stick inputs for finer control around center
3. -> STICK_RATE, THROTTLE_RATE : Limit the rate of change for stick and throttle inputs
4. -> MAX_ANGLE, MAX_YAW_RATE : Cap the maximum angle and yaw rate for the drone
*/