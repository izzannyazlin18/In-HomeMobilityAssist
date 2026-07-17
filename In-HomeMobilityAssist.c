#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <math.h>

#define I2C_PORT            i2c0
#define AS5600_ADDR         0x36         // AS5600 I2C address
#define AS5600_RAW_ANGLE_REG 0x0C         // Register for raw angle reading
#define TCA9548A_ADDR        0x70

//I2C Pin on Raspberry Pi Pico
#define SDA_PIN 4 
#define SCL_PIN 5

//Busses on MUX
#define bus_num_t           1
#define bus_num_r           0
#define bus_num_rot         2
#define bus_num_tran        3

//Translatioinal Motor Pins
#define PUL_PIN_T           0
#define DIR_PIN_T           1
#define ENA_PIN_T           4

#define PUL_PIN_T2          2
#define DIR_PIN_T2          3
//#define ENA_PIN_T2          12

//Rotational Motor Pins
#define PUL_PIN_R           16
#define DIR_PIN_R           17
//#define ENA_PIN_R           8

#define FORWARD true
#define BACKWARD false

//define geometry constant
#define Z 33 // distance from end effector to ground
#define R 40 // fixed arm length

// useful global constants
#define BTI 0.010 // interval time
#define BTI_divisions 2000
#define FORWARD true
#define BACKWARD false

// Define clamping bounds based on stepping rate
#define STEP_DIV_MIN 1000      // 1000 µs between steps = 1000 steps/sec
#define STEP_DIV_MAX 10000     // 10,000 µs between steps = 100 steps/sec

bool last_state_translate = false;
bool last_state_translate2 = false;
bool last_state_rotate = false;

//Initialize Variables for Sensors
//Theta t
static float angular_velocity_t = 0;
static float angle_delta_t = 0;
static float current_angle_t = 0;
static float last_angle_t = 0;
static float total_movement_t = 0;
static float raw_angle_t = 0;
const float z_axis_angle_t = 294.69;

//Theta tran
static float angular_velocity_tran = 0;
static float angle_delta_tran = 0;
static float current_angle_tran = 0;
static float last_angle_tran = 0;
static float total_movement_tran = 0;
static float raw_angle_tran = 0;
const float z_axis_angle_tran = 113.90;

//Theta rot
static float angular_velocity_rot = 0;
static float angle_delta_rot = 0;
static float current_angle_rot = 0;
static float last_angle_rot = 0;
static float total_movement_rot = 0;
static float raw_angle_rot = 0;
const float z_axis_angle_rot = 347.69;

////Theta r
static float angular_velocity_r = 0;
static float angle_delta_r = 0;
static float current_angle_r = 0;
static float last_angle_r = 0;
static float total_movement_r = 0;
static float raw_angle_r = 0;
const float z_axis_angle_r = 176.13;

//Initialize variables for kinematics
static float theta_r = 0;
static float theta_t = 0;
static float theta_tran = 0;
static float theta_rot = 0;

static float theta_2 = 0;

static float deltaX = 0;
static float deltaY = 0;

static float delta_theta_hub = 0;
static float y_hub = 0;

static float theta_1 = M_PI/2;
static float delta_theta_hub_last = 0;
static float y_hub_last = 0;

void init_motor(int PUL_PIN, int DIR_PIN){
    //Initialize Translational Motor
    gpio_init(PUL_PIN);
    gpio_set_dir(PUL_PIN, GPIO_OUT);
    gpio_put(PUL_PIN, 0);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 1); // 1 for forward, 0 for reverse
}

void init_PICOI2C(){
    //Initialize Pico LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN,GPIO_OUT);
    printf("Starting Scan");

    gpio_put(PICO_DEFAULT_LED_PIN,1); // wait 10 seconds to start serial output
    sleep_ms(1500);
    gpio_put(PICO_DEFAULT_LED_PIN,0);
    printf("Starting Program");

    // Initialize I2C at 100 kHz.
    i2c_init(I2C_PORT, 100 * 1000);
    
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C); // Set up GPIO pins for I2C functionality.
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    
    gpio_pull_up(SDA_PIN); // Enable pull-ups on I2C lines.
    gpio_pull_up(SCL_PIN);
}

void mux_bus_select(int bus_num){
    uint8_t control = 1 << bus_num;
    int bus_ret = i2c_write_blocking(I2C_PORT,TCA9548A_ADDR,&control,1,false);
    if (bus_ret<0){
        printf("Bus write error.\n");
    }
    sleep_us(100);
}

// Function to read the 12-bit angle from the AS5600
uint16_t read_as5600_angle(int bus_num) {
    mux_bus_select(bus_num);
    uint8_t reg = AS5600_RAW_ANGLE_REG;
    uint8_t buffer[2];

    // Write the register address to the sensor.
    // The 'true' parameter sends a repeated start instead of a stop.
    int ret = i2c_write_blocking(I2C_PORT, AS5600_ADDR, &reg, 1, true);
    if (ret < 0) {
        printf("I2C write error\n");
        return 0;
    }

    // Read 2 bytes from the sensor.
    ret = i2c_read_blocking(I2C_PORT, AS5600_ADDR, buffer, 2, false);
    if (ret < 0) {
        printf("I2C read error\n");
        return 0;
    }

    // Combine the two bytes into a 16-bit value.
    uint16_t angle = ((uint16_t)buffer[0] << 8) | buffer[1];
    // The AS5600 provides a 12-bit value (0-4095), so mask the upper bits.
    angle &= 0x0FFF;
    return angle;
}

// Function to calculate angle movement, accounting for rollover
void calculate_angle_movement(float* angle_delta, float current_angle, float* last_angle, float* angular_velocity, float* total_movement) {
    *angle_delta = current_angle - *last_angle;
    *angular_velocity = *angle_delta / BTI;

    if (*angle_delta > 180.0f)
        *angle_delta -= 360.0f;
    else if (*angle_delta < -180.0f)
        *angle_delta += 360.0f;

    *total_movement += *angle_delta;
    *last_angle = current_angle;
}

void actuate_motors(int i, int translate_divs, int translate2_divs, int rotate_divs, float y_hub, float acos_input){
    
    if(i % translate_divs == 0){

        if(fabs(y_hub) > 2.0f ){

        bool direction_translate = y_hub > 0 ? FORWARD : BACKWARD;

        gpio_put(DIR_PIN_T, direction_translate);
        gpio_put(PUL_PIN_T, !last_state_translate);

        last_state_translate = !last_state_translate;
        }
    }

    if(i % translate2_divs == 0){

        if(fabs(y_hub) > 2.0f ){

        bool direction_translate = y_hub > 0? BACKWARD: FORWARD;

        gpio_put(DIR_PIN_T2, direction_translate);
        gpio_put(PUL_PIN_T2, !last_state_translate2);

        last_state_translate2 = !last_state_translate2;
        }

    }

    if(i % rotate_divs == 0){

        if(fabs(delta_theta_hub) >= 0.1f){
        
        bool direction_rotate = delta_theta_hub > 0 ? BACKWARD : FORWARD;

        gpio_put(DIR_PIN_R, direction_rotate);
        gpio_put(PUL_PIN_R, !last_state_rotate);

        last_state_rotate = !last_state_rotate;
        }
        
    }
}

int main() {
    // Enable UART so we can print status output
    stdio_init_all();

    init_motor(PUL_PIN_R,DIR_PIN_R);//initialize Rotational Motor

    init_motor(PUL_PIN_T,DIR_PIN_T);//initialize Translational Motor 1

    init_motor(PUL_PIN_T2,DIR_PIN_T2);//initialize Translational Motor 2

    init_PICOI2C();// initialize PICO BLINK and I2C

    //Theta t
    angular_velocity_t = 0;
    angle_delta_t = 0;
    current_angle_t = 0;
    last_angle_t = 0;
    total_movement_t = 0;
    raw_angle_t = 0;
    
    ////Theta r
    angular_velocity_r = 0;
    angle_delta_r = 0;
    current_angle_r = 0;
    last_angle_r = 0;
    total_movement_r = 0;
    raw_angle_r = 0;

    //Theta translational
    angular_velocity_tran = 0;
    angle_delta_tran = 0;
    current_angle_tran = 0;
    last_angle_tran = 0;
    total_movement_tran = 0;
    raw_angle_tran = 0;

    //Theta rotational
    angular_velocity_rot = 0;
    angle_delta_rot = 0;
    current_angle_rot = 0;
    last_angle_rot = 0;
    total_movement_rot = 0;
    raw_angle_rot = 0;
    
    //Initialize variables for kinematics
    theta_r = 0;
    theta_t = 0;
    theta_tran = 0;
    theta_rot = 0;
    
    deltaX = 0;
    deltaY = 0;
    
    theta_2 = 0;
    delta_theta_hub = 0;
    y_hub = 0;
    
    theta_1 = M_PI/2;
    delta_theta_hub_last = 0;
    y_hub_last = 0;

    float y_hub_velocity = 0;
    float delta_theta_hub_velocity = 0;

    float L = 0;

    printf("AS5600 t Magnetic Encoder Demo\n");
    last_angle_t = read_as5600_angle(bus_num_t);
    last_angle_t = (last_angle_t*360.0f)/4096.0f; 

    printf("AS5600 r Magnetic Encoder Demo\n");
    last_angle_r = read_as5600_angle(bus_num_r);
    last_angle_r = (last_angle_r*360.0f)/4096.0f; 

    printf("AS5600 tran Magnetic Encoder Demo\n");
    last_angle_tran = read_as5600_angle(bus_num_tran);
    last_angle_tran = (last_angle_tran*360.0f)/4096.0f; 

    printf("AS5600 rot Magnetic Encoder Demo\n");
    last_angle_rot = read_as5600_angle(bus_num_rot);
    last_angle_rot = (last_angle_rot*360.0f)/4096.0f; 

    while (true) {

        raw_angle_t = read_as5600_angle(bus_num_t); 
        current_angle_t = (raw_angle_t*360.0f)/4096.0f; 
        calculate_angle_movement(&angle_delta_t, current_angle_t, &last_angle_t, &angular_velocity_t, &total_movement_t);

        raw_angle_r = read_as5600_angle(bus_num_r);
        current_angle_r = (raw_angle_r*360.0f)/4096.0f; 
        calculate_angle_movement(&angle_delta_r, current_angle_r, &last_angle_r, &angular_velocity_r, &total_movement_r);

        raw_angle_tran = read_as5600_angle(bus_num_tran); 
        current_angle_tran = (raw_angle_tran*360.0f)/4096.0f; 
        calculate_angle_movement(&angle_delta_tran, current_angle_tran, &last_angle_tran, &angular_velocity_tran, &total_movement_tran);

        raw_angle_rot = read_as5600_angle(bus_num_rot);
        current_angle_rot = (raw_angle_rot*360.0f)/4096.0f; 
        calculate_angle_movement(&angle_delta_rot, current_angle_rot, &last_angle_rot, &angular_velocity_rot, &total_movement_rot);

        theta_t = current_angle_t - z_axis_angle_t;
        float theta_t_rad = theta_t * (M_PI / 180.0f);

        theta_r = current_angle_r - z_axis_angle_r;
        float theta_r_rad = theta_r * (M_PI / 180.0f);
        
        theta_tran = current_angle_tran - z_axis_angle_tran;
        float theta_tran_rad = theta_tran * (M_PI / 180.0f);

        theta_rot = current_angle_rot - z_axis_angle_rot;
        float theta_rot_rad = theta_rot * (M_PI / 180.0f);
      
        deltaX = (Z*theta_r_rad *cos(theta_1)) - (Z*theta_t_rad*sin(theta_1));
        deltaY = (Z*theta_r_rad*sin(theta_1)) + (Z*theta_t_rad*cos(theta_1));

        float acos_input = ((R*cos(theta_1))+ deltaX)/R;

        if (acos_input > 1.0f) { //clamping acos input values 
            acos_input = 1.0f;
        } else if (acos_input < -1.0f) {
            acos_input = -1.0f;
        }

        theta_2 = acos(acos_input);

        delta_theta_hub = theta_2 - theta_1;
        y_hub = (R*sin(theta_1))+ deltaY - (R*sin(theta_2));

        angular_velocity_tran = (2.3*angular_velocity_tran* (M_PI / 180.0));
        angular_velocity_rot = (angular_velocity_rot * (M_PI / 180.0));

        y_hub_velocity = ((y_hub - y_hub_last)/BTI)+angular_velocity_tran;//human trans vel
        delta_theta_hub_velocity = ((delta_theta_hub - delta_theta_hub_last)/BTI)+angular_velocity_rot;//human trans vel

        L = sqrt(((Z*tan(theta_r_rad))*(Z*tan(theta_r_rad)))+ ((Z*tan(theta_t_rad))*(Z*tan(theta_t_rad))));

        //PI Controller for Translational & Rotational Motor
        int Kp_R = 150000;
        int Ki_R = 0.005;

        float num_steps_R = fabs((Kp_R * delta_theta_hub * BTI) + Ki_R *(delta_theta_hub_last*BTI + 0.5f*(delta_theta_hub - delta_theta_hub_last)*BTI));

        int Kp_T = 20000;
        int Ki_T = 0.005;

        float num_steps_T = fabs((Kp_T * y_hub* BTI) + Ki_R *(y_hub_last*BTI + 0.5f*(y_hub - y_hub_last)*BTI));

        float num_steps_T2 = fabs(1.0352*((Kp_T * y_hub* BTI) + Ki_R *(y_hub_last*BTI + 0.5f*(y_hub - y_hub_last)*BTI)));

        if (num_steps_R < 0.000001f) num_steps_R = 0.000001f; //avoid dividing zero
        if (num_steps_T < 0.000001f) num_steps_T = 0.000001f; //avoid dividing zero
        if (num_steps_T2 < 0.000001f) num_steps_T2 = 0.000001f; //avoid dividing zero

        float translate_divs = (BTI*1000/num_steps_T)*BTI_divisions*0.5f;
        float translate2_divs = (BTI*1000/num_steps_T2)*BTI_divisions*0.5f;
        float rotate_divs =  (BTI*1000/num_steps_R)*BTI_divisions*0.5f;
            
        last_state_rotate = true;
        last_state_translate2 = true;
        last_state_translate = true;
            
        for (int i = 0; i < BTI_divisions; i++){
            actuate_motors(i, translate_divs, translate2_divs, rotate_divs, y_hub, acos_input);
            sleep_us((int)((BTI*1000000)/BTI_divisions));
        }

        last_state_rotate = false;
        last_state_translate = false;
        last_state_translate2 = false;   
        
       
        
        // The angle is a raw 12-bit value (0-4095). Optionally convert to degrees:
        printf(" %f , %f,  %f, %f \n", y_hub_velocity, angular_velocity_tran, delta_theta_hub_velocity, angular_velocity_rot);

        //, Delta Theta Human Velocity: %f, Rotational Hub Velocity: %f\n", y_hub_velocity , angular_velocity_tran, delta_theta_hub_velocity, angular_velocity_rot);

        printf("\n");

        theta_2 = theta_1;
        y_hub_last = y_hub;
        delta_theta_hub_last = delta_theta_hub;
    }

    return 0;
}