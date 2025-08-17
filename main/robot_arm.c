/*  
 * talk to PCA9685 using i2c
 * 
 * ref:
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
 * https://github.com/espressif/esp-idf/blob/master/examples/peripherals/i2c/i2c_basic/main/i2c_basic_example_main.c
 * https://www.alldatasheet.com/datasheet-pdf/pdf/293576/NXP/PCA9685.html (See 7)
 * https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/Adafruit_PWMServoDriver.cpp
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/vfs.html#_CPPv423uart_vfs_dev_use_driveri
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"

#define RAD_TO_DEG(radian) (radian/M_PI)*180
#define DEG_TO_RAD(degrees) (degrees/180)*M_PI

// defined in Kconfig
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL       
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA       
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY 

// PRE_SCALE determines pwm output freq / frame size
// round(25MHz/(4096*50Hz)) - 1 = 121 
#define PRESCALE_VAL 121

// PCA9685 datasheet constants
#define REG_MODE1 0x0
#define REG_LED5 0x1A // servo0 at led5 
#define REG_PRESCALE 0xFE

#define MODE1_SLEEP 16
#define MODE1_AI 32
#define MODE1_RESTART 128

//TODO why are some movements so much faster than others?
#define DELAY_PWM 100 // width of delay block each pwm frame
#define MIN_WIDTH 108
#define MAX_WIDTH 533
#define SERVO_RANGE 178 // servo range 0-178 degrees

// link lengths
#define A1 10.5
#define A2 12.4
#define A3 18.2

// joint equilibrium (90 degrees) actual angles
#define EQB1 45
#define EQB2 0
#define EQB3 0

#define UPDATE_RATE 5    // angles to add each time
#define UPDATE_DELAY 50 // delay to wait each time
#define GRIPPER_CLOSE_ANGLE 46
#define GRIPPER_OPEN_ANGLE 10

#define TX_PIN 26
#define RX_PIN 27

double cur_angles[] = {90,2.316760,3.797824,151.481064,90,0};
double target_angles[6] = {};

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;
static const char* TAG = "I2C";

uint8_t read_reg(uint8_t reg){
    uint8_t data[] = {reg};
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle,data,1,data,1,-1));
    return data[0];
}

void write_reg(uint8_t reg, uint8_t byte){
    uint8_t data[] = {reg,byte};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle,data,2,-1));
}

void delay(int ms){
    // portTICK_PERIOD_MS: how many ms per tick
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void uart_print(char* str){
    uart_write_bytes(UART_NUM_2,str,strlen(str));
}

char uart_getc(){
    // block main thread until read a byte
    size_t uart_buf_len = 0;
    while (uart_buf_len == 0) 
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2,&uart_buf_len));

    // there is data to be read
    char c;
    uart_read_bytes(UART_NUM_2,&c,1,0);
    return c;
}

void uart_fgets(char *buf, int len) {
    // len includes the null byte
    int i;

    for (i = 0; i < (len-1); i++){
        char c = uart_getc();
        buf[i] = c;

        if (c == '\n') {
            i++;
            break;
        }
    }

    buf[i] = '\x00';
    return;
}


void set_angle(uint8_t id, double angle){
    if (angle > 178 || angle < 0){
        ESP_LOGE(TAG,"set_angle(), tried to set servo %d to angle %f",id,angle);
    }

    // map angle to pwm width
    double pwm_width = MIN_WIDTH + angle * ((MAX_WIDTH - MIN_WIDTH)/(double)SERVO_RANGE);
           pwm_width = round(pwm_width);
    uint16_t on = DELAY_PWM;
    uint16_t off = DELAY_PWM + (uint16_t)pwm_width;

    uint8_t buf[] = {REG_LED5 + 4*id, on & 0xff, on >> 8, off & 0xff, off >> 8};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle,buf,5,-1));
}

bool ik_solve(double target[3], double sol_1[3], double sol_2[3]){
    double x = target[0], y = target[1], psi = target[2];    
    double theta1,theta2;

    // use 2R equations
    double p2x = x - A3*cos(DEG_TO_RAD(psi));
    double p2y = y - A3*sin(DEG_TO_RAD(psi));

    theta2 = acos((p2x*p2x + p2y*p2y - A1*A1 - A2*A2) / (2 * A1 * A2));

    //TODO figure out -x,-y
    if ((p2x < 0 && p2y < 0) || isnan(theta2)){
        return false;
    }
 
    // find 1st solution
    if (p2x >= 0)
        theta1 = atan(p2y/p2x) - atan( A2*sin(theta2) / (A1 + A2*cos(theta2)) );  
    else
        theta1 = M_PI - atan(p2y/p2x) - atan( A2*sin(theta2) / (A1 + A2*cos(theta2)) );  
    sol_1[0] = RAD_TO_DEG(theta1);
    sol_1[1] = RAD_TO_DEG(theta2);
    sol_1[2] = psi - sol_1[0] - sol_1[1]; 

    // find 2nd solution
    theta2 = -theta2;
    if (p2x >= 0)
        theta1 = atan(p2y/p2x) - atan( A2*sin(theta2) / (A1 + A2*cos(theta2)) );  
    else
        theta1 = M_PI - atan(p2y/p2x) - atan( A2*sin(theta2) / (A1 + A2*cos(theta2)) );  
    sol_2[0] = RAD_TO_DEG(theta1);
    sol_2[1] = RAD_TO_DEG(theta2);
    sol_2[2] = psi - sol_2[0] - sol_2[1];  

    return true;
}

void convert_angles(double sol[3]){
    //TODO might need to do some stuff about looping angles, like % 360 etc 
    // convert theoretical/irl angles into servo angles
    sol[0] = 90 + (EQB1 - sol[0]); 
    sol[1] = 90 + (sol[1] - EQB2);
    sol[2] = 90 + (EQB3 - sol[2]);
}

bool choose_sol(double final_sol[3], double sol_1[3], double sol_2[3]){
    double score_1 = NAN;
    double score_2 = NAN;

    // TODO is there a better way to write this?
    // check if angles are in servo's range
    if (sol_1[0] <= SERVO_RANGE && sol_1[0] >= 0 
     && sol_1[1] <= SERVO_RANGE && sol_1[1] >= 0
     && sol_1[2] <= SERVO_RANGE && sol_1[2] >= 0){
        // calculate score for sol_1
        score_1 = pow(cur_angles[1] - sol_1[0],2) + pow(cur_angles[2] - sol_1[1],2) + pow(cur_angles[3] - sol_1[2],2);
    }

    if (sol_2[0] <= SERVO_RANGE && sol_2[0] >= 0 
     && sol_2[1] <= SERVO_RANGE && sol_2[1] >= 0
     && sol_2[2] <= SERVO_RANGE && sol_2[2] >= 0){
        score_2 = pow(cur_angles[1] - sol_2[0],2) + pow(cur_angles[2] - sol_2[1],2) + pow(cur_angles[3] - sol_2[2],2);
    }
    
    if (isnan(score_1) && isnan(score_2)){
        return false;      
    }

    if (isnan(score_1))
        memcpy(final_sol,sol_2,sizeof(double)*3); // must use sol_2 
    else if(isnan(score_2))
        memcpy(final_sol,sol_1,sizeof(double)*3); // must use sol_1                                         
    else if (score_1 > score_2)
        memcpy(final_sol,sol_2,sizeof(double)*3); // sol_2 less change 
    else if (score_1 <= score_2)
        memcpy(final_sol,sol_1,sizeof(double)*3); // sol_1 less change or equal amount of change
         
    return true;
}

void update_servos(double target[6]){
    // TODO is there a better way to code this?
    while (cur_angles[0] != target[0] || cur_angles[1] != target[1] || cur_angles[2] != target[2] ||
           cur_angles[3] != target[3] || cur_angles[4] != target[4] || cur_angles[5] != target[5]) {

        // turn all servos slowly at the same pace so that it looks smoother
        for (int i=0; i<6; i++){
            double diff = target[i] - cur_angles[i];
            if (diff == 0)
                continue;
            else if (diff > 0)
                cur_angles[i] += (diff > UPDATE_RATE) ? UPDATE_RATE : diff;       
            else if (diff < 0)
                cur_angles[i] += (diff < -UPDATE_RATE) ? -UPDATE_RATE : diff;       

            set_angle(i,cur_angles[i]);
        }

        delay(UPDATE_DELAY);
    }
}


void init(){
    // initialise uart with RP
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2,0x2000,0x2000,0,0,0));
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // only tx,rx, no cts,rts
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2,&uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2,TX_PIN,RX_PIN,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));


    // initialise esp32 as i2c master
    i2c_master_bus_config_t i2c_mst_config = {
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .i2c_port = I2C_NUM_0, 
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config,&bus_handle));


    // initialise pca9685 device
    i2c_device_config_t dev_cfg = {
        .device_address = 0x40,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle,&dev_cfg,&dev_handle));
    ESP_LOGI(TAG,"I2C initialised succesfully");
   

    // write PRESCALE (can only be written in sleep mode)
    uint8_t init_mode1 = read_reg(REG_MODE1) & ~MODE1_SLEEP;
    write_reg(REG_MODE1,init_mode1 | MODE1_SLEEP);
    write_reg(REG_PRESCALE,PRESCALE_VAL);
    write_reg(REG_MODE1,init_mode1);

    // turn on auto increment
    // have to wait 500 microseconds after awake from sleep before restarting
    delay(1);
    write_reg(REG_MODE1,init_mode1 | MODE1_AI | MODE1_RESTART);
    ESP_LOGI(TAG,"PRESCALE and AUTO_INCREMENT initialised");  

    // initiate servo positions
    for (int i=0; i<6; i++){
        set_angle(i,cur_angles[i]);
        delay(100);
    }
}
 

void app_main(void){
    init();

    // main loop
    while (true){
        // 1: move 2: control gripper 3: rotate base
        char choice_str[10] = "";
        uart_print("choice:\n");
        uart_fgets(choice_str,10);
        int choice = atoi(choice_str);

        
        memcpy(target_angles,cur_angles,sizeof(double)*6);

        switch (choice) { 
            case 0:
                // move end effector
                char buf[30];
                uart_print("x,y: \n");
                uart_fgets(buf,30);

                double coords[3] = {0,0,0}; // x,y,psi
                char* s; int idx = 0;
                for (s = strtok(buf,","); s; s = strtok(NULL,",")){
                    coords[idx] = atof(s);
                    idx++;
                }

                // keep trying different psi
                // TODO is there a better way of dealing with this? 
                bool is_successful = false;
                for (double psi=-180; psi <= 180; psi += 15) {
                    coords[2] = psi;
                    double sol_1[3] = {0,0,0};
                    double sol_2[3] = {0,0,0};

                    if (!ik_solve(coords,sol_1,sol_2))
                        continue;
                    

                    convert_angles(sol_1);
                    convert_angles(sol_2);

                    double final_sol[3] = {0,0,0};
                    if (!choose_sol(final_sol,sol_1,sol_2))
                        continue;
                    
                    memcpy(&target_angles[1],final_sol,sizeof(double)*3);
                    update_servos(target_angles);
                    is_successful = true;
                    break;
                }

                uart_print("success: ");
                if (is_successful) uart_print("1\n");
                else uart_print("0\n");

                break;

            case 1:
                // control gripper
                char buf2[5];
                uart_print("release/grip:\n");
                uart_fgets(buf2,5);
                if (atoi(buf2) != 0)
                    target_angles[5] = GRIPPER_CLOSE_ANGLE;    
                else
                    target_angles[5] = GRIPPER_OPEN_ANGLE;

                update_servos(target_angles);
                break;
            
            case 2:
                // rotate base
                char buf3[10];
                uart_print("base angle:\n");
                uart_fgets(buf3,10);
                double base_angle = atof(buf3);

                if (0 <= base_angle && base_angle <= 178) {
                    target_angles[0] = base_angle;
                    update_servos(target_angles);
                    uart_print("success: 1\n");
                } else {
                    uart_print("success: 0\n");
                }

                break;

            default:
                uart_print("no such choice\n");
                break;

        }
    }
}


// current config: 45 0 0 
// starting pos: 19, 12, -15

// TODO:
// make it so that after every update_servos, target_angles gets updated? prob. no need pass in vars also.
// change start position so when fall, its smoother
// rewrite case 3 success string maybe
