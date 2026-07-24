#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "motor_driver.h"
#include "ultrasonic.h"
#include "encoder.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <interfaces/srv/control_robot.h>


#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define MICRO_ROS_APP_STACK 24000
#define MICRO_ROS_APP_TASK_PRIO 5

/* ── Sensor ultrasónico ─────────────────────────────────────── */
#define PIN_TRIG        GPIO_NUM_4
#define PIN_ECHO        GPIO_NUM_19
#define MAX_DISTANCE_CM 400
#define AVG_SAMPLES     5
#define ECHO_TIMEOUT_MS 30

/* ── Cinemática diferencial ─────────────────────────────────── */
#define WHEEL_SEPARATION  0.15f
#define MAX_LINEAR_VEL    1.0f
#define MAX_ANGULAR_VEL   1.0f
#define MAX_WHEEL_SPEED   (MAX_LINEAR_VEL + MAX_ANGULAR_VEL * (WHEEL_SEPARATION / 2.0f))

#define DOMAIN_ID 12

static const char *TAG = "micro_ros";

//publicadores
rcl_publisher_t publisher;
rcl_publisher_t publisher_encoder;

//subscriber
rcl_subscription_t subscriber;


std_msgs__msg__Float32 ultrasonic_msg;
std_msgs__msg__Float32 enc_izq_msg;
std_msgs__msg__Float32 enc_der_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

/* Inicialización de pines de motores */
motor_t motor_right = { .channel = LEDC_CHANNEL_0, .pin_pwm = GPIO_NUM_18, .pin_dir = GPIO_NUM_5  };
motor_t motor_left  = { .channel = LEDC_CHANNEL_1, .pin_pwm = GPIO_NUM_16, .pin_dir = GPIO_NUM_17 };

static uint16_t contador = 0;
//volatile bool motor_flag = true;

rcl_service_t service;
interfaces__srv__ControlRobot_Request service_req;
interfaces__srv__ControlRobot_Response service_res;


/* ── Callbacks micro-ROS ────────────────────────────────────── */

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    (void) last_call_time;
    if (timer == NULL) 
        return;
    else{
        ultrasonic_msg.data = ultrasonic_read_filtered_cm(AVG_SAMPLES,PIN_TRIG,PIN_ECHO, ECHO_TIMEOUT_MS, MAX_DISTANCE_CM);
        enc_izq_msg.data = encoder_get_rpm_left();
        enc_der_msg.data = encoder_get_rpm_right();
        RCSOFTCHECK(rcl_publish(&publisher,&ultrasonic_msg,NULL));
        RCSOFTCHECK(rcl_publish(&publisher_encoder, &enc_izq_msg,NULL));
        RCSOFTCHECK(rcl_publish(&publisher_encoder, &enc_der_msg,NULL));
        ESP_LOGI(TAG, "Publishing: %d", (int)ultrasonic_msg.data);
        ESP_LOGI(TAG, "Publishing: left -> %d, right -> %d"  , (int)enc_izq_msg.data, (int)enc_der_msg.data );
    }

}

void subscription_callback(const void *msgin)
{
    // Recepción del comando de velocidad desde el tópico
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    // CONVERSIÓN CINEMÁTICA DEL COMANDO DE VELOCIDAD RECIBIDO A LA "VELOCIDAD" DE CADA RUEDA 
    float vel_lineal = msg -> linear.x;
    float vel_angular = msg -> angular.z;

    // CONVERSIÓN CINEMÁTICA DEL COMANDO DE VELOCIDAD RECIBIDO A LA "VELOCIDAD" DE CADA RUEDA 
    float vel_rueda_izq = vel_lineal - (vel_angular * WHEEL_SEPARATION/2.0);
    float vel_rueda_der = vel_lineal + (vel_angular * WHEEL_SEPARATION/2.0);

    // SETEAR VELOCIDAD DE MOTORES (% DUTY Y DIRECCION)

    uint32_t pwm_right = (uint32_t)((fabsf(vel_rueda_der) / MAX_WHEEL_SPEED) * MAX_DUTY);
    uint32_t pwm_left  = (uint32_t)((fabsf(vel_rueda_izq)  / MAX_WHEEL_SPEED) * MAX_DUTY);

    if(vel_rueda_izq > 0.0){
            motor_set(&motor_left, pwm_left, MOTOR_FWD);
        } else {
            motor_set(&motor_left, pwm_left, MOTOR_BWD);
        }
        if(vel_rueda_der > 0.0){
            motor_set(&motor_right, pwm_right, MOTOR_FWD);
        } else {
            motor_set(&motor_right, pwm_right, MOTOR_BWD);
    }
    

   /* if(!motor_flag){
        if(vel_rueda_izq > 0.0){
            motor_set(&motor_left, pwm_left, MOTOR_FWD);
        } else {
            motor_set(&motor_left, pwm_left, MOTOR_BWD);
        }
        if(vel_rueda_der > 0.0){
            motor_set(&motor_right, pwm_right, MOTOR_FWD);
        } else {
            motor_set(&motor_right, pwm_right, MOTOR_BWD);
        }
    }else{
        motor_set(&motor_left, 0,MOTOR_BWD);
        motor_set(&motor_right, 0,MOTOR_BWD);
    
    }*/
   
}

/*void service_callback(const void* request, void *response){
    interfaces__srv__ControlRobot_Response * res = (interfaces__srv__ControlRobot_Response *) response;
    contador += 1;
    if(contador %2 == 0){
        res -> brake = 1;
        motor_flag = true;
        ESP_LOGI(TAG, "Freno activado");
    } else{
        res ->brake = 0;
        motor_flag = false;
        ESP_LOGI(TAG, "Freno desactivado");
    }
    ESP_LOGI(TAG, "Entre al servicio");

}

*/
/* ── Tarea micro-ROS ────────────────────────────────────────── */

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t  support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    // Setear el DOMAIN_ID 
    RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "node_micro_ros", "/", &support));
    ESP_LOGI(TAG, "Nodo creado correctamente");

    // Inicialización del publicador
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "sensor_data"));

    // Inicialización del publicador
    RCCHECK(rclc_publisher_init_default(
        &publisher_encoder,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "encoder_data"));

    // Inicialización del suscriptor
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    // Inicializo el servicio
    /*RCCHECK(rclc_service_init_default(&service,
        &node, 
        ROSIDL_GET_SRV_TYPE_SUPPORT(interfaces, srv, ControlRobot), 
        "control_robot"));*/   
    


    // Inicialización del Timer
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback,
        true));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1000)));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg,
                                           &subscription_callback, ON_NEW_DATA));
                                           
    //RCCHECK(rclc_executor_add_service(&executor, &service, &service_req,
    //&service_res, &service_callback));

    ESP_LOGI(TAG, "Todo creado correctamente");
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    if (ultrasonic_init(PIN_TRIG, PIN_ECHO) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar el ultrasonido");
        return;
    }

    motors_init(&motor_right, &motor_left);
    encoders_init();

    xTaskCreate(micro_ros_task, "micro_ros_task",
                MICRO_ROS_APP_STACK, NULL, MICRO_ROS_APP_TASK_PRIO, NULL);
}