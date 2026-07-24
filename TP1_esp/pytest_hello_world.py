//--------Librerías--------
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//Verifica que el build se configure con el middleware Micro XRCE-DDS.
//Si esto es así (usualmente es así) se habilita el header para setear-
//-IP/puerto del agente y opciones RMW.
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

//Macros de chequeo y configuración:
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define MICRO_ROS_APP_STACK 16000
#define MICRO_ROS_APP_TASK_PRIO 5

//Log tag y objetos globales:
static const char *TAG = "micro_ros";
rcl_publisher_t publisher;  //estos dos son gloables para que el cb de timer pueda publicar.
std_msgs__msg__Int32 msg;

//cb del timer: 

//* Se invoca periódicamente por el executor.
//* Publica msg en el tópico del publisher y luego incrementa msg.data.

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);    //silencia warnings.
	if (timer != NULL) {
        //publica:
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		ESP_LOGI(TAG, "Publishing: %d", (int)msg.data);
        msg.data++;
	}
}

//Tarea principal de micro-ROS:
void micro_ros_task(void * arg)
{
    //configuración e inicialización de la gestión de mem:
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

    //configuración del trnasporte (si XRCE-DDS)
    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        //obtiene rmw_options embebidas en init_options:
        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	    //Fija la IP y puerto del micro-ROS Agent (por macros de sdkconfig, definidas vía menuconfig).
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    #endif

	//Crea el support que contiene el contexto ROS2 y config necesarias para crear nodos, timers, etc.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	//Nodo:
	rcl_node_t node;
	//crea el nodo llamado esp32_publisher.
    RCCHECK(rclc_node_init_default(&node, "esp32_publisher", "", &support));
    ESP_LOGI(TAG, "Nodo creado correctamente");
	//Se crea el publicador del tipo std_msgs/msg/Int32 en el tópico datos_int32
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"datos_int32"));
    ESP_LOGI(TAG, "Publisher creado correctamente.");
	//Crea un timmer de 1000ms y cb timer_callback.
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback,
		true));

	//Crea el executor con capacidad para 1 handle (el timer en este caso).
    //Si quisieramos agregar suscripciones, servicios, etc. se aumenta la capacidad.
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	//Registra el timer en el executor:
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

	msg.data = 0;

    //Loop principal.
	while(1){
        //Procesa eventos no bloqueantes, con un timeout de 100ms para esperar.
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000); //timer para no saturar la cpu.
	}
	//Liberación de recursos: 
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));
  	vTaskDelete(NULL);
}

void app_main(void)
{
    //Si el transporte está configurado para wifi, inicializa la interfaz de red para micro-ros.
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif
    //Crea la tarea FreeRTOS con el stack y prioridad definidas.
    xTaskCreate(micro_ros_task,
            "micro_ros_task",
            MICRO_ROS_APP_STACK,
            NULL,
            MICRO_ROS_APP_TASK_PRIO,
            NULL);
}