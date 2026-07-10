#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define ESPNOW_MAXDELAY 512

#define LOGGING 0 //Toggle to compile with/without detailed logging

//Some setup stuff
#define WIFI_CHANNEL 1 //Set the wifi channel ESPNOW operates on
#define WIFI_MODE WIFI_MODE_STA //ESPNOW can work in both station and softap mode - arbitrarily selecting station mode
#define WIFI_IF WIFI_IF_STA //Interface ESPNOW peer operates on. MUST MATCH WIFI_MODE

//Define UART end transaction pattern
#define UART_END_PATTERN 0x0A//This is the pattern char we send over UART to signal the end of a packet transmission to the STM. The STM should listen for this pattern to know when a full packet has been received and can be processed

//More setup stuff
uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; //This is the mac we're broadcasting to (Everyone)
static const char* TAG = "Reciever"; //Tag for logging

//GPIO Pin configuration
#define RCV_HOST    SPI2_HOST
#define GPIO_UART_TX      17
#define GPIO_UART_RX      16

//Define Inter-task communication flags
#define UART_TRANS_NOT_IN_PROG   (1U << 0)
#define DATA_UPDATE_NOT_IN_PROG  (1U << 0)



typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_cb_data_t; //This is the format of the data that is passed to the espnow callback function


//PACKET CONFIG
#define NUMROMIS 50 //Max ID number that can occur/ max number of Romis we can have in a packet
//Pose data type - for easily storing pose data in memory
typedef struct {
            struct{ //Data for each Romi - 12 bytes each (center x, center y, heading)
                float center_x;
                float center_y;
                float heading;
    } romi_data[NUMROMIS]; //Assuming max 10 Romis per packet
} pose_data_t;
#define PACKETSIZE NUMROMIS*16 + 3 //packet size (pose data plus header and num of romis detected)


typedef struct { //Data that will get passed to Watchdog task
    pose_data_t *pose_data; //Pointer to pose data heap var
    EventGroupHandle_t event_group; //Event group handle
} packet_watchdog_task_params_t;

typedef struct { //Data that will get passed to UART task
    pose_data_t *pose_data; //Pointer to pose data heap var
    EventGroupHandle_t event_group; //Event group handle
} uart_task_params_t;


//print packer data helper function
//void print_data(packet_t * packet_struct) {
//    if (packet_struct->header[0] != 0x5A || packet_struct->header[1] != 0x5A) {
//        printf("Invalid packet header! Got %02X %02X\n", packet_struct->header[0], packet_struct->header[1]);
//        return;
//    }
//    printf("Received packet with %d Romi poses:\n", packet_struct->numRomi);
//    for (int i = 0; i < packet_struct->numRomi; i++) {
//    printf("Romi ID: %f, Center Position (%f, %f), Heading %f\n",
//                packet_struct->romi_data[i].id,
//                packet_struct->romi_data[i].center_x,
//                packet_struct->romi_data[i].center_y,
//                packet_struct->romi_data[i].heading
//            );
//    }
//}


//SKETCHY!!
//Initialize queue handle as global variable so we can access it in callbacks
//Stupid esp_now_register_recieve_cb doesn't allow passing args to callback
//I thought this was a bad idea but this is what the ESP example code does
QueueHandle_t packet_queue_handle = NULL;
QueueHandle_t uart_event_queue_handle = NULL;



/* WiFi should start before using ESPNOW */
//Initialize wifi using example wifi init function
void wifi_init(void) { //Basic Wifi setup
    ESP_ERROR_CHECK(esp_netif_init());                            //Initialize underlying TCP/IP stack
    ESP_ERROR_CHECK(esp_event_loop_create_default());             //Create default event loop
    esp_netif_create_default_wifi_sta();                          //Apparently this is needed for initializing wifi in station mode
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();          //Set wifi config to default
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );                       //Allocate resources for WiFi driver and start WiFi task
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );    //Set the WiFi API configuration storage type (Set to RAM)
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE) );          //Set wifi mode
    ESP_ERROR_CHECK( esp_wifi_start());                           //Start wifi with current configuration
    ESP_ERROR_CHECK( esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    #if LOGGING
    ESP_LOGI(TAG, "WiFi initialized");
    #endif
}

//Define ESPnow call back function
//This function throws the packet into a queue. It needs to be lightweight
void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len){
    #if LOGGING
    ESP_LOGI(TAG, "Got packet in callback!");
    ESP_LOGI(TAG, "Data length: %d", len);
    ESP_LOGI(TAG, "Data in callback: %02X %02X %02X %02X %02x %02x %02x %02x %02x %02x %02x", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10]); //Print first 11 bytes of packet for debugging
    #endif
    xQueueOverwrite(packet_queue_handle, ( void * const ) data); //Push packet data into the queue. Will overwrite so we only keep most recent value
}

//initialize ESPnow
void espnow_init(void)
{
    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    //This is where we would init send/recieve callbacks
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) ); //Register callback function to handle packets
    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t peer = {0};     //Create peer information structure and fill with zeros
    //Fill relevant structure fields
    peer.channel = WIFI_CHANNEL;
    peer.ifidx = WIFI_IF;
    peer.encrypt = false; //Disable encryption (not necessary)
    memcpy(peer.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN); //Copy mac address from broadcast mac structure to appropriate field
    ESP_ERROR_CHECK( esp_now_add_peer(&peer) ); // add peer to the peer list (pass pointer to peer struct)
    #if LOGGING
    ESP_LOGI(TAG, "ESPNOW initialized");
    #endif
}

//UART Setup
void UART_init(void) {
    //Configure UART parameters
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //define these as variables for readibility (they'll disappear when this function exits)
    int rx_buff_size = 256;
    int tx_buff_size = 256;
    int freertos_queue_size = 5;
    int interrupt_flag = 0; //Default - allows driver to set interrupt flags

    esp_err_t driver_install = uart_driver_install(UART_NUM_2, rx_buff_size, tx_buff_size, freertos_queue_size, &uart_event_queue_handle, interrupt_flag);
    assert(driver_install == ESP_OK);
    esp_err_t param_set = uart_param_config(UART_NUM_2, &uart_config);
    assert(param_set == ESP_OK);
    esp_err_t pin_set = uart_set_pin(UART_NUM_2, GPIO_UART_TX, GPIO_UART_RX, -1, -1);
    assert(pin_set == ESP_OK);

    //Setup UART driver to trigger event when end of message is detected.
    uint8_t num_of_end_chars = 3; //Require 3 terminator bytes so one frame only triggers one pattern event
    esp_err_t pattern_enable = uart_enable_pattern_det_baud_intr(UART_NUM_2, UART_END_PATTERN, num_of_end_chars, 9, 0, 0);
    assert(pattern_enable == ESP_OK);
    esp_err_t pattern_queue_reset = uart_pattern_queue_reset(UART_NUM_2, 8);
    assert(pattern_queue_reset == ESP_OK);

    assert(uart_event_queue_handle != NULL);


    //Enable pull-ups on UART lines so we don't detect rogue pulses when nothing is connected.
    gpio_set_pull_mode(GPIO_UART_TX, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_UART_RX, GPIO_PULLUP_ONLY);

    #if LOGGING
    ESP_LOGI(TAG, "UART interface initialized");
    #endif
}

void UART_event_task(void * params)
{
    //Pull stuff from parameters structure
    uart_task_params_t *uart_task_params = (uart_task_params_t *)params; //cast void pointer to appropriate struct pointer
    pose_data_t* pose_data_buff = uart_task_params->pose_data;
    EventGroupHandle_t event_group = uart_task_params->event_group;

    uart_event_t event; //allocate variable to get uart event from queue
    size_t buffer_len; //allocate var to get amount of uart received data buffered
    int bytes_read = 0;
    unsigned char request[4] = {0};
    uint8_t requested_id = 0;
    unsigned char tx_buff[12] = {0}; //Buffer to hold response data (pose data for 1 Romi)

    while (1) {
        if (xQueueReceive(uart_event_queue_handle, &event, portMAX_DELAY) == pdTRUE) { //Wait until something is successfully received from queue
            //DESIGN DECISION: FORCE NICE BEHAVIOR FROM STM.
            //IF MORE THAN 4 BYTES IN BUFFER, CLEAR
            //IF BUFFER FULL, CLEAR
            #if LOGGING
            ESP_LOGI(TAG, "UART event!");
            #endif
            //Only handle pattern detected or fifo overflow.
            if (event.type == UART_PATTERN_DET) { //If custom end of transmission detected
                //Make sure there's not too much in the buffer
                uart_get_buffered_data_len(UART_NUM_2, &buffer_len);
                if (buffer_len != 4) {
                    #if LOGGING
                    ESP_LOGI(TAG, "Got RX buffer length of %d, ignoring", buffer_len);
                    #endif
                    uart_flush_input(UART_NUM_2); //clear buffer
                    continue;
                };
                //Read from UART RX buffer and see if anything useful is there
                bytes_read = uart_read_bytes(UART_NUM_2, request, sizeof(request), pdMS_TO_TICKS(20)); //Read 4 bytes
                if (bytes_read != sizeof(request)) {
                    #if LOGGING
                    ESP_LOGI(TAG, "UART read returned %d bytes (expected %d), flushing RX", bytes_read, sizeof(request));
                    #endif
                    uart_flush_input(UART_NUM_2);
                    continue;
                }
                memcpy(&requested_id, request, sizeof(uint8_t)); //First byte of request is the ID of the Romi the STM is requesting data for. Copy to variable
                //Sanity check
                if (requested_id >= NUMROMIS) {
                    #if LOGGING
                    ESP_LOGI(TAG, "Got request for Romi ID [%d], which is out of bounds. Ignoring.", requested_id);
                    #endif
                    continue;
                };
                #if LOGGING
                ESP_LOGI(TAG, "Got request for Romi ID [%d]", requested_id);
                #endif
                //Now we know the ID of the Romi the STM is requesting data for. We just need to copy the appropriate data into the request buffer and send it back over UART
                xEventGroupWaitBits(event_group, DATA_UPDATE_NOT_IN_PROG, pdFALSE, pdFALSE, portMAX_DELAY); //Wait until watchdog isn't updating data (data is safe to read)
                //Clear flag to indicate UART transaction in progress so watchdog won't update data
                xEventGroupClearBits(event_group, UART_TRANS_NOT_IN_PROG);
                memcpy(tx_buff, &pose_data_buff->romi_data[requested_id], sizeof(pose_data_buff->romi_data[requested_id]));
                #if LOGGING
                ESP_LOGI(TAG, "tx buffer dump: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                         tx_buff[0], tx_buff[1], tx_buff[2], tx_buff[3], tx_buff[4], tx_buff[5], tx_buff[6], tx_buff[7], tx_buff[8], tx_buff[9], tx_buff[10], tx_buff[11]);
                #endif

                xEventGroupSetBits(event_group, UART_TRANS_NOT_IN_PROG); //Set flag to indicate UART transaction not in progress so watchdog can update data again
                uart_write_bytes(UART_NUM_2, (const char *)tx_buff, sizeof(tx_buff)); //Write response payload
                //term_written = uart_write_bytes(UART_NUM_2, (const char *)&tx_terminator, 1); //Frame terminator for STM parser
            } else if (event.type == UART_FIFO_OVF) { //If RX buffer overflow detected or buffer full
                #if LOGGING
                ESP_LOGI(TAG, "RX fifo overflow");
                #endif
                uart_flush_input(UART_NUM_2); //Clear buffer
            };
        };
    }
}
void packet_watchdog_task(void * params)
{
    //Pull stuff from parameters structure
    packet_watchdog_task_params_t *packet_watchdog_task_params = (packet_watchdog_task_params_t *)params; //cast void pointer to appropriate struct pointer
    pose_data_t* pose_data_buff = packet_watchdog_task_params->pose_data;
    EventGroupHandle_t event_group = packet_watchdog_task_params->event_group;

    //Allocate buffer on the stack to pull packet into from queue
    unsigned char packet_buff[PACKETSIZE];
    memset(packet_buff, 0, sizeof(packet_buff)); //init to zeros
    uint8_t packet_depth = 0; //Useful depth of packet (3rd byte of packet)
    float current_id_f = 0.0; //current Romi ID
    uint8_t current_id = 0; //current Romi ID
    #if LOGGING
    ESP_LOGI(TAG, "Packet Watchdog task started");
    //Init vars for watchdog update profiling
    uint64_t start = 0;
    uint64_t count = 0;
    uint64_t runsum = 0;
    uint64_t end = 0;
    start = esp_timer_get_time();
    #endif
    while (1) { //Do this forever
        //Pull from queue. Will block until there's something in queue (or timeout is hit)
        if( xQueueReceive(packet_queue_handle, &packet_buff, portMAX_DELAY) == pdTRUE) { //Wait until something is successfully received from queue
            /*packet_buff now contains a copy of the packet. */
            #if LOGGING
            ESP_LOGI(TAG, "Packet Watchdog task got packet!");
            start = esp_timer_get_time();
            count++;
            #endif

            /*Packet validation goes here*/
            if (!((packet_buff[0] == 0x5A) && (packet_buff[1] == 0x5A))) { //if packet header not detected
                #if LOGGING
                ESP_LOGI(TAG, "Invalid packet header! Got %02X %02X", packet_buff[0], packet_buff[1]);
                #endif
                continue;
            };

            #if LOGGING
            ESP_LOGI(TAG, "packet dump incoming!");
            for (size_t i = 0; i < sizeof(packet_buff); i++)
            {
                printf("%02x ", packet_buff[i]);
            };
            printf("\n");
            #endif


            //Block if UART transaction is in progress
            xEventGroupWaitBits(event_group, UART_TRANS_NOT_IN_PROG, pdFALSE, pdFALSE, portMAX_DELAY);
            //event group handle, bit to wait to be set, don't clear bit on function return, don't wait for multiple bits, max timeout

            //Set data write flag so uart task doesn't try to write while we're modifying data
            xEventGroupClearBits(event_group, DATA_UPDATE_NOT_IN_PROG);

            //Now read the packt into heap variable
            #if LOGGING
            printf("Packet received with header %02X %02X and depth %d\n", packet_buff[0], packet_buff[1], packet_buff[2]);
            #endif
            packet_depth = packet_buff[2];
            for (int i = 0; i < packet_depth; i++){
                memcpy(&current_id_f, &packet_buff[3+(16*i)], sizeof(float)); //copy id to current_id float variable
                current_id = (uint8_t)current_id_f; //convert to a unsigned integer
                #if LOGGING
                ESP_LOGI(TAG, "Romi ID in packet: %d", current_id);
                #endif
                //Sanity check
                if (current_id >= NUMROMIS) {
                    #if LOGGING
                    ESP_LOGI(TAG, "Got Romi ID [%d] which is out of bounds, skipping", current_id);
                    #endif
                    continue; //Skip this set of data
                };
                //Now, here's where the magic happens. We're gonna copy the data into the heap variable IN THE CORRECT ORDER
                //So, pose_data_buff[1] is the data for romi w/ ID = 1, etc.
                //This is why we can't have current_id > NUMROMIS
                memcpy(&pose_data_buff->romi_data[current_id].center_x, &packet_buff[3+(16*i)+4], sizeof(float));
                memcpy(&pose_data_buff->romi_data[current_id].center_y, &packet_buff[3+(16*i)+8], sizeof(float));
                memcpy(&pose_data_buff->romi_data[current_id].heading, &packet_buff[3+(16*i)+12], sizeof(float));
            };
            #if LOGGING
            ESP_LOGI(TAG, "ensure memcpy worked properly:");
            const uint8_t *pose_bytes = (const uint8_t *)pose_data_buff;
            for (size_t i = 0; i < sizeof(*pose_data_buff); i++)
            {
                printf("%02x ", pose_bytes[i]);
            };
            printf("\n");
            #endif
            //Now that pose_data_buff has the latest data, set flag and go back to waiting
            xEventGroupSetBits(event_group, DATA_UPDATE_NOT_IN_PROG);

            #if LOGGING
            end = esp_timer_get_time();
            runsum += end - start;
            if ((count % 10) == 0) {
                printf("Average watchdog hotloop time: %lld\n", runsum/count);
            }
            ESP_LOGI(TAG, "Packet copied to UART buffer");
            //print_data((packet_t *)packet_buff); //Unpack and print packet data for debugging
             #endif
        }
    }
}

//from chat to see info about tasks
void print_tasks(void)
{
    char buffer[1024];
    vTaskList(buffer);
    printf("Task Name\tState\tPrio\tStack\tNum\n");
    printf("%s\n", buffer);
}


void app_main(void)
{
    //Need to init NVS before wifi
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //Init Wifi, ESPnow, SPI
    wifi_init();
    espnow_init();
    #if LOGGING
    ESP_LOGI(TAG, "Initialization complete, starting tasks...");
    #endif

    //Init buffer for Pose data
    pose_data_t *pose_data_ptr =(pose_data_t *)malloc(sizeof(pose_data_t));
    memset(pose_data_ptr, 0, sizeof(*pose_data_ptr)); //init to zeros


    //Init queue for passing packet from ESP callback to Watchdog task
    QueueHandle_t packet_queue = xQueueCreate(1, PACKETSIZE);
    //Write to global queue handle so we can access it in the callback function
    packet_queue_handle = packet_queue;
    //Initialize UART driver/event queue (uart_event_queue_handle is assigned by driver install)
    UART_init();

    //Init Event group for inter-task communication
    EventGroupHandle_t event_group = xEventGroupCreate();
    //Set both flags
    xEventGroupSetBits(event_group, (UART_TRANS_NOT_IN_PROG | DATA_UPDATE_NOT_IN_PROG));

    #if LOGGING
    ESP_LOGI(TAG, "Queues and event group for inter-task communication created");
    #endif

    //Setup structure to pass parameters to the Packet Watchdog task
    packet_watchdog_task_params_t *packet_watchdog_task_params = malloc(sizeof(packet_watchdog_task_params_t));
    //Set pointer to SPI transaction send buffer in the parameters structure so the Packet Watchdog task can access it
    packet_watchdog_task_params->pose_data = pose_data_ptr; //Pointer to buffer of pose data
    packet_watchdog_task_params->event_group = event_group; //Handle to event group


    //Setup structure to pass parameters to the SPI reciever task
    uart_task_params_t *uart_task_params = malloc(sizeof(uart_task_params_t));
    uart_task_params->pose_data = pose_data_ptr; //Pointer to buffer of pose data
    uart_task_params->event_group = event_group;

    #if LOGGING
    ESP_LOGI(TAG, "Task parameter structures allocated and configured");
    #endif


    //Create a task to send ESPNOW messages and pass pointer to task parameters structure to it
    xTaskCreatePinnedToCore(packet_watchdog_task, "Packet Watchdog Task", 4096, packet_watchdog_task_params, 4, NULL, 1); //Pin to core 1 to keep it off the wifi core
    //Create a task to send ESPNOW messages and pass pointer to SPI transaction config to it
    xTaskCreatePinnedToCore(UART_event_task, "UART Event Task", 8192, uart_task_params, 4, NULL, 1); //Pin to core 1 to keep it off the wifi core
    #if LOGGING
    ESP_LOGI(TAG, "Tasks created, entering main loop...");
    #endif
    print_tasks(); //Peek behind the scenes

}
