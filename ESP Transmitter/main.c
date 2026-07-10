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

#define ESPNOW_MAXDELAY 512

#define LOGGING 0 //Toggle to compile with/without detailed logging

//Some setup stuff
#define WIFI_CHANNEL 1 //Set the wifi channel ESPNOW operates on
#define WIFI_MODE WIFI_MODE_STA //ESPNOW can work in both station and softap mode - arbitrarily selecting station mode
#define WIFI_IF WIFI_IF_STA //Interface ESPNOW peer operates on. MUST MATCH WIFI_MODE



//More setup stuff
uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; //This is the mac we're broadcasting to (Everyone)
static const char* TAG = "Transmitter"; //Tag for logging

//GPIO Pin configuration
#define RCV_HOST    SPI2_HOST

#define GPIO_HANDSHAKE      2
#define GPIO_MOSI           23
#define GPIO_MISO           19
#define GPIO_SCLK           18
#define GPIO_CS             5



typedef struct { //Data that will get passed to ESPnow task
    uint8_t dest_mac[6]; //Mac addresses where packet will be sent
    QueueHandle_t queue; //packet queue handle
} espnow_task_params_t;

typedef struct { //Data that will be passed to SPI task
    spi_slave_transaction_t *transaction_t_pointer; //Pointer to spi_slave_transaction data structure
    QueueHandle_t queueptr; //packet queue handle
} spi_task_params_t;

//PACKET DEFINITION
#define PACKET_STRUCT_IMPLEMENTED 0 //Remove this and #if once packet_struct_t is implemented
#if PACKET_STRUCT_IMPLEMENTED
//Packet structure - for unpacking the raw bytes we get from SPI into something more usable
typedef struct {
    uint8_t header[2]; //Header bytes (0x5A 0x5A)
    uint8_t numRomi; //Number of Romis in this packet
    struct { //Data for each Romi - 16 bytes each (ID, center x, center y, heading)
        float id;
        float center_x;
        float center_y;
        float heading;
    } romi_data[NUMROMIS]; //Assuming max 10 Romis per packet
} packet_t;

#define NUMROMIS 10
#define PACKET_SIZE sizeof(packet_t)

#else
#define PACKET_SIZE 168 //Packet size (in bytes)
#define ROMI_DATA_LEN 16 //Each Romi has 16 bytes of data (ID (float), center x, center y, heading)
//Packet type - for SPI -> ESP-NOW queue
typedef struct {           
     unsigned char data[PACKET_SIZE];          //Packet data (usigned bytes)
} packet_t;
#endif





/* WiFi should start before using ESPNOW */
//Initialize wifi using example wifi init function
void wifi_init(void) //Basic wifi setup
//passing everything to ESP_ERROR_CHECK function to make sure everything inits properly
{
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


//initialize ESPnow
void espnow_init(void)
{
    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    //This is where we would init send/recieve callbacks
    
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

//Task which send stuff over ESPNow
void espnow_send_task(void *params)
{
    //Pull stuff from parameters structure
    espnow_task_params_t *espnow_task_params = (espnow_task_params_t *)params; //cast void pointer to appropriate struct pointer
    QueueHandle_t packet_queue_handle = espnow_task_params->queue;

    //Allocate buffer on the stack to pull packet into from queue
    unsigned char packet_buff[PACKET_SIZE];
    #if LOGGING
    ESP_LOGI(TAG, "ESPNow send task started");
    #endif
    while (1) { //Do this forever
        //Pull from queue. Will block until there's something in queue (or timeout is hit)
        if( xQueueReceive(packet_queue_handle, &packet_buff, portMAX_DELAY) == pdTRUE) { //Wait until something is successfully received from queue

            /*packet_buff now contains a copy of the packet. */
                /*Packet validation goes here*/
            
            #if LOGGING
            ESP_LOGI(TAG, "ESPNOW task got packet!");
            #endif
            //Now that we've gotten a packet, send it with espnow
            esp_err_t send_result = esp_now_send(espnow_task_params->dest_mac, packet_buff, PACKET_SIZE);
            if (send_result != ESP_OK) {
            #if LOGGING
            ESP_LOGE(TAG, "Error sending the packet over ESPNow: %s", esp_err_to_name(send_result));
            #endif
                //abort(); //Give up and die
                //This should be improved so sending fails gracefully
                //Setup event handling in espnow_send callback
            }
            #if LOGGING
            ESP_LOGI(TAG, "Packet sent over ESPNow!");
            #endif            
        }
    }
}
//SPI Setup and Tasks

//Callback Functions
//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 1);
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 0);
}


//SPI Initializer Function
void SPI_init(void) {
    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 1,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb
    };

    //Configuration for the handshake line
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(GPIO_HANDSHAKE),
    };

    //Configure handshake line as output
    gpio_config(&io_conf);
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    assert(spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO) == ESP_OK);
    #if LOGGING
    ESP_LOGI(TAG, "SPI slave interface initialized");
    #endif
}

bool unpack_print_packet(packet_t * packet_struct, uint8_t * numRomi, float pose_data[10][4]) { //equivalent: float (*pose_data)[4]
    //unpack packet structer
    unsigned char *packet = packet_struct->data; //pointer to first byte of packet data
    
    //Check header
    if (packet[0] != 0x5A || packet[1] != 0x5A) {
        return false;
    }

    //Get number of Romis poses to unpack
    *numRomi = packet[2];

    //make sure number of Romis is within bounds
    if (*numRomi > 10) return false;


    //Unpack packets
    for (int i = 0; i < *numRomi; i++) {
        // Process packet data
        //uint8_t id = packet[3+(i*ROMI_DATA_LEN)];      //extract ID as uint
        int offset = 3 + (i * ROMI_DATA_LEN);          //Calculate offset for current Romi
        //Uses memcpy to copy 4 bytes from packet to float array
        memcpy(&pose_data[i][0], &packet[offset], 4);             //Store ID
        memcpy(&pose_data[i][1], &packet[offset+4], 4);              //Store center x
        memcpy(&pose_data[i][2], &packet[offset+8], 4);              //Store center y
        memcpy(&pose_data[i][3], &packet[offset+12], 4);            //Store Heading
    }
    
    printf("Received packet with %d Romi poses:\n", *numRomi);
    //Print glorious data
    //printf(pose_data);
    for (int i = 0; i < *numRomi; i++) {
    printf("Romi ID: %f, Center Position (%f, %f), Heading %f\n",pose_data[i][0], pose_data[i][1], pose_data[i][2], pose_data[i][3]);
    }
    return true;
}

#if PACKET_STRUCT_IMPLEMENTED
void print_data(packet_t * packet_struct) {
    printf("Received packet with %d Romi poses:\n", packet_struct->NumRomi);
    for (int i = 0; i < packet_struct->NumRomi; i++) {
    printf("Romi ID: %f, Center Position (%f, %f), Heading %f\n",
                packet_struct->RomiData[i][0],
                packet_struct->RomiData[i][1],
                packet_struct->RomiData[i][2],
                packet_struct->RomiData[i][3]
            );
}
#endif

//Task which receives SPI data and puts it in the queue for the ESPNow task to send
void spi_slave_task(void *params) {
    
    //unpack parameters
    spi_task_params_t *spi_task_params = (spi_task_params_t *)params; //cast void pointer to appropriate struct pointer
    spi_slave_transaction_t * transaction = spi_task_params->transaction_t_pointer;
    QueueHandle_t xPacketQueue = spi_task_params->queueptr;

    //Unpack buffer pointers from transaction struct for ease of use
    unsigned char *recvbuf = transaction->rx_buffer;
    const unsigned char *sendbuf = transaction->tx_buffer;

    //Allocate buffer for position data
    #if LOGGING
    float pose_data[10][4]; //Assuming max 10 Romis, each with 4 data points (ID, center x, center y, heading)
    uint8_t numRomi = 0;
    #endif
    
    //allocate packet struct to easily intrepret packet data


    #if LOGGING
    ESP_LOGI(TAG, "SPI slave task started");
    #endif
    while (1) {
        //Clear receive buffer, set send buffer to something sane
        memset(recvbuf, 0xA5, 256);
        sprintf((char *)sendbuf, "This is the receiver, sending data for a transmission.");
        
        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc. In this specific example, we use the handshake line, pulled up by the
        .post_setup_cb callback that is called as soon as a transaction is ready, to let the master know it is free to transfer
        data.
        */

        esp_err_t ret = spi_slave_transmit(RCV_HOST, transaction, portMAX_DELAY);

        assert(ret == ESP_OK); //Check that transmission was successful

        #if LOGGING
        ESP_LOGI(TAG, "SPI transaction complete, data received");
        #endif

        //spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
        //received data from the master.

        //Check packet header
        //MORE DATA VALIDATION GOES HERE
        if (recvbuf[0] != 0x5A || recvbuf[1] != 0x5A) {
            ESP_LOGE(TAG, "Bad Packet recieved!");
        } else {
            //Copy the packet into the queue
            xQueueSend(xPacketQueue, ( void * ) recvbuf, portMAX_DELAY); //Push recieved packet (first PACKET_SIZE bytes of recvbuf) into the queue. Will block until it can be sent.
            #if LOGGING
                    ESP_LOGI(TAG, "Received packet - pushed to queue for ESPNow task");
                    printf("Packet contents (first 10 bytes): ");
                    for (int i = 0; i < 10; i++) {
                     printf("%02X ", recvbuf[i]);
                    }
                    printf("\n");
            #endif
        
           //Upack the packet and print the Romi pose data
            #if PACKET_STRUCT_IMPLEMENTED
            packet_t * packet_struct = (packet_t *)recvbuf; //Cast the raw byte buffer to a pointer to our packet struct for easier access to fields
                #if LOGGING
                print_data(packet_struct);
                #endif
            #else
            #if LOGGING
            bool fred = unpack_print_packet((packet_t *)recvbuf, &numRomi, pose_data);
            #endif
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
    SPI_init();
    #if LOGGING
    ESP_LOGI(TAG, "Initialization complete, starting tasks...");
    #endif

    //Init DMA buffers for SPI messaging
    unsigned char *sendbuf = spi_bus_dma_memory_alloc(RCV_HOST, 256, 0);
    unsigned char *recvbuf = spi_bus_dma_memory_alloc(RCV_HOST, 256, 0);
    assert(sendbuf && recvbuf);

    //Init spi transaction data structure and set to zero
    spi_slave_transaction_t *transaction = malloc(sizeof(spi_slave_transaction_t)); 
    
    //Set up an SPI transaction of 256 bytes to send/receive
    transaction->length = 256 * 8;
    transaction->tx_buffer = sendbuf;
    transaction->rx_buffer = recvbuf;
    #if LOGGING
    ESP_LOGI(TAG, "SPI transaction buffers allocated and configured");
    #endif

    //Init queue for passing packets between tasks
    QueueHandle_t packet_queue = xQueueCreate(1, sizeof(packet_t));
    #if LOGGING
    ESP_LOGI(TAG, "Queue for inter-task communication created");
    #endif

    //Setup structure to pass parameters to the task that will send messages
    espnow_task_params_t *espnow_task_params = malloc(sizeof(espnow_task_params_t)); //allocate on heap b/c we're giving it to another function
    memset(espnow_task_params, 0, sizeof(espnow_task_params_t));           //Init structure to zero
    memcpy(espnow_task_params->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN); //Copy mac address we want to braodcast to
    espnow_task_params->queue = packet_queue;                              //Give queue handle to structure so task can pull packets


    //Setup structure to pass parameters to the SPI reciever task
    spi_task_params_t *spi_task_params = malloc(sizeof(spi_task_params_t));
    spi_task_params->transaction_t_pointer = transaction;       //Pointer to previously allocated spi transaction struct
    spi_task_params->queueptr = packet_queue;                    //Packet queue handle

    #if LOGGING
    ESP_LOGI(TAG, "Task parameter structures allocated and configured");
    #endif


    //Create a task to send ESPNOW messages and pass pointer to task parameters structure to it
    xTaskCreatePinnedToCore(espnow_send_task, "ESPNow Send Task", 4096, espnow_task_params, 4, NULL, 0); //Pin to core 0 which is where other wifi stuff runs

    //Create a task to send ESPNOW messages and pass pointer to SPI transaction config to it
    xTaskCreatePinnedToCore(spi_slave_task, "SPI Recieve Task", 8192, spi_task_params, 4, NULL, 1); //Pin to core 1 to keep it off the wifi core
    #if LOGGING
    ESP_LOGI(TAG, "Tasks created, entering main loop...");
    #endif
    print_tasks(); //Peek behind the scenes

}
