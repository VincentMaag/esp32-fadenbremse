/*


*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "fb_projdefs.h"
#include "fb_sd_card.h"
#include "fb_esp_time.h"
#include "fb_blinker.h"

static const char *TAG = "fb_sd_card";

#define PIN_NUM_MISO 32
#define PIN_NUM_MOSI 33
#define PIN_NUM_CLK 25
#define PIN_NUM_CS 26

// ======================================================================
// variables global to fb_sd_card.c
sdmmc_host_t host = SDSPI_HOST_DEFAULT();
sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
esp_vfs_fat_sdmmc_mount_config_t mount_config;
sdmmc_card_t *card;
//
QueueHandle_t xQueue_data_to_store;
//
const char *filenameBase = "/sdcard/data";
const char *fileType = ".csv";
char filename[30];
int filenameIndex = 1;
// --> around 50 bytes per measurement, 40byte * 1meas/s * 60/min * 60/h * 24/d =
long maxFileSize = 4000 * 1000; // maximum file size on sd card in Bytes
int maxFiles = 7;               // maximum files stored on sd card
int CycleTimeStoring = 60;      // store data every x [s]

// ======================================================================
// Initialize sd-card
void sd_card_init(void)
{
    ESP_LOGI(TAG, "Initializing SD card");
    // init params for sd card
    ESP_LOGI(TAG, "Using SPI peripheral");
    host.max_freq_khz = 800;
    slot_config.gpio_miso = PIN_NUM_MISO;
    slot_config.gpio_mosi = PIN_NUM_MOSI;
    slot_config.gpio_sck = PIN_NUM_CLK;
    slot_config.gpio_cs = PIN_NUM_CS;
    mount_config.format_if_mount_failed = true;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;
    //
    // init queue. there are 1 measurements per second, we want to store
    // 1min worth of data --> 60*1 = 60
    xQueue_data_to_store = xQueueCreate(70, sizeof(data_to_store_t));
    if (xQueue_data_to_store == NULL)
    {
        ESP_LOGE(TAG, "ERROR: Queue not created, so we cannot use it");
    }
    else
    {
        ESP_LOGI(TAG, "Queue created and ready to use");
    }
}
// ======================================================================
// Try to mount sd-card
esp_err_t sd_card_mount()
{
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set format_if_mount_failed = true.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return ret;
    }
    // Card has been initialized, print its properties
    // sdmmc_card_print_info(stdout, card);
    return ret;
}
// ======================================================================
// Try to unmount sd-card
esp_err_t sd_card_unmount()
{
    // All done, unmount partition and disable SDMMC or SPI peripheral
    esp_err_t ret = esp_vfs_fat_sdmmc_unmount();
    // ESP_LOGI(TAG, "Card unmounted");
    return ret;
}
// ======================================================================
// place new data into queue
int sd_card_post_on_queue(data_to_store_t newData)
{
    if (xQueue_data_to_store == NULL)
    {
        ESP_LOGE(TAG, "queue not initialized");
        return 0;
    }
    else if (xQueueSend(xQueue_data_to_store, (void *)&newData, 0) == pdTRUE)
    {
        // ESP_LOGI(TAG, "confirmed a post on queue");
        return 1;
    }
    else
    {
        ESP_LOGW(TAG, "full queue");
        return 2;
    }
}
// ======================================================================
// get new data from queue and write as new line in file
// return 0 if no valid name created, return 1 if valid name created
int sd_card_get_valid_filename()
{
    // temp vars
    data_to_store_t receivedData;
    FILE *f;
    struct stat st;
    //
    // get current filename
    // sprintf(filename, "%s%d%s", filenameBase, filenameIndex, fileType);
    // ESP_LOGI(TAG, "Checking files on sd-card");
    //
    // for loop
    for (filenameIndex = 1; filenameIndex < (maxFiles + 1); filenameIndex++)
    {
        // create filename
        sprintf(filename, "%s%d%s", filenameBase, filenameIndex, fileType);
        // check if file exists
        if (stat(filename, &st) == 0)
        {
            ESP_LOGI(TAG, "File exists");
            // if file exists, try to open and check file size
            f = fopen(filename, "a");
            if (f == NULL)
            {
                ESP_LOGE(TAG, "Failed to open file %s, file may be corrupted. trying next index!", filename);
                // break;
                // return 0;
            }
            else
            {
                // check file size
                fseek(f, 0, SEEK_END);
                long size = ftell(f);
                ESP_LOGI(TAG, "File size is currently: %ld", size);
                fseek(f, 0, SEEK_SET);
                fclose(f);
                // if file in boundaries use this index and exit
                if (size < maxFileSize)
                {
                    ESP_LOGI(TAG, "File size ok");
                    break;
                }
                else
                {
                    // else, if file is already to large, continue this for loop
                    ESP_LOGW(TAG, "File size to large, incrementing file index");
                }
            }
        }
        // if file doesn't exist, break and use current index
        else
        {

            ESP_LOGI(TAG, "File doesn't exist yet. Try to create with current Index");
            break;
        }
    }
    // get current filename
    // sprintf(filename, "%s%d%s", filenameBase, filenameIndex, fileType);
    // ESP_LOGE(TAG, "Using this filename: %s", filename);

    // check fileindex. if maxFiles have been reached, return 0. Else return 1
    if (filenameIndex > maxFiles)
    {
        ESP_LOGE(TAG, "max amount of files reached. Please clear data from SD-Card");
        return 0;
    }
    else
    {
        ESP_LOGI(TAG, "Using this filename: %s", filename);
        return 1;
    }
}
// ======================================================================
// get new data from queue and write as new line in file
void sd_card_write_from_queue()
{
    // temp vars
    data_to_store_t receivedData;
    FILE *f;

    // create filename
    sprintf(filename, "%s%d%s", filenameBase, filenameIndex, fileType);
    // ESP_LOGI(TAG, "Filename created: %s", filename);

    // open file
    ESP_LOGI(TAG, "Opening file %s in Append Mode and reading from queue", filename);
    // ESP_LOGI(TAG, " Before SD-Card Writing: HEAP min free: %d = free: %d\n",esp_get_minimum_free_heap_size(),esp_get_free_heap_size());
    //
    f = fopen(filename, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file %s for writing. leaving queue as it is, trying fallback file", filename);
        // if mounting fails, because filname seems to be faulty (even though feasible filename should
        // have been created), lets open a standard fallback file and try again
        sprintf(filename, "/sdcard/dataFallback.csv");
        f = fopen(filename, "a");
        // if fallback file doesn't open either, return and log nothing
        if (f == NULL)
        {
            ESP_LOGE(TAG, "Failed to open fallback file. breaking function, not logging anything");
            return;
        }
    }
    // if we are at this point, we try to write data to sd card. signal this with burst of LED
    blinker_burst(1);
    // write all available data from queue to our opened file
    while (xQueueReceive(xQueue_data_to_store, (void *)&receivedData, 0) == pdTRUE)
    {
        // ESP_LOGI(TAG, "got from queue: value1 = %.2f, value2 = %.2f", receivedData.value1, receivedData.value2);
        //  fprintf(f, "%s;%.2f;%.2f\n", receivedData.timestamp, receivedData.value1, receivedData.value2);
        fprintf(f, "%s;%.1f;%.1f;%.1f;%.1f\n", receivedData.timestamp, receivedData.value1, receivedData.value2, receivedData.value3, receivedData.value4);
    }
    // ESP_LOGI(TAG, " After SD-Card Writing: HEAP min free: %d = free: %d\n",esp_get_minimum_free_heap_size(),esp_get_free_heap_size());
    

    // close file again
    fclose(f);
}
// ======================================================================
// reset queue
bool sd_card_reset_queue()
{
    // just reset queue here
    bool ret = (bool)xQueueReset(xQueue_data_to_store);
    return ret;
}
// ======================================================================
// sd_card_task
void sd_card_task(void)
{
    // =====================================================
    // local variables
    // variable to store last tick time for exakt cycle time. Updates every cycle
    TickType_t previousWakeTime = xTaskGetTickCount();
    TickType_t cycleFrequency = 1;                    // actual cycle frequency [Hz]
    float cycleTime = 1000.0 / (float)cycleFrequency; // actual cycle time [ms] of loop
    int cycleCount = 0;
    int sdcardCycleCounter = 0;

    sd_card_init();

    while (1)
    {
        // every sd-card cycle, try to read all available data from queue and append it in file
        if (sdcardCycleCounter >= CycleTimeStoring)
        {
            sdcardCycleCounter = 0;
            // mount sd card. if mounted successfully, try to write onto file from queue and unmount again
            // if we cannot mount card, check if queue is full. If so, reset queue so that new data can be written
            // --> how to check if queue is full?? For now, if we cannot store the data, we will just reset it
            // and loose current data (ok, because we cannot store it anyway)
            if (sd_card_mount() != ESP_OK)
            {
                // signal mount fail
                set_event_bit(diagnose_event_group, BIT_DIAGNOSE_2_SD_CARD_NOT_MOUNTED);
                // try and reset queue (may fail if other tasks are trying to write at the same time)
                sd_card_reset_queue();
            }
            else
            {
                // signal successful mount
                clear_event_bit(diagnose_event_group, BIT_DIAGNOSE_2_SD_CARD_NOT_MOUNTED);
                // write to queue only if valid filenmae is created
                if (sd_card_get_valid_filename())
                {
                    sd_card_write_from_queue();
                    // clear diagnose bit
                    clear_event_bit(diagnose_event_group, BIT_DIAGNOSE_2_SD_CARD_NO_VALID_FILENAME);
                }
                else
                {
                    // signal unsuccessful filename creation
                    set_event_bit(diagnose_event_group, BIT_DIAGNOSE_2_SD_CARD_NO_VALID_FILENAME);
                }
            }
            // unmount card after loop
            sd_card_unmount();
        }
        else
        {
            sdcardCycleCounter++;
        }

        // ============================================
        if (cycleCount >= 5)
        {

            cycleCount = 0;
        }
        // ====================================================
        // delay until next cycle, increment cycle counter
        cycleCount++;
        vTaskDelayUntil(&previousWakeTime, (configTICK_RATE_HZ / cycleFrequency));
    }
}
