#include "mpu_6050.h"
#include "mcu_driver.h"
#include "FreeRTOSIncludes.h"

void vIMURead(void* args)
{
    imu_init();// can't call this in mcu_init because this uses a vTaskDelay which can only be called after starting the scheduler
    
    struct imu_data imu_vals;
    
    while (1)
    {
        imu_read(&imu_vals);
        vTaskDelay(pdMS_TO_TICKS(500));
        io_set_output((io_e)PA5, IO_OUTPUT_HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        io_set_output((io_e)PA5, IO_OUTPUT_LOW);
    }

}


int main()
{
    mcu_init();


    BaseType_t imu_ok = xTaskCreate(
        vIMURead,
        "IMURead",
        256,
        NULL,
        1,
        NULL
    );


    if (imu_ok != pdPASS)
    {
        while (1)
        {
            // Task creation failed
        }
    }

    vTaskStartScheduler();

    // Should never get here unless there is not enough heap
    while (1)
    {
    }
}
