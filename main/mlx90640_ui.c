
#include <mlx90640.h>
#include "mlx90640_ui.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>    
#include <limits.h>
#include <string.h>

#define FPS 16
#define FRAME_TIME_MICROS (1000000/FPS)

// Despite the framerate being ostensibly FPS hz
// The frame is often not ready in time
// This offset is added to the FRAME_TIME_MICROS
// to account for this.
#define OFFSET_MICROS 850
#define MLX_I2C_ADDR 0x33

// 0 for normal
// 1 for 90
// 2 for 180
// 3 for 270
#define MLX90640_ROTATION 1



    paramsMLX90640 mlx90640;


static SemaphoreHandle_t mlx90640_mutex = NULL; 
static StaticSemaphore_t xMutexBuffer;

TaskHandle_t  MLX90640Thread;

thermal_image_t image;

void thermalCameraTask(void *param);

void thermal_getframe(thermal_image_t img)
{
        xSemaphoreTake(mlx90640_mutex,portMAX_DELAY);
    memcpy(img,image,sizeof(thermal_image_t));
        xSemaphoreGive(mlx90640_mutex);
}

float thermal_getTempAtPoint(uint32_t x, uint32_t y)
{
    if (x>MLX90640_SENSOR_W)
    return 0;
    if (y>MLX90640_SENSOR_H)
    return 0;
            uint32_t pos = MLX90640_SENSOR_W * y + x;
    return image[pos];
}


    #define NUM_COLORS  5
//var keys = ["white", "gold", "#c07", "#20008c", "black"];
    static float color[NUM_COLORS][3] = { {0,0,0},{32.0/255.0,0,140.0/255.0} , {204.0/255.0,0.0/255.0,119.0/255.0}, {1,221.0/255.0,0}, {1,1,1} };
   // static float color[NUM_COLORS][3] = {  {1,1,1} , {32.0/255.0,0,140.0/255.0}, {204.0/255.0,0.0/255.0,119.0/255.0}, {1,221.0/255.0,0},{0,0,0}};

void colorPixel(thermal_color_t *pixelOut, float pixelIn, float maxTemp, float minTemp)
{
    int idx1, idx2;
    float fractBetween = 0;
    float vrange = maxTemp-minTemp;
    pixelIn -= minTemp;
    pixelIn /= vrange;
    if(pixelIn <= 0) {idx1=idx2=0;}
    else if(pixelIn >= 1) {idx1=idx2=NUM_COLORS-1;}
    else
    {
        pixelIn *= (NUM_COLORS-1);
        idx1 = floor(pixelIn);
        idx2 = idx1+1;
        fractBetween = pixelIn - (float)idx1;
    }

    int ir, ig, ib;

    ir = (int)((((color[idx2][0] - color[idx1][0]) * fractBetween) + color[idx1][0]) * 255.0);
    ig = (int)((((color[idx2][1] - color[idx1][1]) * fractBetween) + color[idx1][1]) * 255.0);
    ib = (int)((((color[idx2][2] - color[idx1][2]) * fractBetween) + color[idx1][2]) * 255.0);

    
    pixelOut->r = ir;
    pixelOut->g = ig;
    pixelOut->b = ib;
}

void thermal_colorImage(thermal_image_t image, thermal_color_image_t colorImage)
{
    static float maxTemp = 50;
    static float minTemp = 0;
    float newMaxTemp = -INT_MAX;
    float newMinTemp = INT_MAX;
   for(int y = 0; y < MLX90640_SENSOR_H; y++){
        for(int x = 0; x < MLX90640_SENSOR_W; x++){
            uint32_t pos = MLX90640_SENSOR_W * y + x;
            //uint32_t pos = MLX90640_SENSOR_H * (MLX90640_SENSOR_W-1-y) +x;
            float val = image[pos];
            if (val>newMaxTemp) 
            {
                newMaxTemp = val;
            }
            else
            if (val<newMinTemp) 
            {
                newMinTemp = val;
            }
            colorPixel(& colorImage[pos],val,maxTemp,minTemp);

        }
    }
    maxTemp = newMaxTemp;
    minTemp = newMinTemp;
}

#define MLX90640TASKSIZE 8000

/* Structure that will hold the TCB of the task being created. */
StaticTask_t xMLX90640TaskBuffer;

/* Buffer that the task being created will use as its stack.  Note this is
an array of StackType_t variables.  The size of StackType_t is dependent on
the RTOS port. */
StackType_t xMLX90640TaskStack[ MLX90640TASKSIZE ];

void thermal_init()
{

    //MLX90640_SetDeviceMode(MLX_I2C_ADDR, 0);
    //MLX90640_SetSubPageRepeat(MLX_I2C_ADDR, 0);
    MLX90640_SetRefreshRate(MLX_I2C_ADDR, 1<<(FPS-1));
    MLX90640_SetChessMode(MLX_I2C_ADDR);

    uint16_t eeMLX90640[832];
    MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

    
	mlx90640_mutex = xSemaphoreCreateMutexStatic( &xMutexBuffer );
    configASSERT( mlx90640_mutex );

	MLX90640Thread = xTaskCreateStatic(thermalCameraTask,"MLX90640 Task",MLX90640TASKSIZE,NULL,tskIDLE_PRIORITY+4, xMLX90640TaskStack,&xMLX90640TaskBuffer,0);  /* Variable to hold the task's data structure. */

}



void thermalCameraTask(void *param)
{
    
    static uint16_t eeMLX90640[832];
    float emissivity = 1;
    uint16_t frame[834];
    float mlx90640To[768];
    float eTa;
    int32_t frame_time = (FRAME_TIME_MICROS);
    
    while(1){
       // uint64_t start = micros();
        MLX90640_GetFrameData(MLX_I2C_ADDR, frame);

        eTa = MLX90640_GetTa(frame, &mlx90640);
        MLX90640_CalculateTo(frame, &mlx90640, emissivity, eTa, mlx90640To);

        MLX90640_BadPixelsCorrection((&mlx90640)->brokenPixels, mlx90640To, 1, &mlx90640);
        MLX90640_BadPixelsCorrection((&mlx90640)->outlierPixels, mlx90640To, 1, &mlx90640);
       // printf("Time to i2c: %d\r\n",(uint32_t)(micros()-start));

        xSemaphoreTake(mlx90640_mutex,portMAX_DELAY);
        memcpy(image,mlx90640To,sizeof(image));
        xSemaphoreGive(mlx90640_mutex);
    }
}