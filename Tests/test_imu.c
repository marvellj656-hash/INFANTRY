#include "imu/bmi088.h"
#include "bsp_imu.h"
#include "spi.h"
#include "task.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
GPIO_TypeDef port_a,port_b;
SPI_HandleTypeDef hspi1={SPI1};
static uint8_t registers[2][128];
static int selected=-1;
static bool fail,corrupt;
static unsigned waited;
unsigned HAL_SPI_GetState(SPI_HandleTypeDef *p){(void)p;return HAL_SPI_STATE_READY;}
void vTaskDelay(TickType_t ticks){waited+=ticks;}
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state){
    int chip=port==GPIOA?0:1;assert(pin==(chip==0?16:1));
    if(state==GPIO_PIN_RESET){assert(selected==-1);selected=chip;}
    else {assert(selected==chip);selected=-1;}
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,uint8_t *tx,uint8_t *rx,uint16_t n,uint32_t timeout){
    assert(spi==&hspi1 && selected>=0 && timeout==2);
    if(fail)return HAL_ERROR;
    memset(rx,0xcc,n);
    unsigned reg=tx[0]&0x7f;
    if(tx[0]&0x80){unsigned skip=selected==0?2:1;for(unsigned i=skip;i<n;i++)rx[i]=registers[selected][reg+i-skip] | ((selected==1 && reg+i-skip==0x10)?0x80:0);}
    else {assert(n==2);registers[selected][reg]=corrupt?(uint8_t)(tx[1]^1):tx[1];}
    return HAL_OK;
}
int main(void){
    Bmi088State imu={0};registers[0][0]=0x1e;registers[1][0]=0x0f;
    uint8_t byte=0;assert(bsp_imu_read(BMI088_ACCEL,0,&byte,1)&&byte==0x1e);
    assert(bsp_imu_read(BMI088_GYRO,0,&byte,1)&&byte==0x0f);
    fail=true;byte=42;assert(!bsp_imu_read(BMI088_ACCEL,0,&byte,1)&&byte==42&&selected==-1);fail=false;
    assert(bmi088_init(&imu)&&imu.initialized&&!imu.valid&&waited>=100);
    assert(registers[0][0x41]==1&&registers[0][0x40]==0xab&&registers[1][0x10]==1);
    assert(bsp_imu_read(BMI088_GYRO,0x10,&byte,1) && byte==0x81);
    registers[0][0x16]=0x55;registers[0][0x17]=0x15; /* +5461 LSB ~=1g */
    registers[1][2]=0;registers[1][3]=0xc0; /* -16384 ~= -1000deg/s */
    registers[0][0x22]=0x3e;registers[0][0x23]=0;
    assert(bmi088_read(&imu)&&imu.valid&&imu.samples==1);
    assert(fabsf(imu.accel[2]-9.80665f)<0.005f);
    assert(fabsf(imu.gyro[0]+17.45329252f)<0.001f);
    assert(imu.temperature_c==85);
    registers[0][0x22]=0xc1;assert(bmi088_read(&imu)&&imu.temperature_c==-40);
    fail=true;assert(!bmi088_read(&imu)&&!imu.valid&&!imu.initialized&&imu.errors==1);fail=false;
    registers[1][0]=0xff;assert(!bmi088_init(&imu));registers[1][0]=0x0f;
    corrupt=true;assert(!bmi088_init(&imu));corrupt=false;
    assert(debug_bmi088.init_failures>=2);
    assert(bmi088_init(&imu));
    puts("PASS: SPI ACC dummy/GYRO framing, CS release, ID/config readback, SI units, temperature, recovery");
}
