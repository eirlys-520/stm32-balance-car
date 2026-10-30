#include "pt_mpu6050.h"

extern I2C_HandleTypeDef hi2c2;

/*******************************************************************
    功能：MPU6050写寄存器
    参数：
                reg:  寄存器地址
                data: 要写入的数据
    返回值：
                成功返回0
                失败返回-1
*******************************************************************/
static unsigned char MPU6050_Write_Reg(unsigned char reg, unsigned char data)
{
    unsigned char buff[2] = {reg, data};

    if (HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, buff, 2, 1000) != HAL_OK)
    {
        return -1;
    }

    return 0;
}
/*******************************************************************
    功能：MPU6050写寄存器
    参数：
                reg:  寄存器地址
                data: 读取数据的指针
  返回值：
                成功返回0
                失败返回负值
*******************************************************************/
static unsigned char MPU6050_Read_Reg(unsigned char reg, unsigned char* data)
{
    if (HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, &reg, 1, 1000) != HAL_OK)
    {
        return -1;
    }

    if (HAL_I2C_Master_Receive(&hi2c2, MPU6050_ADDR_READ, data, 1, 1000) != HAL_OK)
    {
        return -2;
    }

    return 0;
}
/*******************************************************************
    功能：MPU6050连续读取多个字节
    参数：
                reg:  起始寄存器地址
                buf:  存放数据缓冲区
              len:  读取数据长度
  返回值：
                成功返回0
                失败返回负值
*******************************************************************/
static unsigned char MPU6050_Read_Bytes(unsigned char reg, unsigned char* buf, unsigned char len)
{
    if (HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, &reg, 1, 1000) != HAL_OK)
    {
        return -1;
    }

    if (HAL_I2C_Master_Receive(&hi2c2, MPU6050_ADDR_READ, buf, len, 1000) != HAL_OK)
    {
        return -2;
    }

    return 0;
}
/*******************************************************************
    功能：MPU6050初始化
    参数：
            缺省
  返回值：
                成功返回0
                失败返回-1
*******************************************************************/
int MPU6050_Init(void)
{
    unsigned char check = 0;

    // 读取器件ID
    MPU6050_Read_Reg(MPU6050_WHO_AM_I, &check);
    if (check != 0x68)
    {
        return -1;
    }

    // 复位设备
    MPU6050_Write_Reg(MPU6050_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    // 唤醒设备，使用内部8MHz时钟
    MPU6050_Write_Reg(MPU6050_PWR_MGMT_1, 0x00);

    // 设置陀螺仪量程 ±250°/s
    MPU6050_Write_Reg(MPU6050_GYRO_CONFIG, 0x00);

    // 设置加速度计量程 ±2g
    MPU6050_Write_Reg(MPU6050_ACCEL_CONFIG, 0x00);

    // 设置数字低通滤波器带宽 5Hz
    MPU6050_Write_Reg(MPU6050_CONFIG, 0x06);

    // 设置采样率 1kHz/(1+4)=200Hz
    MPU6050_Write_Reg(MPU6050_SMPLRT_DIV, 0x04);

    return 0;
}

/*******************************************************************
    功能：读取MPU6050数据
    参数：
            data:  mpu6050数据结构体指针
  返回值：
                成功返回0
                失败返回-1
*******************************************************************/
void MPU6050_Read_ALL(MPU6050_Data* data)
{
    unsigned char buffer[14];

    if (MPU6050_Read_Bytes(MPU6050_ACCEL_XOUT_H, buffer, 14) == 0)
    {
        // 加速度计数据
        data->accel_x_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
        data->accel_y_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
        data->accel_z_raw = (int16_t)((buffer[4] << 8) | buffer[5]);

        // 陀螺仪数据
        data->gyro_x_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
        data->gyro_y_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
        data->gyro_z_raw = (int16_t)((buffer[12] << 8) | buffer[13]);

        // 转换为物理量
        // 加速度 (g -> m/s2)
        data->accel_x = data->accel_x_raw / MPU6050_ACCEL_FS_SEL * GRAVITY;
        data->accel_y = data->accel_y_raw / MPU6050_ACCEL_FS_SEL * GRAVITY;
        data->accel_z = data->accel_z_raw / MPU6050_ACCEL_FS_SEL * GRAVITY;

        // 陀螺仪 (°/s)
        data->gyro_x = data->gyro_x_raw / MPU6050_GYRO_FS_SEL;
        data->gyro_y = data->gyro_y_raw / MPU6050_GYRO_FS_SEL;
        data->gyro_z = data->gyro_z_raw / MPU6050_GYRO_FS_SEL;
    }
}

// mpu6050 写
int mpu6050_write(unsigned char slaveaddr, unsigned char regaddr, int datalen, unsigned char* pdata)
{

    HAL_I2C_Mem_Write(&hi2c2, slaveaddr << 1, regaddr, 1, pdata, datalen, 1000);

    return 0;
}
// mpu6050 读
int mpu6050_read(unsigned char slaveaddr, unsigned char regaddr, int datalen, unsigned char* pdata)
{

    HAL_I2C_Mem_Read(&hi2c2, (slaveaddr << 1) | 0x01, regaddr, 1, pdata, datalen, 1000);

    return 0;
}

// DMP解算 四元数转换欧拉角
int DMP_Pitch(void)
{
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];  // 四元数：w, x, y, z

    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);  // 从DMP FIFO读取数据

    // 四元数转欧拉角（俯仰角Pitch、偏航角Yaw、横滚角Roll）
    gGlobalData.PitchAngle = atan2(2 * (quat[1] * quat[2] + quat[0] * quat[3]), quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3]) * 180 / M_PI;
	//gGlobalData.PitchAngle = -asin(2 * (quat[1] * quat[3] - quat[0] * quat[2])) * 180.0f / M_PI;
    return 0;
}