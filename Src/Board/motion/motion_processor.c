#include "motion_processor.h"

#include <string.h>
#include "MahonyAHRS.h"
#include "quaternion.h"


#define G 9.8f

#define MadgwickAHR 0
#define MahonyAHR 1
#define HaanAHR 2

#define ALGO_SELECT MahonyAHR

#if ALGO_SELECT == MadgwickAHR
#define AHRS_Reset MadgwickAHRSreset
#define AHRS_Update MadgwickAHRSupdate
#define AHRS_GetQ   MadgwickAHRSgetQ
#elif ALGO_SELECT == MahonyAHR
#define AHRS_Reset MahonyAHRSreset
#define AHRS_Update MahonyAHRSupdate
#define AHRS_GetQ   MahonyAHRSgetQ
#else
#define AHRS_Reset HaanAHRS_Reset
#define AHRS_Update HaanAHRS_Update
#define AHRS_GetQ HaanAHRS_GetQ
#endif

float m_lastTimeStamp;
motion_data_t m_motionData;
motion_data_t m_motionDataPrev;
int motion_processor_initialized = 0;

/* Parameters */
motion_processor_config_t motion_processor_config;

void motion_processor_init(motion_processor_config_t * config)
{
    m_lastTimeStamp = 0;
    motion_processor_initialized = 0;
    motion_processor_config = *config;
    memset(&m_motionData, 0, sizeof(m_motionData));
}

void motion_processor_reset()
{
    motion_processor_init(&motion_processor_config);
}

void motion_processor_initBaseline()
{
    m_motionData.gyro.x = 0;
    m_motionData.gyro.y = 0;
    m_motionData.gyro.z = 0;
    m_motionData.accel.x = 0;
    m_motionData.accel.y = 0;
    m_motionData.accel.z = 0;
    m_motionData.compass.x = 0;
    m_motionData.compass.y = 0;
    m_motionData.compass.z = 0;
    m_motionData.euler.roll = 0;
    m_motionData.euler.pitch = 0;
    m_motionData.euler.yaw = 0;
    m_motionData.velocity.x = 0;
    m_motionData.velocity.y = 0;
    m_motionData.velocity.z = 0;
    m_motionData.coordinate.x = 0;
    m_motionData.coordinate.y = 0;
    m_motionData.coordinate.z = 0;

    m_motionDataPrev.gyro.x = 0;
    m_motionDataPrev.gyro.y = 0;
    m_motionDataPrev.gyro.z = 0;
    m_motionDataPrev.accel.x = 0;
    m_motionDataPrev.accel.y = 0;
    m_motionDataPrev.accel.z = 0;
    m_motionDataPrev.compass.x = 0;
    m_motionDataPrev.compass.y = 0;
    m_motionDataPrev.compass.z = 0;
    m_motionDataPrev.euler.roll = 0;
    m_motionDataPrev.euler.pitch = 0;
    m_motionDataPrev.euler.yaw = 0;
    m_motionDataPrev.velocity.x = 0;
    m_motionDataPrev.velocity.y = 0;
    m_motionDataPrev.velocity.z = 0;
    m_motionDataPrev.coordinate.x = 0;
    m_motionDataPrev.coordinate.y = 0;
    m_motionDataPrev.coordinate.z = 0;
}

void motion_processor_process(motion_raw_t * data)
{
    float dt = 0;
    if(!motion_processor_initialized)
    {
        AHRS_Reset();
        motion_processor_initialized = 1;
        m_motionDataPrev.timestamp = data->timestamp;
        return;
    }
    else
    {
        dt = data->timestamp - m_motionDataPrev.timestamp;

        if(motion_processor_config.DOF69 == 0)
        {
            AHRS_Update(data->gyro.x*PI/180.f, data->gyro.y*PI/180.f,data->gyro.z*PI/180.f,
                        data->accel.x,data->accel.y,data->accel.z,
                        data->compass.x,data->compass.y,data->compass.z, dt);//0,0,0,dt);
        }
        else
        {
            AHRS_Update(data->gyro.x*PI/180.f, data->gyro.y*PI/180.f,data->gyro.z*PI/180.f,
                        data->accel.x*G,data->accel.y*G,data->accel.z*G,
                        0,0,0, dt);//0,0,0,dt);
        }
    }

    float q[4];
    float q_conj[4];
    float q_prod[4];
    float q_temp[4];
    float acc_raw[4] = {0, data->accel.x, data->accel.y, data->accel.z};


    AHRS_GetQ(q);
    Quaternion_Conjugate(q_conj, q);
    Quaternion_Multiply(q_temp, q, acc_raw);
    /* q_prod is the result */
    Quaternion_Multiply(q_prod, q_temp, q_conj);

    m_motionData.normAcc.x = q_prod[1];
    m_motionData.normAcc.y = q_prod[2];
    m_motionData.normAcc.z = q_prod[3];

    if(motion_processor_config.integrateRotation)
    {
        m_motionData.euler.roll += dt*m_motionDataPrev.gyro.x;
        m_motionData.euler.pitch += dt*m_motionDataPrev.gyro.y;
        m_motionData.euler.yaw += dt*m_motionDataPrev.gyro.z;
    }
    else
    {
        float angle[3];
        Quaternion_getEulerAngles(q, angle);
        m_motionData.euler.roll = angle[0];
        m_motionData.euler.pitch = angle[1];
        m_motionData.euler.yaw = angle[2];
    }
    m_motionData.quat.w = q[0];
    m_motionData.quat.x = q[1];
    m_motionData.quat.y = q[2];
    m_motionData.quat.z = q[3];

    m_motionData.velocity.x += dt*m_motionDataPrev.normAcc.x*G;
    m_motionData.velocity.y += dt*m_motionDataPrev.normAcc.y*G;
    m_motionData.velocity.z += dt*(m_motionDataPrev.normAcc.z-1.f)*G;

    m_motionData.coordinate.x += dt*m_motionData.velocity.x;
    m_motionData.coordinate.y += dt*m_motionData.velocity.y;
    m_motionData.coordinate.z += dt*m_motionData.velocity.z;

    m_motionData.accel = data->accel;
    m_motionData.gyro = data->gyro;
    m_motionData.compass = data->compass;
    m_motionData.timestamp = data->timestamp;
    m_motionDataPrev = m_motionData;

}

motion_data_t* motion_processor_get_data(void)
{
    return &m_motionData;
}
