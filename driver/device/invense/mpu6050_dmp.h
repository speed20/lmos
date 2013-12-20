#ifndef _MPU6050_DMP_H_
#define _MPU6050_DMP_H_

#define min(a, b) ((a) < (b) ? (a) : (b))

/* DMP function */
uint8_t mpu6050_dmpInitialize();
bool mpu6050_dmpPacketAvailable();
uint8_t mpu6050_dmpGetAccel_32(int32_t *data, const uint8_t* packet);
uint8_t mpu6050_dmpGetAccel_16(int16_t *data, const uint8_t* packet);
uint8_t mpu6050_dmpGetAccel_vector(VectorInt16 *v, const uint8_t* packet);
uint8_t mpu6050_dmpGetQuaternion_32(int32_t *data, const uint8_t* packet);
uint8_t mpu6050_dmpGetQuaternion_16(int16_t *data, const uint8_t* packet);
uint8_t mpu6050_dmpGetQuaternion_tuple(Quaternion *q, const uint8_t* packet);
uint8_t mpu6050_dmpGetGyro_32(int32_t *data, const uint8_t* packet);
uint8_t mpu6050_dmpGetGyro_16(int16_t *data, const uint8_t* packet);
uint8_t mpu6050_dmpGetMag(int16_t *data, const uint8_t* packet);
uint8_t mpu6050_dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
uint8_t mpu6050_dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
uint8_t mpu6050_dmpGetGravity(VectorFloat *v, Quaternion *q);
uint8_t mpu6050_dmpGetEuler(float *data, Quaternion *q);
uint8_t mpu6050_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
uint8_t mpu6050_dmpProcessFIFOPacket(const unsigned char *dmpData);
uint8_t mpu6050_dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed);
uint16_t mpu6050_dmpGetFIFOPacketSize();
#endif
