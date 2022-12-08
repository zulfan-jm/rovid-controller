//imu variable
bool imu_calibration_mode = false;
//int imu_int_pin = 4;
bool dmpReady = false;      //IMU Init Variables
volatile bool imuInterruptReady = false;
// uint8_t imuInterruptStatus;
// uint8_t imuDevStatus;
// uint16_t imuPacketSize;
// uint16_t imuFifoCount;
// uint8_t imuFifoBuffer [64];
int last_millis[3]; //Number of tasks
float imu_offsets[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //Acc X, Y, Z, Gyro X, Y, Z
float acc_scale = 0.00059855;   // measured in m/s^2 is 9.80665/16384 
float gyro_scale = 0.000133231; // measured in rad/s is 0,017453293/131
float acc_stddev = 0.033; // Dari sini https://robotics.stackexchange.com/questions/8860/angle-random-walk-vs-rate-noise-density-mpu6050
float gyro_stddev = 0.033;
float orientation_stddev = 0.0;