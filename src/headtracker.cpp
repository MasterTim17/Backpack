#include "headtracker.h"
#include <Arduino.h>
#include <Wire.h>

#ifdef USE_QMC5883
#include <DFRobot_QMC5883.h>
#endif

#ifdef USE_MPU6050
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#endif

/////////// FUNCTION DEFS ///////////

extern void sendMSPViaEspnow(mspPacket_t *packet);

/////////////////////////////////////

#ifdef USE_QMC5883
DFRobot_QMC5883 compass(&Wire, /*I2C addr*/QMC5883_ADDRESS);
#endif

#ifdef USE_MPU6050
MPU6050 mpu;

// MPU control/status vars
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
bool firstBoot;         // used to detect orientation of headtracker
int orient = 0;
#endif

void
Headtracker::Init()
{
    delay(0);
    Wire.begin(PIN_SDA,PIN_SCL);

#ifdef USE_QMC5883
    while (!compass.begin())
    {
        Serial.println("Could not find a valid compass _MC5883 sensor!");
        delay(500);
    }
#endif

#ifdef USE_MPU6050
    mpu.initialize();
    while(mpu.dmpInitialize() != 0)
    {
        Serial.println("Could not initialize dmp of MPU6050!");
        delay(500);
    }

    // mpu.setXGyroOffset(mpu.getXGyroOffset());
    // mpu.setYGyroOffset(mpu.getYGyroOffset());
    // mpu.setZGyroOffset(mpu.getZGyroOffset());
    // mpu.setZAccelOffset(mpu.getZAccelOffset());

    // mpu.CalibrateAccel();
    // mpu.CalibrateGyro();

    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    firstBoot = true;
#endif

}

void
Headtracker::Loop(uint32_t now)
{
    if((now-m_last)>70 && !firstBoot){ // it seems like a buffer is overloading when <60

        mspPacket_t packet;
        packet.reset();
        packet.makeCommand();
        packet.function = MSP_ELRS_BACKPACK_HEADTRACKER;

        int yaw=0;
        static int dynamicYaw=0;
        int pitch=0;

#ifdef USE_QMC5883
        float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
        compass.setDeclinationAngle(declinationAngle);
        sVector_t mag = compass.readRaw();
        compass.getHeadingDegrees();
        yaw = mag.HeadingDegress*10 + dynamicYaw;

        if(yaw>2700) yaw=yaw-3600;
        else if(yaw>1800) yaw=-450;
        else if(yaw>450) yaw=450;
#endif

#ifdef USE_MPU6050
        yaw = ypr[0]*1800/PI + dynamicYaw;
        pitch = ypr[1]*1800/PI;

        //accelerometer
        // Serial.print("ypr\t");
        // Serial.print(ypr[0] * 180/PI);
        // Serial.print("\t");
        // Serial.print(ypr[1] * 180/PI);
        // Serial.print("\t");
        // Serial.println(ypr[2] * 180/PI);

        // handle gimbal lock for yaw
        if(yaw>1800){ 
            dynamicYaw -= 3600;
            yaw-=3600;
        }else if(yaw<-1800){
            dynamicYaw += 3600;
            yaw+=3600;
        }

        if(pitch>900) pitch=1800-pitch;
        else if(pitch<-900) pitch=-1800-pitch;
#endif

        if(yaw>450){
            dynamicYaw-=yaw-450;
            yaw=450;
        }else if(yaw<-450) {
            dynamicYaw+=-yaw-450;
            yaw=-450;
        }

        if(pitch>450) pitch=450;
        else if(pitch<-450) pitch=-450;

        packet.addByte(yaw>>8);
        packet.addByte(yaw);
        packet.addByte(pitch>>8);
        packet.addByte(pitch);
        sendMSPViaEspnow(&packet);

        m_last = now;
    }

#ifdef USE_MPU6050
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    uint16_t fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if ((mpuIntStatus & 0x02) > 0) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        // accelerometer
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);

        if(firstBoot && now>5000){ // wait until mpu gets stable output
            if(abs(gravity.y)>0.7){
                orient = 1;
                Serial.println("Orientation y");
            }else if(abs(gravity.x)>0.7){
                orient = 2;
                Serial.println("Orientation x");
            }
            firstBoot = false;
        }else{
            // adjust ypr inputs to the new orientation
            // WARNING: yaw axis is bugged when in another orientation
            if(orient == 1){
                float og = gravity.y;
                gravity.y = gravity.z;
                gravity.z = og;
                float oq = q.y;
                q.y = q.z;
                q.z = oq;
            } else if(orient == 2){
                float og = gravity.x;
                gravity.x = gravity.z;
                gravity.z = og;
                float oq = q.x;
                q.x = q.z;
                q.z = oq;
            }
        }

        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
#endif

}

Headtracker::Headtracker(Stream *port)
{
    m_port = port;
}