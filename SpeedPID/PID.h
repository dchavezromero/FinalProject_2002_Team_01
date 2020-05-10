#ifndef _PID_H
#define _PID_H

class PID {
private:
    float irConsts[3] = {0, 0, 0};
    float gyroConsts[3] = {0, 0, 0};
    gloat wallConsts[3] = {0, 0, 0};

public:
    void setIRPID(float P, float I, float D);
    void setGyroPID(float P, float I, float D);
    void setWallPID(float P, float I, float D);

    void calcIRPID();
    void calcGyroPID();
    void calcWallPID();

    void getIREfforts(float &left, float &right);
    void getGyroEfforts(float &left, float &right);
    void getWallEfforts(float &left, float &right);
}

#endif
