#include <PID_v1.h>
#include <Motor.h>

// Create Encoder Object from its pin(En1, En2 , DirectionalOffset)
// Create Motor Object (Pwm pin, Dir pin1, Dir pin2 (Optional))
#define pulseMode 0

class positionalnew
{
public:
    Motor *mtr = new Motor();
    Motor *mtr2 = new Motor();

    int mode = pulseMode;
    double Setpoint = 0, Input, Output;
    int targetPulse = 0;
    double aggKp = 0.03, aggKi = 0, aggKd = 0.00;
    double softKp = 0.01, softKi = 0, softKd = 0.00;

    int softThreshold = 0;
    long diff = 0;
    bool enable = true, twoMotor = false, temp = false, inc = false;
    int speed = 0, pwm = 0;
    PID *myPID = new PID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);
    int sampleTime = 0;

    positionalnew()
    {
        myPID->SetMode(AUTOMATIC);
        myPID->SetSampleTime(this->sampleTime);
    }
    positionalnew(Motor *mtr)
    {
        myPID->SetMode(AUTOMATIC);
        myPID->SetSampleTime(this->sampleTime);
        this->mtr = mtr;
    }
    positionalnew(Motor *mtr, Motor *mtr2)
    {
        myPID->SetMode(AUTOMATIC);
        myPID->SetSampleTime(this->sampleTime);
        this->mtr = mtr;
        this->mtr2 = mtr2;
        twoMotor = true;
    }
    void setThreshold(int targetThreshold)
    {
        softThreshold = targetThreshold;
    }
    void setPulse(int targetPulse)
    {
        inc = true;
        temp = true;
        this->targetPulse = targetPulse;
        mode = pulseMode;
        this->Setpoint = targetPulse;
    }
    void setOutputLimits(int min, int max)
    {
        myPID->SetOutputLimits(min, max);
    }
    void setAggTunings(double Kp, double Ki, double Kd)
    {
        this->aggKp = Kp;
        this->aggKi = Ki;
        this->aggKd = Kd;
        myPID->SetTunings(Kp, Ki, Kd);
    }
    void setSoftTunings(double Kp, double Ki, double Kd)
    {
        this->softKp = Kp;
        this->softKi = Ki;
        this->softKd = Kd;
        myPID->SetTunings(Kp, Ki, Kd);
    }
    void compute()
    {
        if (enable)
        {

            Input = mtr->getReadings();
            if (abs(Input - targetPulse) < softThreshold)
            {
                this->setSoftTunings(softKp, softKi, softKd);
            }
            else
            {
                this->setAggTunings(aggKp, aggKi, aggKd);
            }

            if (abs(Setpoint - Input) < 4 && abs(Output) < 4 || temp == false && abs(Setpoint - Input) < 4)
            {
                temp = false;
                mtr->setPWM(0);
            }
            else
            {
                myPID->Compute();
                if (inc == true)
                {
                    // Serial.println("1st");
                    mtr->setPWM(Output);
                    // mtr2->setPWM(100);
                    // analogWrite(22,Output);
                    if (twoMotor)
                    {
                        Serial.println("2nd");
                        if (Output > 0)
                        {
                            mtr2->setPWM(Output + 2);
                            // Serial.println("Forwards555");
                        }
                        else if (Output < 0)
                        {
                            mtr2->setPWM(Output - 2);
                            // Serial.println("Backwards");
                        }
                        else
                            mtr2->setPWM(0);
                    }
                }
            }
        }
    }
};