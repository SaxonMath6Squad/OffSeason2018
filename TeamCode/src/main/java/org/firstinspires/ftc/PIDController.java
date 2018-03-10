package org.firstinspires.ftc;

/**
 * Created by jothm on 3/9/2018.
 */

public class PIDController {
    double P = 0;
    double I = 0;
    double D = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double setPoint = 0;
    double error = 0;
    double prevError = 0;
    double tempI = 0;
    double iMax = 0;
    long timeAtLastCalculation = 0;

    public PIDController(double kp, double ki, double kd){
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
    }

    public void setSp(double sp){
        setPoint = sp;
    }
    public void setKp(double gain){
        Kp = gain;
    }
    public void setKi(double gain){
        Ki = gain;
    }
    public void setKd(double gain){
        Kd = gain;
    }
    public void setIMax(double max){
        iMax = max;
    }

    public double calculatePID(double pointValue){
        double deltaTime = (System.currentTimeMillis() - timeAtLastCalculation)/1000.0;
        error = setPoint - pointValue;
        P = Kp * error;
        tempI = Ki * error * deltaTime;
        I += tempI;
        if(iMax != 0){
            if(I > 0 && I > iMax) I = iMax;
            else if(I < 0 && I < -iMax) I = -iMax;
        }
        D = Kd * (prevError - error) / deltaTime;
        prevError = error;
        timeAtLastCalculation = System.currentTimeMillis();
        return P + I + D;
    }
}
