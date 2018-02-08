package Actions.HardwareWrappers;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by robotics on 1/5/18.
 */

/*
    A class to control the robots
 */
public class JennyFlagControllerOneArm extends Thread{
    ServoHandler flagWaver;
    public static final int[] FLAG_WAVER_MIN_MAX_DEG = {20,160};
    public final long PERIOD = 1000;
    volatile boolean shouldRun = true;
    volatile boolean flagShouldMove = false;
    HardwareMap hardwareMap;

    public JennyFlagControllerOneArm(HardwareMap hw){
        hardwareMap = hw;
        flagWaver = new ServoHandler("flagWaver", hardwareMap);

        new Thread(new Runnable() {
            @Override
            public void run() {
                while(shouldRun){
                    if(flagShouldMove) moveFlag();
                }
            }
        }).start();
    }

    private void moveFlag(){
        double waver = FLAG_WAVER_MIN_MAX_DEG[0] + (FLAG_WAVER_MIN_MAX_DEG[1] - FLAG_WAVER_MIN_MAX_DEG[0])*(.5+Math.cos(System.currentTimeMillis()/1000.0) * (1000.0/PERIOD));
        Log.d("Waver deg","" + waver);
        flagWaver.setDegree(waver);
    }



    public void setFlagWaverPosition(double positionInDeg){
        flagWaver.setDegree(positionInDeg);
    }

    public void startFlag(){
        flagShouldMove = true;
    }

    public void pauseFlag() {
        flagShouldMove = false;
    }

    public void stopFlag(){
        flagShouldMove = false;
        shouldRun = false;
    }
}
