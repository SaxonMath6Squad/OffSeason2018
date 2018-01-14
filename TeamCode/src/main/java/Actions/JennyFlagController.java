package Actions;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Actions.ServoHandler;

/**
 * Created by robotics on 1/5/18.
 */

/*
    A class to control the robots
 */
public class JennyFlagController extends Thread{
    ServoHandler[] flagHolder = new ServoHandler[2];
    public static final int FLAG_SPINNER = 0;
    public static final int FLAG_WAVER = 1;
    public static final int[] FLAG_WAVER_MIN_MAX_DEG = {0,120};
    public static final int[] FLAG_SPINNER_MIN_MAX_DEG = {0,120};
    volatile boolean shouldRun = true;
    volatile boolean flagShouldMove = false;
    HardwareMap hardwareMap;

    public JennyFlagController(HardwareMap hw){
        hardwareMap = hw;
        flagHolder[FLAG_SPINNER] = new ServoHandler("flagSpinner", hardwareMap);
        flagHolder[FLAG_WAVER] = new ServoHandler("flagWaver", hardwareMap);

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
        double spinner = FLAG_SPINNER_MIN_MAX_DEG[0] + (FLAG_SPINNER_MIN_MAX_DEG[1] - FLAG_SPINNER_MIN_MAX_DEG[0])*(.5 + Math.sin(3*(double)System.currentTimeMillis()/1000)/2.0);
        double waver = FLAG_WAVER_MIN_MAX_DEG[0] + (FLAG_WAVER_MIN_MAX_DEG[1] - FLAG_WAVER_MIN_MAX_DEG[0])*(.5+Math.cos(4*(double)System.currentTimeMillis()/1000)/2.0);
        Log.d("Spinner deg","" + spinner);
        Log.d("Waver deg","" + waver);
        flagHolder[FLAG_SPINNER].setDegree(spinner);
        flagHolder[FLAG_WAVER].setDegree(waver);
    }

    public void setFlagSpinnerPosition(double positionInDeg){
        flagHolder[FLAG_SPINNER].setDegree(positionInDeg);
    }

    public void setFlagWaverPosition(double positionInDeg){
        flagHolder[FLAG_WAVER].setDegree(positionInDeg);
    }

    public void startFlag(){
        flagShouldMove = true;
    }

    public void stopFlag(){
        flagShouldMove = false;
        shouldRun = false;
    }
}
