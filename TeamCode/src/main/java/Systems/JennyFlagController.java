package Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Actions.ServoHandler;

/**
 * Created by robotics on 1/5/18.
 */

public class JennyFlagController extends Thread{
    ServoHandler[] flagHolder = new ServoHandler[2];
    public static final int FLAG_SPINNER = 0;
    public static final int FLAG_WAVER = 1;
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
        double spinner = Math.sin(System.currentTimeMillis());
        double waver = Math.cos(System.currentTimeMillis());
        flagHolder[FLAG_SPINNER].setPosition(spinner);
        flagHolder[FLAG_WAVER].setPosition(waver);
    }

    public void setFlagSpinnerPosition(double positionInDeg){
        flagHolder[FLAG_SPINNER].setDegree(positionInDeg);
    }

    public void setFlagWaverPosition(double positionInDeg){
        flagHolder[FLAG_WAVER].setDegree(positionInDeg);
    }

    public void setFlagShouldMove(boolean shouldMove){
        flagShouldMove = shouldMove;
    }

    public void stopFlagController(){
        flagShouldMove = false;
        shouldRun = false;
    }
}
