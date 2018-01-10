package Actions;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Autonomous.REVColorDistanceSensorController;

/**
 * Created by robotics on 1/6/18.
 */

public class JewelJouster implements ActionHandler {
    private ServoHandler servo;
    private REVColorDistanceSensorController colorSensor;
    private HardwareMap hardwareMap;
    public enum EXTENDION_MODE {STORE, READ, HIT, NEUTRAL};
    private EXTENDION_MODE curMode = EXTENDION_MODE.STORE;
    private static final double STORE_POSITION = 106.0;
    private static final double NEUTRAL_POSITION = 85.0;
    private static final double READ_POSITION = 45.0;
    private static final double HIT_POSITION = 35.0;

    public JewelJouster(String servoName, HardwareMap h){
        servo = new ServoHandler(servoName,h);
        hardwareMap = h;
        colorSensor = new REVColorDistanceSensorController(REVColorDistanceSensorController.type.JEWEL_SNATCH_O_MATIC,"jewelSensor", hardwareMap);
        servo.setServoRanges(HIT_POSITION, STORE_POSITION);
        setPosition(EXTENDION_MODE.STORE);
    }

    public void setPosition(EXTENDION_MODE m){
        switch (m){
            case STORE:
                servo.setDegree(STORE_POSITION);
                break;
            case READ:
                servo.setDegree(READ_POSITION);
                break;
            case HIT:
                servo.setDegree(HIT_POSITION);
                break;
            case NEUTRAL:
                servo.setDegree(NEUTRAL_POSITION);
                break;
        }
    }

    public void changeDegree(double toChange){
       servo.incrementDegree(toChange);
    }

    public void setDegree(double degree){
        servo.setDegree(degree);
    }

    public REVColorDistanceSensorController.color getJewelColor(){
        return colorSensor.getColor();
    }

    public double getDistance() {
        return colorSensor.getDistance(DistanceUnit.CM);
    }

    @Override
    public boolean doAction(String action, long maxTimeAllowed) {
        return false;
    }

    @Override
    public boolean stopAction(String action) {
        return false;
    }

    @Override
    public boolean startDoingAction(String action) {
        return false;
    }

    @Override
    public void stop() {

    }
}
