package Actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Actions.HardwareWrappers.ServoHandler;
import Autonomous.REVColorDistanceSensorController;

/**
 * Created by robotics on 1/6/18.
 */

public class JewelJousterV2 implements ActionHandler {
    private ServoHandler armServo, turnServo;
    private REVColorDistanceSensorController colorSensor;
    private HardwareMap hardwareMap;

    public enum JEWEL_JOUSTER_POSITIONS {STORE,READ,HIT_LEFT,HIT_RIGHT};

    public enum EXTENSION_MODE {STORE, READ, HIT_LEFT, HIT_RIGHT, NEUTRAL};
    public enum TURN_TO_HIT_MODE{STORE,READ,LEFT,RIGHT};
    private EXTENSION_MODE curArmMode = EXTENSION_MODE.STORE;

    private static final double TURN_STORE_POSITION = 95.0;
    private static final double TURN_READ_POSITION = 74.0;
    private static final double TURN_LEFT_POSITION = 54.0;
    private static final double TURN_RIGHT_POSITION = 93.0;


    private static final double ARM_STORE_POSITION = 77.0;
    private static final double ARM_NEUTRAL_POSITION = 77.0;
    private static final double ARM_READ_POSITION = 167.0;
    private static final double ARM_HIT_LEFT_POSITION = 167.0;
    private static final double ARM_HIT_RIGHT_POSITION = 153.0;
    private LinearOpMode mode;

    public JewelJousterV2(String armServo, String turningServo, LinearOpMode m, HardwareMap h){
        this.armServo = new ServoHandler(armServo,h);
        this.turnServo = new ServoHandler(turningServo,h);
        hardwareMap = h;
        colorSensor = new REVColorDistanceSensorController(REVColorDistanceSensorController.type.JEWEL_SNATCH_O_MATIC,"jewelSensor", hardwareMap);
        this.armServo.setServoRanges(ARM_STORE_POSITION-1, ARM_HIT_LEFT_POSITION+1);
        this.turnServo.setServoRanges(TURN_LEFT_POSITION-1, TURN_STORE_POSITION+1);
        mode = m;
        setJoustMode(JEWEL_JOUSTER_POSITIONS.STORE);

    }

    public void setJoustMode(JEWEL_JOUSTER_POSITIONS pos){
        switch(pos){
            case STORE:
                setArmPosition(EXTENSION_MODE.STORE);
                mode.sleep(200);
                setTurnPosition(TURN_TO_HIT_MODE.STORE);
                break;
            case READ:
                setTurnPosition(TURN_TO_HIT_MODE.READ);
                mode.sleep(200);
                setArmPosition(EXTENSION_MODE.READ);
                mode.sleep(200);
                break;
            case HIT_LEFT:
                setArmPosition(EXTENSION_MODE.HIT_LEFT);
                setTurnPosition(TURN_TO_HIT_MODE.LEFT);
                break;
            case HIT_RIGHT:
                setArmPosition(EXTENSION_MODE.HIT_RIGHT);
                setTurnPosition(TURN_TO_HIT_MODE.RIGHT);
                break;
        }
    }

    public void setArmPosition(EXTENSION_MODE m){
        switch (m){
            case STORE:
                armServo.setDegree(ARM_STORE_POSITION);
                break;
            case READ:
                armServo.setDegree(ARM_READ_POSITION);
                break;
            case HIT_LEFT:
                armServo.setDegree(ARM_HIT_LEFT_POSITION);
                break;
            case HIT_RIGHT:
                armServo.setDegree(ARM_HIT_RIGHT_POSITION);
                break;
            case NEUTRAL:
                armServo.setDegree(ARM_NEUTRAL_POSITION);
                break;
        }
    }

    public void setTurnPosition(TURN_TO_HIT_MODE mode){
        switch (mode){
            case LEFT:
                turnServo.setDegree(TURN_LEFT_POSITION);
                break;
            case READ:
                turnServo.setDegree(TURN_READ_POSITION);
                break;
            case RIGHT:
                turnServo.setDegree(TURN_RIGHT_POSITION);
                break;
            case STORE:
                turnServo.setDegree(TURN_STORE_POSITION);
                break;
        }
    }

    public void changeArmDegree(double toChange){
        armServo.incrementDegree(toChange);
    }

    public void setArmDegree(double degree){
        armServo.setDegree(degree);
    }

    public void changeTurnDegree(double toChange){
        turnServo.incrementDegree(toChange);
    }

    public void setTurnDegree(double degree){
        turnServo.setDegree(degree);
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
    public void kill() {

    }
}
