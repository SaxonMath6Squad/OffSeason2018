package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import MotorControllers.NewMotorController;

/**
 * Created by robotics on 11/15/17.
 */

public class WheelPickerDoubleMotor implements ActionHandler {
    NewMotorController wheelMotors[] = new NewMotorController[2];
    private final int LEFT_MOTOR = 0,RIGHT_MOTOR = 1;
    HardwareMap hardwareMap;
    private double WHEEL_POWERS = 1;


    public WheelPickerDoubleMotor(HardwareMap hw) throws Exception{
        hardwareMap = hw;
        wheelMotors[LEFT_MOTOR] = new NewMotorController("leftGlyphMotor", "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
        wheelMotors[RIGHT_MOTOR] = new NewMotorController("rightGlyphMotor", "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
        wheelMotors[LEFT_MOTOR].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelMotors[RIGHT_MOTOR].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelMotors[LEFT_MOTOR].setMotorDirection(DcMotorSimple.Direction.FORWARD);
        wheelMotors[RIGHT_MOTOR].setMotorDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int pick(){
        wheelMotors[LEFT_MOTOR].setMotorPower(WHEEL_POWERS);
        wheelMotors[RIGHT_MOTOR].setMotorPower(WHEEL_POWERS);
        return 0;
    }

    public int pause(){
        wheelMotors[LEFT_MOTOR].setMotorPower(0);
        wheelMotors[RIGHT_MOTOR].setMotorPower(0);
        return 0;
    }

    public int spit() {
        wheelMotors[LEFT_MOTOR].setMotorPower(-WHEEL_POWERS);
        wheelMotors[RIGHT_MOTOR].setMotorPower(-WHEEL_POWERS);
        return 0;
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
        for(int i =0; i < wheelMotors.length; i ++){
            wheelMotors[i].killMotorController();
        }
    }
}
