package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import MotorControllers.NewMotorController;

/**
 * Created by robotics on 11/15/17.
 */

public class WheelPickerSingleMotor implements ActionHandler {
    NewMotorController wheelMotor;
    HardwareMap hardwareMap;
    private double WHEEL_POWERS = 1;


    public WheelPickerSingleMotor(HardwareMap hw) throws Exception{
        hardwareMap = hw;
        wheelMotor = new NewMotorController("glyphMotor", "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
        wheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelMotor.setMotorDirection(DcMotorSimple.Direction.FORWARD);
    }

    public int pick(){
        wheelMotor.setMotorPower(WHEEL_POWERS);
        return 0;
    }

    public int pause(){
        wheelMotor.setMotorPower(0);
        return 0;
    }

    public int spit() {
        wheelMotor.setMotorPower(-WHEEL_POWERS);
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
        wheelMotor.killMotorController();
    }
}
