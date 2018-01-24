package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import MotorControllers.MotorController;

/**
 * Created by robotics on 11/15/17.
 */

/*
    A class to control our glyph picker with one motor
 */
public class JennyO1BGlyphPicker implements ActionHandler {
    MotorController wheelMotor;
    HardwareMap hardwareMap;
    private double WHEEL_POWERS = 0.4;

    public JennyO1BGlyphPicker(HardwareMap hw) throws Exception{
        hardwareMap = hw;
        wheelMotor = new MotorController("glyphMotor", "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
        wheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelMotor.setMotorDirection(DcMotorSimple.Direction.FORWARD);
    }

    public int grab(){
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
    public void kill() {
        wheelMotor.killMotorController();
    }
}
