package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;

import Actions.ActionHandler;
import MotorControllers.MotorController;

/**
 * Created by robotics on 1/22/18.
 */

public class NewSpoolMotor extends MotorController implements ActionHandler{
    private double extendSpeedInPerSecond = 0;
    private double retractSpeedInPerSecond = 0;

    public NewSpoolMotor(DcMotor m, String configFileLoc, double extendInPerSecond, double retractInPerSecond, HardwareMap hw) throws IOException {
        super(m,configFileLoc,hw);
        extendSpeedInPerSecond = extendInPerSecond;
        retractSpeedInPerSecond = retractInPerSecond;
    }

    public NewSpoolMotor(String motorName, String configFileLoc, double extendInPerSecond, double retractInPerSecond, HardwareMap hw) throws IOException {
        super(motorName,configFileLoc,hw);
        extendSpeedInPerSecond = extendInPerSecond;
        retractSpeedInPerSecond = retractInPerSecond;
    }

    public NewSpoolMotor(String motorName, String configFileLoc, String debugTag, double extendInPerSecond, double retractInPerSecond, HardwareMap hw) throws IOException {
        super(motorName,configFileLoc,debugTag,hw);
        extendSpeedInPerSecond = extendInPerSecond;
        retractSpeedInPerSecond = retractInPerSecond;
    }

    public void extend(){
        if(getMotorRunMode() != DcMotor.RunMode.RUN_USING_ENCODER) setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setInchesPerSecondVelocity(extendSpeedInPerSecond);
    }

    public void retract(){
        if(getMotorRunMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setInchesPerSecondVelocity(-retractSpeedInPerSecond);
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
}
