package Autonomous.OpModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;
import Systems.JennyO1BPickAndExtend;

import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.WEST;

/**
 * Created by robotics on 12/15/17.
 */

/*
    An opmode to test centering the robot on a cryptobox with the camera
 */
@Autonomous(name = "Matthew Autonomous", group = "visual autonomous")
//@Disabled
public class MatthewAutonomous extends LinearOpMode{

    JennyNavigation nav;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            nav = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
        } catch (Exception e){
            throw new RuntimeException(e);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();



        nav.stopNavigation();
    }
}
