/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package UserControlled;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayList;

import Actions.JennyFlagController;
import Actions.JennyO1BGlyphPicker;

import Actions.JewelJousterV2;
import Autonomous.ImageAlignmentHelper;
import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import Actions.NewArialDepositor;
import SensorHandlers.JennySensorTelemetry;
import Actions.JennyO1CRAD;


import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_COLUMN_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_PERCENT_COLUMN_CHECK;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.DEFAULT_SLEEP_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.HIGH_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SLOW_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.WEST;
import static SensorHandlers.JennySensorTelemetry.RAD_LIMIT;

/*
    An opmode for the User Controlled portion of the game
    This code was used for the first regional qualifier at Asheville NC
    Limited autonomous, no ciphers on red team, glyphs close but not always scored.
    We won 1st place
 */
@TeleOp(name="Jenny O1B User Controlled", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class JennyO1B extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    JoystickHandler leftJoystick, rightJoystick;
    NewArialDepositor glyphLift;
    JennyO1BGlyphPicker glyphPicker;
    JennyO1CRAD RAD;
    JewelJousterV2 jouster;
    JennySensorTelemetry sensorTelemetry;
    JennyFlagController flagController;
    ImageAlignmentHelper alignmentHelper;
    boolean autoLiftPositionMode = false;
    int paralaxedControl = 0;
    boolean flagOn = false;
    boolean ready = false;
    CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR color = CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE;
    NewArialDepositor.GLYPH_PLACEMENT_LEVEL[] liftPosition = new NewArialDepositor.GLYPH_PLACEMENT_LEVEL[]{NewArialDepositor.GLYPH_PLACEMENT_LEVEL.GROUND, NewArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW1_AND_2, NewArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW3_AND_4};

    VuforiaHelper vuforia;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    Bitmap curImage = null;
    ArrayList<Integer> coloredColumns;

    boolean isSlowMode = false;
    double driveVelocity = 0;
    double turnRps = 0;

    @Override
    public void runOpMode() {
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());

        }
        try{
            glyphLift = new NewArialDepositor(hardwareMap);

        }
        catch(Exception e){
            Log.e("Error!","Glyph Lift: " + e.toString());
            throw new RuntimeException("Glyph Lift Creation Error! " + e.toString());
        }
        try{
            RAD = new JennyO1CRAD(hardwareMap);
        } catch (Exception e){
            Log.e("Error!", "Jenny RAD: " + e.toString());
            throw new RuntimeException("RAD Creation Error! " + e.toString());
        }
        try{
            jouster = new JewelJousterV2("jewelJoust", "jewelJoustTurn", this, hardwareMap);
        } catch (Exception e){
            Log.e("Error!", "Jenny JewelJouster: " + e.toString());
            throw new RuntimeException("Jenny JewelJouster! " + e.toString());
        }
        try {
            glyphPicker = new JennyO1BGlyphPicker(hardwareMap);
        } catch (Exception e) {
            Log.e("Error!", "Glyph Picker: " + e.toString());
            throw new RuntimeException("Glyph Picker! " + e.toString());
        }
        sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
        flagController = new JennyFlagController(hardwareMap);
        leftJoystick = new JoystickHandler(gamepad1,JoystickHandler.LEFT_JOYSTICK);
        rightJoystick = new JoystickHandler(gamepad1,JoystickHandler.RIGHT_JOYSTICK);
        alignmentHelper = new ImageAlignmentHelper(DESIRED_WIDTH, navigation, this);
        vuforia = new VuforiaHelper();
        jouster.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.STORE);
        while (!opModeIsActive()) {
            if (gamepad1.start) {
                if(color == CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE) color = CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.RED;
                else color = CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE;
                while (gamepad1.start) ;
            }
            telemetry.addData("Color", color);
            telemetry.update();
        }
        cryptoBoxFinder = new CryptoBoxColumnImageProcessor(DESIRED_HEIGHT, DESIRED_WIDTH, CLOSE_UP_MIN_PERCENT_COLUMN_CHECK, CLOSE_UP_MIN_COLUMN_WIDTH, color);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        long loopStartTime = 0;



        while (opModeIsActive()) {
            loopStartTime = System.currentTimeMillis();
            jouster.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.STORE_WITHOUT_PAUSE);
            handleDriveControl();
            handlePickSystem();
            handleAerialLift();
            handleRAD();

            //GLYPH LIFT

            //GLYPH LIFT
            long glyphLiftStart = System.currentTimeMillis();

            /*
            else {
                if (gamepad1.right_trigger > 0.1 || (gamepad2.right_trigger > .1 && gamepad1.right_trigger < .1)) {
                    position++;
                    while (gamepad1.right_trigger > 0.1 || (gamepad2.right_trigger > .1 && gamepad1.right_trigger < .1)) ;
                }
                else if (gamepad1.right_bumper || (gamepad2.right_bumper && !gamepad1.right_bumper) && !glyphLift.isPressed()) {
                    position--;
                    while (gamepad1.right_bumper || (gamepad2.right_bumper && !gamepad1.right_bumper) && !glyphLift.isPressed()) ;
                }

                if (position <= 0) position = 0;
                if (position >= liftPosition.length - 1) position = liftPosition.length - 1;
                if(position == 0) {
                    while (opModeIsActive() && !glyphLift.isPressed() && !gamepad1.dpad_up && !gamepad1.a && !gamepad1.b) {
                        glyphLift.slowRetract();
                    }
                    glyphLift.stopLift();
                    glyphLift.setLiftRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                } else glyphLift.goToGlyphLevel(liftPosition[position]);
            }
            /*
            long glyphLiftStart = System.currentTimeMillis();
            if (!autoLiftPositionMode) {
                if (gamepad1.right_trigger > 0.1) {
                    if(!isSlowMode){
                        glyphLift.extend();
                    }
                    else{
                        glyphLift.slowExtend();
                    }
                }
                else if (gamepad1.right_bumper && !gamepad1.left_bumper && !glyphLift.isPressed()){
                    if(!isSlowMode){
                        glyphLift.retract();
                    }
                    else{
                        glyphLift.slowRetract();
                    }
                }
                else if (gamepad2.right_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper){
                    if(!isSlowMode){
                        glyphLift.extend();
                    }
                    else {
                        glyphLift.slowExtend();
                    }
                }

                else if (gamepad2.right_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper && !glyphLift.isPressed()){
                    if(!isSlowMode){
                        glyphLift.retract();
                    }
                    else{
                        glyphLift.slowRetract();
                    }
                }
                else if(!glyphLift.isPressed()) {
                    glyphLift.stopLift();
                }
                else
                    glyphLift.setLiftPower(0);
            } else {
                if (gamepad1.right_trigger > 0.1) {
                    position++;
                    while (gamepad1.right_trigger > 0.1) ;
                }
                if (gamepad1.right_bumper) {
                    position--;
                    while (gamepad1.right_bumper) ;
                }
                if (gamepad2.right_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper) {
                    position++;
                    while (gamepad2.right_trigger > 0.1) ;
                }

                if (gamepad2.right_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper) {
                    position--;
                    while (gamepad2.right_bumper) ;
                }

                if (position <= 0) position = 0;
                if (position >= liftPosition.length - 1) position = liftPosition.length - 1;
                if(position == 0) {
                    while (opModeIsActive() && !glyphLift.isPressed() && !gamepad1.dpad_up && !gamepad1.a && !gamepad1.b) {
                        glyphLift.slowRetract();
                    }
                    glyphLift.stopLift();
                    glyphLift.setLiftRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                } else glyphLift.goToGlyphLevel(liftPosition[position]);
            }
            */
            if(gamepad2.x && gamepad2.y) {
                autoLiftPositionMode = !autoLiftPositionMode;
                while (gamepad2.x && gamepad2.y);
            }
            telemetry.addData("Glyph Lift Time", "" + (System.currentTimeMillis() - glyphLiftStart));
            //GLYPH ROLLER



            //MISC
            if(gamepad1.left_trigger > 0.1){
                paralaxedControl++;
                if(paralaxedControl >= 3) paralaxedControl = 0;
                while (gamepad1.left_trigger > 0.1);
            }
            if(gamepad1.right_stick_button){
                navigation.turnToHeading(NORTH, this);
            }
            else if(gamepad1.left_stick_button){
                navigation.setOrientationOffset(360 - navigation.getOrientation());
            }

            if(gamepad1.x){
                curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                long startTimeImage = System.currentTimeMillis();
                while(!alignmentHelper.centerOnCryptoBoxClosestToCenter(0,coloredColumns,EAST,WEST) && opModeIsActive() && !(gamepad1.dpad_up && gamepad1.a && gamepad1.b)){
                    curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                    coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                    Log.d("Time Taken","" + (System.currentTimeMillis() - startTimeImage));
                    handleAerialLift();
                    startTimeImage = System.currentTimeMillis();

                }
            }
            else if(gamepad1.dpad_left){
                navigation.driveDistance(CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES/1.5, WEST, SLOW_SPEED_IN_PER_SEC, this);
                sleep(DEFAULT_SLEEP_DELAY_MILLIS);
                curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                while(!alignmentHelper.centerOnCryptoBoxClosestToCenter(0,coloredColumns,EAST,WEST) && opModeIsActive() && !(gamepad1.dpad_up && gamepad1.a && gamepad1.b)){
                    curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                    coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                    handleAerialLift();
                }
            }
            else if(gamepad1.dpad_right){
                navigation.driveDistance(CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES/1.5, EAST, SLOW_SPEED_IN_PER_SEC, this);
                sleep(DEFAULT_SLEEP_DELAY_MILLIS);
                curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                while(!alignmentHelper.centerOnCryptoBoxClosestToCenter(0,coloredColumns,EAST,WEST) && opModeIsActive() && !(gamepad1.dpad_up && gamepad1.a && gamepad1.b)){
                    curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                    coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                    handleAerialLift();
                }
            }
            if(gamepad2.left_stick_button){
                if(flagOn) flagController.pauseFlag();
                else flagController.startFlag();
                flagOn = !flagOn;
                while (gamepad2.left_stick_button);
            }

            //jouster.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.STORE);
            telemetry.addData("lift tick", glyphLift.getLiftMotorPosition());
            telemetry.addData("Belt power", glyphLift.getBeltPower());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Min Button","" + glyphLift.isPressed());
            telemetry.addData("Time for Loop","" + (System.currentTimeMillis() - loopStartTime));
            telemetry.update();
        }
        navigation.stopNavigation();
        glyphLift.kill();
        RAD.kill();
        sensorTelemetry.stopTelemetryLogging();
        glyphPicker.kill();
        flagController.stopFlag();
        jouster.kill();
        vuforia.kill();
    }

    public void handleDriveControl(){
        if(gamepad1.right_trigger > 0.1){
            isSlowMode = true;
        }
        else isSlowMode = false;
        driveVelocity = (isSlowMode)? (SLOW_SPEED_IN_PER_SEC):HIGH_SPEED_IN_PER_SEC;
        driveVelocity *= leftJoystick.magnitude();
        //Log.d("DriveVelocity","" + driveVelocity);
        turnRps = (isSlowMode)? (.25 * rightJoystick.magnitude() * rightJoystick.x()/Math.abs(rightJoystick.x())):.7 *rightJoystick.magnitude() * rightJoystick.x()/Math.abs(rightJoystick.x());
        navigation.driveOnHeadingWithTurning((paralaxedControl == 2)? (leftJoystick.angle() + 90)%360:(paralaxedControl == 0)? leftJoystick.angle(): (leftJoystick.angle() - 90)%360, driveVelocity, turnRps);

    }

    public void handleAerialLift(){
        if (!autoLiftPositionMode) {
            if (gamepad2.right_trigger > .1) {
                glyphLift.extend();
            }
            else if (gamepad2.right_bumper && !glyphLift.isPressed()){
                glyphLift.retract();
            }
            else
                glyphLift.stopLift();
        }

        if(gamepad1.right_bumper && gamepad1.left_bumper){
            glyphLift.stopBelt();
        }
        else {
            if (!glyphLift.isPressed()) {
                //if(gamepad1.left_bumper)
                //if (gamepad1.left_trigger > 0.1) glyphLift.startBeltSlow();
//                else if (gamepad1.left_bumper && !gamepad1.right_bumper)
//                    glyphLift.retractBeltSlow();
                if (gamepad2.left_trigger > 0.1)
                    glyphLift.startBeltSlow();
                else if (gamepad2.left_bumper )
                    glyphLift.retractBeltSlow();
                else
                    glyphLift.stopBelt();
            } else {
                //if (gamepad1.left_trigger > 0.1) glyphLift.startBelt();
                //else if (gamepad1.left_bumper && !gamepad1.right_bumper) glyphLift.retractBelt();
                if (gamepad2.left_trigger > 0.1)
                    glyphLift.startBelt();
                else if (gamepad2.left_bumper)
                    glyphLift.retractBelt();
                else
                    glyphLift.stopBelt();
            }
        }
    }

    public void handleRAD(){
        //RAD Extender
        if(gamepad2.dpad_right){
            RAD.extendRAD();
        }
        else if(gamepad2.dpad_left && !sensorTelemetry.isPressed(RAD_LIMIT)){
            RAD.retractRAD();
        }
        else {
            RAD.pauseRADExtender();
        }

        //RAD Grabber
        if(gamepad2.dpad_up){
            RAD.grabFrontRelic();
        }
        else if(gamepad2.dpad_down){
            RAD.releaseFrontRelic();
        }
        RAD.activateStopper();
    }

    public void handlePickSystem(){
        //GLYPH GRABBER
        if(gamepad1.a) glyphPicker.grab();
        else if(gamepad1.b) glyphPicker.spit();
        else if(!gamepad1.a && !gamepad1.b && gamepad2.a) glyphPicker.grab();
        else if(!gamepad1.a && !gamepad1.b && gamepad2.b) glyphPicker.spit();
        else glyphPicker.pause();
    }

}

