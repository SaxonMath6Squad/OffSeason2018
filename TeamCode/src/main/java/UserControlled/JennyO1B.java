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

import java.sql.SQLSyntaxErrorException;
import java.util.ArrayList;

import Actions.ArialDepositor;
import Actions.ArialDepositorTest;
import Actions.HardwareWrappers.NewArialDepositor;
import Actions.HardwareWrappers.NewSpoolMotor;
import Actions.JennyFlagController;
import Actions.JennyO1BGlyphPicker;
import Actions.JewelJouster;
import Actions.JewelJousterV2;
import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import MotorControllers.PIDController;
import SensorHandlers.JennySensorTelemetry;
import Actions.JennyO1BRAD;
import Testers.MotorStopperTest;

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
    JennyO1BRAD RAD;
    JewelJousterV2 jouster;
    JennySensorTelemetry sensorTelemetry;
    JennyFlagController flagController;
    boolean autoLiftPositionMode = false;
    boolean paralaxedControl = false;
    boolean flagOn = false;
    boolean ready = false;
    CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR color = CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE;
    NewArialDepositor.GLYPH_PLACEMENT_LEVEL[] liftPosition = new NewArialDepositor.GLYPH_PLACEMENT_LEVEL[]{NewArialDepositor.GLYPH_PLACEMENT_LEVEL.GROUND, NewArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW1_AND_2, NewArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW3_AND_4};

    int position = 0;
    PIDController cameraPIDController;
    VuforiaHelper vuforia;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;

    int CRYPTO_COLUMN_TARGET_POSITION = 70;
    Bitmap curImage = null;
    ArrayList<Integer> coloredColumns;

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
            RAD = new JennyO1BRAD(hardwareMap);
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
        cameraPIDController = new PIDController(5.0/DESIRED_WIDTH,0,0);
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
        boolean isSlowMode = false;
        double driveVelocity = 0;
        double turnRps = 0;
        // run until the end of the match (driver presses STOP)
        long loopStartTime = 0;



        while (opModeIsActive()) {
            loopStartTime = System.currentTimeMillis();


            driveVelocity = (isSlowMode)? (SLOW_SPEED_IN_PER_SEC):HIGH_SPEED_IN_PER_SEC;
            driveVelocity *= leftJoystick.magnitude();
            //Log.d("DriveVelocity","" + driveVelocity);
            turnRps = (isSlowMode)? (.05 * rightJoystick.magnitude() * rightJoystick.x()/Math.abs(rightJoystick.x())):.25 *rightJoystick.magnitude() * rightJoystick.x()/Math.abs(rightJoystick.x());
            navigation.driveOnHeadingWithTurning((paralaxedControl)? (leftJoystick.angle() + 90)%360:leftJoystick.angle(), driveVelocity, turnRps);
            if(gamepad1.left_bumper && gamepad1.right_bumper){
                isSlowMode = !isSlowMode;
                while(gamepad1.left_bumper && gamepad1.right_bumper);
            }

            //GLYPH GRABBER
            if(gamepad1.a) glyphPicker.grab();
            else if(gamepad1.b) glyphPicker.spit();
            else if(!gamepad1.a && !gamepad1.b && gamepad2.a) glyphPicker.grab();
            else if(!gamepad1.a && !gamepad1.b && gamepad2.b) glyphPicker.spit();
            else glyphPicker.pause();

            //GLYPH LIFT
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
            if(gamepad2.x && gamepad2.y) {
                autoLiftPositionMode = !autoLiftPositionMode;
                while (gamepad2.x && gamepad2.y);
            }
            telemetry.addData("Glyph Lift Time", "" + (System.currentTimeMillis() - glyphLiftStart));
            //GLYPH ROLLER


            if(!glyphLift.isPressed()) {

                //if(gamepad1.left_bumper)

                if (gamepad1.left_trigger > 0.1) glyphLift.startBeltSlow();
                else if (gamepad1.left_bumper && !gamepad1.right_bumper) glyphLift.retractBeltSlow();
                else if (gamepad2.left_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.startBeltSlow();
                else if (gamepad2.left_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.retractBeltSlow();
                else
                    glyphLift.stopBelt();
            } else {
                if (gamepad1.left_trigger > 0.1) glyphLift.startBelt();
                else if (gamepad1.left_bumper && !gamepad1.right_bumper) glyphLift.retractBelt();
                else if (gamepad2.left_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.startBelt();
                else if (gamepad2.left_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.retractBelt();
                else
                    glyphLift.stopBelt();
            }

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
                RAD.grabRelic();
            }
            else if(gamepad2.dpad_down){
                RAD.releaseRelic();
            }
            else {
                RAD.stopRelic();
            }
            //MISC
            if(gamepad1.back){
                paralaxedControl = !paralaxedControl;
                while (gamepad1.start);
            }
            if(gamepad1.right_stick_button){
                navigation.turnToHeading(NORTH, this);
            }
            if(gamepad1.left_stick_button){
                navigation.setOrientationOffset(360 - navigation.getOrientation());
            }

            if(gamepad1.x){
                curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                while(!centerOnCryptoBoxClosestToCenter(0,coloredColumns,EAST,WEST) && opModeIsActive() && !gamepad1.dpad_up && !gamepad1.a && !gamepad1.b){
                    curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                    coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                }
            }
            if(gamepad1.dpad_left){
                navigation.driveDistance(CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES/1.5, WEST, SLOW_SPEED_IN_PER_SEC, this);
                sleep(DEFAULT_SLEEP_DELAY_MILLIS);
                curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                while(!centerOnCryptoBoxClosestToCenter(0,coloredColumns,WEST,EAST) && opModeIsActive() && !gamepad1.dpad_up && !gamepad1.a && !gamepad1.b){
                    curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                    coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                }
            }
            else if(gamepad1.dpad_right){
                navigation.driveDistance(CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES/1.5, EAST, SLOW_SPEED_IN_PER_SEC, this);
                sleep(DEFAULT_SLEEP_DELAY_MILLIS);
                curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                while(!centerOnCryptoBoxClosestToCenter(0,coloredColumns,EAST,WEST) && opModeIsActive() && !gamepad1.dpad_up && !gamepad1.a && !gamepad1.b){
                    curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                    coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                }
            }
            if(gamepad2.left_stick_button){
                if(flagOn) flagController.pauseFlag();
                else flagController.startFlag();
                flagOn = !flagOn;
                while (gamepad2.start);
            }

            //jouster.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.STORE);
            telemetry.addData("lift tick", glyphLift.getLiftMotorPosition());
            telemetry.addData("Belt power", glyphLift.getBeltPower());
            telemetry.addData("Status", "Run Time: " + runtime.toString());



            telemetry.addData("Time for Loop","" + (System.currentTimeMillis() - loopStartTime));
            telemetry.update();
        }
        navigation.stopNavigation();
        glyphLift.kill();
        RAD.stop();
        sensorTelemetry.stopTelemetryLogging();
        glyphPicker.kill();
        flagController.stopFlag();
        jouster.kill();
    }

    public int findCryptoColumnClosestToDesired(ArrayList<Integer> columns, int desiredLocationOnScreen){
        if(columns.size() == 0){
            return -1;
        }
        int minDelta = Math.abs(desiredLocationOnScreen - columns.get(0));
        int minDeltaIndex = 0;
        for(int i = 1; i < columns.size(); i ++){
            if(Math.abs(desiredLocationOnScreen - columns.get(i)) < minDelta){
                minDelta = Math.abs(desiredLocationOnScreen - columns.get(i));
                minDeltaIndex = i;
            }
        }
        Log.d("Target Column","" + minDeltaIndex);
        return minDeltaIndex;
    }

    public boolean centerOnCryptoBoxClosestToCenter(int column, ArrayList<Integer> columns, int primaryDirection, int secondaryDirection){
        Log.d("Seen columns", Integer.toString(columns.size()));
        if(columns.size() == 0){
            navigation.correctedDriveOnHeadingIMU(primaryDirection, SLOW_SPEED_IN_PER_SEC, 0, this);
            Log.d("Seek Status","Not found, moving in primary dir");
            return false;
        }
        int targetColumnIndex = findCryptoColumnClosestToDesired(columns, CRYPTO_COLUMN_TARGET_POSITION);
        if(targetColumnIndex != -1) {
            if (Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION) > DESIRED_WIDTH/20.0) {
                Log.d("Delta Pixels","" + Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION));
                //keep driving the hint
                Log.d("Column location", columns.get(column).toString());
                cameraPIDController.setSp(0);
                double distToColumn = columns.get(column) - CRYPTO_COLUMN_TARGET_POSITION;
                double velocityCorrection = -cameraPIDController.calculatePID(distToColumn);
                Log.d("Seek Status","Found, moving at " + velocityCorrection);
                navigation.correctedDriveOnHeadingIMU((velocityCorrection < 0) ? primaryDirection : secondaryDirection, Math.abs(velocityCorrection), this);
            } else {
                navigation.brake();
                Log.d("Seek Status", "Obtained, delta Pixels:" + Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION));
                return true;
            }
        }
        return false;
    }
}
