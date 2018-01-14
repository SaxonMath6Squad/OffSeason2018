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
package Testers;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import Actions.JewelJouster;
import DriveEngine.JennyNavigation;
import Actions.JennyFlagController;
import Actions.JennyO1BRAD;

import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;

/*
    An opmode to test if all our drive wheels are working correctly
 */
@TeleOp(name="Servo Zero Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class ServoZeroTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    JennyO1BRAD rad;
    //JennySensorTelemetry sensorTelemetry;
    JewelJouster jouster;
    JennyFlagController flagController;

    @Override
    public void runOpMode() {
        int servo = 0;
        double position = 0;
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
            rad = new JennyO1BRAD(hardwareMap);
            jouster = new JewelJouster("jewelJouster",hardwareMap);
            flagController = new JennyFlagController(hardwareMap);
        }
        catch (Exception e){
            Log.e("Error!" , e.toString());
            throw new RuntimeException("System Creation Error! " + e.toString());

        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            if(gamepad1.dpad_up){
                position++;
                while(gamepad1.dpad_up);
            } else if(gamepad1.dpad_down){
                position--;
                while(gamepad1.dpad_down);
            }
            if(gamepad1.start){
                servo++;
                if(servo > 3) servo = 0;
                if(servo == 1) position = 90;
                while (gamepad1.start);
            }
            if(position >= 180) position = 180;
            if(position <= 0) position = 0;

            switch (servo){
                case 0:
                    telemetry.addData("Servo", "Jewel Joust");
                    jouster.setDegree(position);
                    break;
                case 1:
                    telemetry.addData("Servo", "RAD Grabber");
                    rad.setGrabberPosition(position);
                    break;
                case 2:
                    telemetry.addData("Servo", "Flag Spinner");
                    flagController.setFlagSpinnerPosition(position);
                    break;
                case 3:
                    telemetry.addData("Servo", "Flag Waver");
                    flagController.setFlagWaverPosition(position);
                    break;
            }
            telemetry.addData("Position", Double.toString(position));
            telemetry.update();
        }

        navigation.stopNavigation();
    }
}
