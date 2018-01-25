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

import Actions.JewelJousterV2;
import Autonomous.REVColorDistanceSensorController;

/*
    An opmode to test if all our drive wheels are working correctly
 */
@TeleOp(name="Zero Joust Tester", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class ZeroJouster extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JewelJousterV2 jewelJoust;
    int degree = 0;
    int turnDegree = 0;

    @Override
    public void runOpMode() {
        try {
            jewelJoust = new JewelJousterV2("jewelJoust", "jewelJoustTurn", hardwareMap);
        }
        catch (Exception e){
            Log.e("Error!" , "Glyph Lift: " + e.toString());
            throw new RuntimeException("Glyph Lift Creation Error! " + e.toString());

        }
        jewelJoust.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.STORE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            if (gamepad1.a){
                degree++;
                while (gamepad1.a);
            }
            else if(gamepad1.b){
                degree--;
                while (gamepad1.b);
            }
            else if(gamepad1.x){
                turnDegree++;
                while (gamepad1.x);
            }
            else if(gamepad1.y){
                turnDegree--;
                while (gamepad1.y);
            }
            if(degree >= 180) degree = 180;
            else if(degree <= 0) degree = 0;
            if(turnDegree >= 180) degree = 180;
            else if(turnDegree <= 0) turnDegree = 0;
            jewelJoust.setArmDegree(degree);
            jewelJoust.setTurnDegree(turnDegree);
            telemetry.addData("Arm degree", degree);
            telemetry.addData("Turn degree", turnDegree);
            telemetry.update();
        }

    }
}
