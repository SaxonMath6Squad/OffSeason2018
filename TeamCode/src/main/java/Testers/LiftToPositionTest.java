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

import Actions.ArialDepositor;

/*
    An opmode to test if all our drive wheels are working correctly
 */
@TeleOp(name="Glyph Auto Level Placement Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class LiftToPositionTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    ArialDepositor glyphSystem;
    double position = 0;
    @Override
    public void runOpMode() {
        try {
            glyphSystem = new ArialDepositor(hardwareMap);
        }
        catch (Exception e){
            Log.e("Error!" , e.toString());
            throw new RuntimeException("System Creation Error! " + e.toString());

        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a) {
                glyphSystem.goToGlyphLevel(ArialDepositor.GLYPH_PLACEMENT_LEVEL.GROUND);
                while (gamepad1.a);
            }
            else if(gamepad1.b) {
                glyphSystem.goToGlyphLevel(ArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW1);
                while (gamepad1.b);
            }
            else if(gamepad1.x) {
                glyphSystem.goToGlyphLevel(ArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW2);
                while (gamepad1.x);
            }
            else if(gamepad1.y) {
                glyphSystem.goToGlyphLevel(ArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW3);
                while (gamepad1.y);
            }



            telemetry.addData("Position ", Double.toString(position));
            telemetry.update();
        }
        glyphSystem.stop();
    }
}
