/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode Mason", group="Iterative Opmode")
//@Disabled
public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intakeDrive = null;
    private DcMotor linearDrive = null;
/*    private Servo hookDrive = null;*/
    private double sensitivity = 0.9;
    private boolean isItUP = false;
    private final static double hook_home = 0.0;
    private double hookPosition;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right motor");
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        linearDrive = hardwareMap.get(DcMotor.class, "linearDrive");
       /* hookDrive = hardwareMap.get (Servo.class,"hookDrive");*/


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
     /*   hookDrive.setPosition(hook_home);*/

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double intakePower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        /*double sensitivity = 0.9;*/

        boolean aButtonPushed = gamepad1.a;
        if (aButtonPushed) {
            sensitivity = sensitivity + 0.01;
        }

        boolean bButtonPushed = gamepad1.b;
        if (bButtonPushed) {
            sensitivity = sensitivity - 0.01;
        }
//We are making our code make sure that when we add and decrease sensitivity does not go over 1 and less than 0
        sensitivity = Math.max(sensitivity,0);
        sensitivity = Math.min(sensitivity,1);

        double drive = sensitivity*gamepad1.left_stick_y;
        double turn  =  -sensitivity*gamepad1.left_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Make X button spin robot
        boolean xButtonPushed = gamepad1.x;
        if (xButtonPushed) {
            // rightDrive.setPower(0.5);
            // leftDrive.setPower(-0.5);
            rightPower = 0.5;
            leftPower = -0.5;
        }

        boolean yButtonPushed = gamepad1.y;
        if (yButtonPushed) {
            // rightDrive.setPower(0.5);
            // leftDrive.setPower(-0.5);
            rightPower = -0.5;
            leftPower = 0.5;
        }
        // This code makes the left stick on gamepad 2 activate the intake

        float sensitivityIntake = 0.5f;

        //This code activates the output's bullet mode/fast mode
        boolean left_stick_button = gamepad2.left_stick_button;
        if (left_stick_button) {
            sensitivityIntake = 1;
        }
        //This code moves the intake
        float leftStickUp = gamepad2.left_stick_y;
        leftStickUp = leftStickUp * sensitivityIntake;
        intakePower = leftStickUp;
        //intakeDrive.setPower(leftStickUp);

        //This code moves the linear slide down.
        double linearPower = 0;
        float left_trigger = gamepad2.left_trigger;
        float sensitivityLinear = 0.75f;
        left_trigger = left_trigger * -1 * sensitivityLinear;
        linearPower = left_trigger;

        //This code moves the linear slide up.     HEMLO 2
        float right_trigger = gamepad2.right_trigger;
        right_trigger = right_trigger * sensitivityLinear;
        if (right_trigger > 0) {
            linearPower = right_trigger;
        }
        //          HEMLO 4
        boolean right_bumper = gamepad2.right_bumper;
        if (right_bumper)  {
            hookPosition = 0.0;
        }
        boolean left_bumper = gamepad2.left_bumper;
        if (left_bumper) {
            hookPosition = 0.25;
        }

        /*hookDrive.setPosition(hookPosition);*/
        linearDrive.setPower(linearPower);
        intakeDrive.setPower(intakePower);
        rightDrive.setPower(rightPower);
        leftDrive.setPower(leftPower);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f), intakePower (%.2f), LinearPower (%.2f)", leftPower, rightPower, intakePower, linearPower);
        telemetry.addData("sensitivity", sensitivity);
        telemetry.addData("hookPosition", hookPosition);
        int linearPosition = linearDrive.getCurrentPosition();
        telemetry.addData("LinearPosition", linearPosition);
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
