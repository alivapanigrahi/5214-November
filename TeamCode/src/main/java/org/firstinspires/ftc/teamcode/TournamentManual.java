/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by aliva on 9/28/18.
 */

@TeleOp(name="5214 Manual", group="Team 5214")
//@Disabled
public class TournamentManual extends LinearOpMode {

    // declare and initialize motors
    private ElapsedTime runtime = new ElapsedTime();
    protected static DcMotor leftFront;
    protected static DcMotor rightFront;
    protected static DcMotor leftBack;
    protected static DcMotor rightBack;
    protected static DcMotor arm;
    protected static Servo eyebrow1; //LEFT WHEN LOOKING AT BACK
    protected static Servo eyebrow2; //RIGHT WHEN LOOKING AT BACK
    protected static Servo flipper;
    protected static Servo handle;
    protected static DcMotor lift;
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // hardware mapping
        leftFront  = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack  = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        arm = hardwareMap.get(DcMotor.class, "ARM");
        eyebrow1 = hardwareMap.get(Servo.class, "EYEBROW1");
        eyebrow2 = hardwareMap.get(Servo.class, "EYEBROW2");
        flipper = hardwareMap.get(Servo.class, "flipper");
        handle = hardwareMap.get(Servo.class, "HANDLE");
        lift = hardwareMap.get(DcMotor.class, "LIFT");
        // reverse right motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // wait for start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double leftPower2;
            double rightPower2;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // uses basic math to combine motions and is easier to drive straight.

            double drive = gamepad1.left_stick_y;
            double turn  =  -gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -0.9, 0.9) ;
            rightPower   = Range.clip(drive - turn, -0.9, 0.9) ;

            // set maximum drive power
            while (gamepad2.a) {
                leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
                rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            }

            // Send calculated power to wheels
            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);

            // sets position to open handle servo continuously
            if(gamepad1.y) {
                handle.setPosition(.05);
            }

            // 0.5 is the zero position for handle servo
            // sets position to stop handle servo
            if (gamepad1.b){
                handle.setPosition(0.5);
            }

            if (gamepad1.x) {
                handle.setPosition(0.7);
            }

            // sets power for lift to move up
            if (gamepad1.left_bumper){
                lift.setPower(1);

            }

            // sets power for lift to move down
            else if (gamepad1.right_bumper){
                lift.setPower(-1);
            }

            // sets power zero to kill lift
            else{
                lift.setPower(0);
            }

            // sets power to arm to move up
            if (gamepad2.dpad_up){
                arm.setPower(.3);
            }

            // sets power to arm to move down
            else if(gamepad2.dpad_down){
                arm.setPower(-.25);
            }

            // sets power zero to kill arm
            else {
                arm.setPower(0);
            }

            // sets position to lift eyebrow 2
            if (gamepad2.right_bumper){
                eyebrow2.setPosition(0.7);
            }

            // sets position to lower eyebrow 2
            if (gamepad2.right_trigger > 0.1){
                eyebrow2.setPosition(0.02);
            }

            // sets position to lift eyebrow1
            if (gamepad2.left_bumper){
                eyebrow1.setPosition(0.1);
            }

            // sets position to lower eyebrow 1
            if (gamepad2.left_trigger > 0.01){
                eyebrow1.setPosition(0.98);
            }

            // sets flipper position to move up
            if (gamepad2.dpad_right){
                flipper.setPosition(0.27);
            }

            // sets flipper position to move down
            if (gamepad2.dpad_left){
                flipper.setPosition(0.9);
            }

            telemetry.update();
        }
    }
}