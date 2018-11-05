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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by aliva on 10/4/18.
 */

@Autonomous(name="Crater Park Only", group="Team 5214")
//@Disabled
public class CraterParkOnly extends LinearOpMode {

    // declare and initialize motors
    protected ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor elevator;
    private Servo handle;
    private Servo flipper;
    private Servo eyebrow1;
    private Servo eyebrow2;
    @Override

    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // hardware mapping
        leftFront  = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack  = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        elevator = hardwareMap.get(DcMotor.class, "LIFT");
        handle = hardwareMap.get(Servo.class, "HANDLE");
        flipper = hardwareMap.get(Servo.class, "flipper");
        eyebrow1 = hardwareMap.get(Servo.class, "EYEBROW1");
        eyebrow2 = hardwareMap.get(Servo.class, "EYEBROW2");



        // reverse right motors
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // wait for start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            eyebrow1.setPosition(0.97);

            eyebrow2.setPosition(0.02);

            flipper.setPosition(0.27);

            sleep(500);

            straightWithEncoder(0.9,52);

            killDriveTrainPower();

            sleep(30000);

            flipper.setPosition(.27);

            sleep(100);

            eyebrow1.setPosition(0.97);

            eyebrow2.setPosition(0.03);
        }
    }

    // sleep function
    public void sleep(int i) {
        long initial_time = System.currentTimeMillis();
        while (System.currentTimeMillis() - initial_time < i) {
        }
    }

    // drive forward function
    public void driveStraight(double power, int time) {
        // set power on all motors
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        sleep(time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    // drive reverse function
    public void driveReverse(double power, int time) {

        // set reverse power on all motors
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);

        // sleep + kill motors
        sleep(time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    // turn right function
    public void turnRight(double power, int time) {

        // set power on all motors (forward on right, backward on left)
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(power);

        // sleep + kill motors
        sleep(time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    // turn left function
    public void turnLeft(double power, int time) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power on all motors (forward on right, backward on left)
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(-power);

        // sleep + kill motors
        sleep(time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    // encoder conversion function
    public void  motorWithEncoder(DcMotor motorName, double power, int inches) {

        //converts inches to ticks
        int ticks = (int) (inches * 288 / (6 * 3.14159));

        //modifies move to position based on starting ticks position, keeps running tally
        int position2move2 = motorName.getCurrentPosition() + ticks;
        motorName.setTargetPosition(position2move2);
        motorName.setPower(power);
    }

    // run to position function
    public void runToPosition() {
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // kill drive train function
    public void killDriveTrainPower() {
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    // reset encoder function
    public void resetEncoders() {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // drive forward encoder function
    public void straightWithEncoder(double strength, int straightInches) {

        runToPosition();

        motorWithEncoder(leftBack, strength, -straightInches);
        motorWithEncoder(leftFront, strength, -straightInches);
        motorWithEncoder(rightBack, strength, -straightInches);
        motorWithEncoder(rightFront, strength, -straightInches);

        while (leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy() && rightFront.isBusy()) {
        }

        killDriveTrainPower();

        //reset encoder values
        resetEncoders();

        //put the motors back into a mode where they can run
        runToPosition();
    }
}
