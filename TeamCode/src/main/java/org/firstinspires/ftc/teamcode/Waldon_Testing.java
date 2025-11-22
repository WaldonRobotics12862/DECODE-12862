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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.config.Config;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

//setup the RoadRunner libraries
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import java.util.List;


@TeleOp(name="Waldon Testing")
@Config
//@Disabled
public class Waldon_Testing extends LinearOpMode {

    public static int A_speed = 4000;
    public static int X_speed = 3000;

    public static double P = 10.0; // Proportional
    public static double I = 3.0; // Integral
    public static double D = 0.0; // Derivitave
    public static double F = 0.00215; // Konstant

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx flywheel = null;
    private DcMotorEx spinEncoder = null;
    private CRServo spindexer = null;
    private DigitalChannel mag = null;

    private DigitalChannel eye1 = null;
    private DigitalChannel eye2 = null;

    private static final int NUM_PIXELS = 16;

    @Override
    public void runOpMode() {

        DigActions.Hopper hopper = new DigActions.Hopper(hardwareMap);
        DigActions.Launcher launcher = new DigActions.Launcher(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        spindexer = hardwareMap.get(CRServo.class, "spin");
        mag = hardwareMap.get(DigitalChannel.class,"mag");

        eye1 = hardwareMap.get(DigitalChannel.class, "eye1");
        eye2 = hardwareMap.get(DigitalChannel.class, "eye3");


        spinEncoder = hardwareMap.get(DcMotorEx.class,"spin_encoder");
        spinEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spinEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        eye1.setMode(DigitalChannel.Mode.OUTPUT);
        eye2.setMode(DigitalChannel.Mode.OUTPUT);

        eye1.setState(true);
        eye2.setState(true);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Actions.runBlocking((new SequentialAction(DigActions.Hopper.spinToSensor(),new SleepAction(2))));


            // Setup a variable for each drive wheel to save power level for telemetry
            double flywheelSpeed;

            if(gamepad2.a){
                eye1.setState(!eye1.getState());
            }
            if(gamepad2.b){
                eye2.setState(!eye2.getState());
            }

            if(gamepad1.a) {
                Actions.runBlocking(new SequentialAction(DigActions.Launcher.motorOn(A_speed)));
            }
            if(gamepad1.b) {
                //Actions.runBlocking(new SequentialAction(DigActions.Launcher.motorOff()));
                flywheel.setPower(0);
            }
            if(gamepad1.x) {
                //Actions.runBlocking(new SequentialAction(DigActions.Launcher.motorOn(X_speed)));
            }
            if(gamepad1.y) {
                //Actions.runBlocking(new SequentialAction(DigActions.Launcher.pullTrigger()));
                flywheel.setVelocityPIDFCoefficients(P, I, D, F);
                flywheel.setVelocity(3500*28/60);
            }
            if(gamepad1.right_bumper){
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flywheel.setPower(1);
            }
            if(gamepad1.left_bumper){
                flywheel.setPower(0);
                flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            PIDFCoefficients coefficients = new PIDFCoefficients(flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

            telemetry.addData("Magnetic Sensor",mag.getState());
            telemetry.addData("Encoder Count", spinEncoder.getCurrentPosition());
            //telemetry.addData("PIDF",coefficients);
            //telemetry.addData("RPM", flywheel.getVelocity()*60/28);
            telemetry.update();
        }
    }
}

