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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="A1", group="Linear Opmode")

public class A3 extends LinearOpMode {

    private DcMotor motoref = null;
    private DcMotor motoret = null;
    private DcMotor motordf = null;
    private DcMotor motordt = null;
    private DcMotor garra = null;
    private Servo coletor = null;
    TouchSensor sensor_toque;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motoref = hardwareMap.get(DcMotor.class, "ef");
        motoret = hardwareMap.get(DcMotor.class, "et");
        motordf = hardwareMap.get(DcMotor.class, "df");
        motordt = hardwareMap.get(DcMotor.class, "dt");
        garra = hardwareMap.get(DcMotor.class, "garra");
        coletor = hardwareMap.get(Servo.class, "servo_garra");
        sensor_toque = hardwareMap.get(TouchSensor.class, "sensor_toque");

        motoref.setDirection(DcMotor.Direction.REVERSE);
        motoret.setDirection(DcMotor.Direction.REVERSE);
        motordf.setDirection(DcMotor.Direction.FORWARD);
        motordt.setDirection(DcMotor.Direction.FORWARD);

        garra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        garra.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {

            coletor.setPosition(0);
            sleep(200);

            motordf.setPower(-0.7);
            motordt.setPower(-0.7);
            motoref.setPower(-0.7);
            motoret.setPower(-0.7);
            sleep(500);
            motordf.setPower(-0.7);
            motordt.setPower(-0.7);
            motoref.setPower(0.7);
            motoret.setPower(0.7);
            sleep(220);
            motordf.setPower(0);
            motordt.setPower(0);
            motoref.setPower(0);
            motoret.setPower(0);
            sleep(1000);
            while (garra.getCurrentPosition() < 1750){
                garra.setTargetPosition(1750);
                garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                garra.setPower(0.8);
            }
            motordf.setPower(0);
            motordt.setPower(0);
            motoref.setPower(0);
            motoret.setPower(0);
            sleep(1000);
            motordf.setPower(-0.7);
            motordt.setPower(-0.7);
            motoref.setPower(-0.7);
            motoret.setPower(-0.7);
            sleep(200);
            motordf.setPower(0);
            motordt.setPower(0);
            motoref.setPower(0);
            motoret.setPower(0);
            sleep(1000);
            motordf.setPower(-0.4);
            motordt.setPower(-0.4);
            motoref.setPower(0.4);
            motoret.setPower(0.4);
            sleep(50);
            coletor.setPosition(1);
            sleep(1000);
            motordf.setPower(0);
            motordt.setPower(0);
            motoref.setPower(0);
            motoret.setPower(0);
            sleep(2000);
            motordf.setPower(-0.7);
            motordt.setPower(-0.7);
            motoref.setPower(0.7);
            motoret.setPower(0.7);
            sleep(700);
            motordf.setPower(-0.7);
            motordt.setPower(-0.7);
            motoref.setPower(-0.7);
            motoret.setPower(-0.7);
            sleep(1000);

        }
    }
}
