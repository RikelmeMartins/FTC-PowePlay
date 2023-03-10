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

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name="AutonomoAzul", group="Linear Opmode")
public class AutonomoAzul extends LinearOpMode {

    private Runtime time = null;
    private DcMotor motoref = null;
    private DcMotor motoret = null;
    private DcMotor motordf = null;
    private DcMotor motordt = null;
    private DcMotor garra = null;
    private Servo coletor = null;
    TouchSensor sensor_toque;
    RevColorSensorV3 sensor_cor;

    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     DRIVE_GEAR_REDUCTION    = 30.24;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 90.0 * 3.14;

    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;


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
        sensor_cor = hardwareMap.get(RevColorSensorV3.class, "sensor_cor");

        motoref.setDirection(DcMotor.Direction.REVERSE);
        motoret.setDirection(DcMotor.Direction.REVERSE);
        motordf.setDirection(DcMotor.Direction.FORWARD);
        motordt.setDirection(DcMotor.Direction.FORWARD);

        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        garra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        garra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        garra.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        coletor.setPosition(0);
        waitForStart();
        if (opModeIsActive()) {


            while (garra.getCurrentPosition() < 1200) {
                garra.setTargetPosition(1200);
                garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                garra.setPower(0.8);
            }

            int esquerdaTarget = (int) (-580 * COUNTS_PER_MM);
            int direitaTarget = (int) (-580 * COUNTS_PER_MM);
            double LTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;
            double RTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;

            motoref.setTargetPosition(esquerdaTarget);
            motoret.setTargetPosition(esquerdaTarget);
            motordf.setTargetPosition(direitaTarget);
            motordt.setTargetPosition(direitaTarget);

            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motordf.setPower(LTPS * 0.1);
            motordt.setPower(RTPS * 0.1);
            motoref.setPower(LTPS * 0.1);
            motoret.setPower(RTPS * 0.1);

            while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                telemetry.addData("motordf:", motordf.getCurrentPosition());
                telemetry.addData("motordt:", motordt.getCurrentPosition());
                telemetry.addData("motoref:", motoref.getCurrentPosition());
                telemetry.addData("motoret:", motoret.getCurrentPosition());
                telemetry.addData("Green", sensor_cor.green());
                telemetry.addData("Blue", sensor_cor.blue());
                telemetry.addData("Red", sensor_cor.red());
                telemetry.update();
            }

            motordf.setPower(0);
            motordt.setPower(0);
            motoref.setPower(0);
            motoret.setPower(0);
            sleep(1000);

            if (sensor_cor.green() > sensor_cor.blue() && sensor_cor.green() > sensor_cor.red()){
                telemetry.addData("Nivel:", 1);
                telemetry.addData("Green", sensor_cor.green());
                telemetry.addData("Blue", sensor_cor.blue());
                telemetry.addData("Red", sensor_cor.red());
                telemetry.update();
                motordf.setPower(0);
                motordt.setPower(0);
                motoref.setPower(0);
                motoret.setPower(0);
                sleep(4000);
            }
            if (sensor_cor.red() > sensor_cor.green() && sensor_cor.red() > sensor_cor.blue()){
                telemetry.addData("Nivel:", 2);
                telemetry.addData("Green", sensor_cor.green());
                telemetry.addData("Blue", sensor_cor.blue());
                telemetry.addData("Red", sensor_cor.red());
                telemetry.update();
                motordf.setPower(0);
                motordt.setPower(0);
                motoref.setPower(0);
                motoret.setPower(0);
                sleep(4000);
            }
            if (sensor_cor.blue() > sensor_cor.green() && sensor_cor.blue() > sensor_cor.red()){
                telemetry.addData("Nivel:", 3);
                telemetry.addData("Green", sensor_cor.green());
                telemetry.addData("Blue", sensor_cor.blue());
                telemetry.addData("Red", sensor_cor.red());
                telemetry.update();
                motordf.setPower(0);
                motordt.setPower(0);
                motoref.setPower(0);
                motoret.setPower(0);
                sleep(4000);
            }

            /*
            motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            esquerdaTarget = (int) (300 * COUNTS_PER_MM);
            direitaTarget = (int) (-300 * COUNTS_PER_MM);
            esquerdaTarget1 = (int) (-300 * COUNTS_PER_MM);
            direitaTarget1 = (int) (300 * COUNTS_PER_MM);
            LTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;
            RTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;

            motoref.setTargetPosition(esquerdaTarget1);
            motoret.setTargetPosition(esquerdaTarget);
            motordf.setTargetPosition(direitaTarget1);
            motordt.setTargetPosition(direitaTarget);

            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motordf.setPower(LTPS * 0.3);
            motordt.setPower(RTPS * 0.3);
            motoref.setPower(LTPS * 0.3);
            motoret.setPower(RTPS * 0.3);

            while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                telemetry.addData("motordf:", motordf.getCurrentPosition());
                telemetry.addData("motordt:", motordt.getCurrentPosition());
                telemetry.addData("motoref:", motoref.getCurrentPosition());
                telemetry.addData("motoret:", motoret.getCurrentPosition());
                telemetry.update();
            }
            */
            /*
            motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            esquerdaTarget = (int) (295 * COUNTS_PER_MM);
            direitaTarget = (int) (-295 * COUNTS_PER_MM);
            LTPS = (100 / 60) * COUNTS_PER_WHEEL_REV;
            RTPS = (70 / 60) * COUNTS_PER_WHEEL_REV;

            motoref.setTargetPosition(esquerdaTarget);
            motoret.setTargetPosition(esquerdaTarget);
            motordf.setTargetPosition(direitaTarget);
            motordt.setTargetPosition(direitaTarget);

            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motordf.setPower(LTPS * 0.3);
            motordt.setPower(RTPS * 0.3);
            motoref.setPower(LTPS * 0.3);
            motoret.setPower(RTPS * 0.3);

            while (garra.getCurrentPosition() < 3300) {
                garra.setTargetPosition(3300);
                garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                garra.setPower(0.8);

            }

            while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                telemetry.addData("motordf:", motordf.getCurrentPosition());
                telemetry.addData("motordt:", motordt.getCurrentPosition());
                telemetry.addData("motoref:", motoref.getCurrentPosition());
                telemetry.addData("motoret:", motoret.getCurrentPosition());
                telemetry.update();
            }

            motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            esquerdaTarget = (int) (-120 * COUNTS_PER_MM);
            direitaTarget = (int) (-120 * COUNTS_PER_MM);
            LTPS = (100 / 60) * COUNTS_PER_WHEEL_REV;
            RTPS = (100 / 60) * COUNTS_PER_WHEEL_REV;

            motoref.setTargetPosition(esquerdaTarget);
            motoret.setTargetPosition(esquerdaTarget);
            motordf.setTargetPosition(direitaTarget);
            motordt.setTargetPosition(direitaTarget);

            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motordf.setPower(LTPS * 0.3);
            motordt.setPower(RTPS * 0.3);
            motoref.setPower(LTPS * 0.3);
            motoret.setPower(RTPS * 0.3);

            while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                telemetry.addData("motordf:", motordf.getCurrentPosition());
                telemetry.addData("motordt:", motordt.getCurrentPosition());
                telemetry.addData("motoref:", motoref.getCurrentPosition());
                telemetry.addData("motoret:", motoret.getCurrentPosition());
                telemetry.update();
            }

            motordf.setPower(0);
            motordt.setPower(0);
            motoref.setPower(0);
            motoret.setPower(0);
            sleep(1000);

            coletor.setPosition(1);
            sleep(100);

            motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            esquerdaTarget = (int) (150 * COUNTS_PER_MM);
            direitaTarget = (int) (150 * COUNTS_PER_MM);
            LTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;
            RTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;

            motoref.setTargetPosition(esquerdaTarget);
            motoret.setTargetPosition(esquerdaTarget);
            motordf.setTargetPosition(direitaTarget);
            motordt.setTargetPosition(direitaTarget);

            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motordf.setPower(LTPS * 0.6);
            motordt.setPower(RTPS * 0.6);
            motoref.setPower(LTPS * 0.6);
            motoret.setPower(RTPS * 0.6);

            while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                telemetry.addData("motordf:", motordf.getCurrentPosition());
                telemetry.addData("motordt:", motordt.getCurrentPosition());
                telemetry.addData("motoref:", motoref.getCurrentPosition());
                telemetry.addData("motoret:", motoret.getCurrentPosition());
                telemetry.update();
            }

            motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            esquerdaTarget = (int) (-270 * COUNTS_PER_MM);
            direitaTarget = (int) (270 * COUNTS_PER_MM);
            LTPS = (100 / 60) * COUNTS_PER_WHEEL_REV;
            RTPS = (70 / 60) * COUNTS_PER_WHEEL_REV;

            motoref.setTargetPosition(esquerdaTarget);
            motoret.setTargetPosition(esquerdaTarget);
            motordf.setTargetPosition(direitaTarget);
            motordt.setTargetPosition(direitaTarget);

            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motordf.setPower(LTPS * 0.6);
            motordt.setPower(RTPS * 0.6);
            motoref.setPower(LTPS * 0.6);
            motoret.setPower(RTPS * 0.6);

            while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                telemetry.addData("motordf:", motordf.getCurrentPosition());
                telemetry.addData("motordt:", motordt.getCurrentPosition());
                telemetry.addData("motoref:", motoref.getCurrentPosition());
                telemetry.addData("motoret:", motoret.getCurrentPosition());
                telemetry.update();
            }

            while (garra.getCurrentPosition() > 0) {
                garra.setTargetPosition(0);
                garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                garra.setPower(0.8);

            }

            motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            esquerdaTarget = (int) (-400 * COUNTS_PER_MM);
            direitaTarget = (int) (-400 * COUNTS_PER_MM);
            LTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;
            RTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;

            motoref.setTargetPosition(esquerdaTarget);
            motoret.setTargetPosition(esquerdaTarget);
            motordf.setTargetPosition(direitaTarget);
            motordt.setTargetPosition(direitaTarget);

            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motordf.setPower(LTPS * 0.6);
            motordt.setPower(RTPS * 0.6);
            motoref.setPower(LTPS * 0.6);
            motoret.setPower(RTPS * 0.6);

            while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                telemetry.addData("motordf:", motordf.getCurrentPosition());
                telemetry.addData("motordt:", motordt.getCurrentPosition());
                telemetry.addData("motoref:", motoref.getCurrentPosition());
                telemetry.addData("motoret:", motoret.getCurrentPosition());
                telemetry.update();
            }

            motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            esquerdaTarget = (int) (-300 * COUNTS_PER_MM);
            direitaTarget = (int) (300 * COUNTS_PER_MM);
            LTPS = (100 / 60) * COUNTS_PER_WHEEL_REV;
            RTPS = (70 / 60) * COUNTS_PER_WHEEL_REV;

            motoref.setTargetPosition(esquerdaTarget);
            motoret.setTargetPosition(esquerdaTarget);
            motordf.setTargetPosition(direitaTarget);
            motordt.setTargetPosition(direitaTarget);

            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motordf.setPower(LTPS * 0.6);
            motordt.setPower(RTPS * 0.6);
            motoref.setPower(LTPS * 0.6);
            motoret.setPower(RTPS * 0.6);

            while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                telemetry.addData("motordf:", motordf.getCurrentPosition());
                telemetry.addData("motordt:", motordt.getCurrentPosition());
                telemetry.addData("motoref:", motoref.getCurrentPosition());
                telemetry.addData("motoret:", motoret.getCurrentPosition());
                telemetry.update();
            }

            motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            esquerdaTarget = (int) (-480 * COUNTS_PER_MM);
            direitaTarget = (int) (-480 * COUNTS_PER_MM);
            LTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;
            RTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;

            motoref.setTargetPosition(esquerdaTarget);
            motoret.setTargetPosition(esquerdaTarget);
            motordf.setTargetPosition(direitaTarget);
            motordt.setTargetPosition(direitaTarget);

            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motordf.setPower(LTPS * 0.6);
            motordt.setPower(RTPS * 0.6);
            motoref.setPower(LTPS * 0.6);
            motoret.setPower(RTPS * 0.6);

            while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                telemetry.addData("motordf:", motordf.getCurrentPosition());
                telemetry.addData("motordt:", motordt.getCurrentPosition());
                telemetry.addData("motoref:", motoref.getCurrentPosition());
                telemetry.addData("motoret:", motoret.getCurrentPosition());
                telemetry.update();
            }

            while (garra.getCurrentPosition() < 1100) {
                garra.setTargetPosition(1100);
                garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                garra.setPower(0.8);

            }

            motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            esquerdaTarget = (int) (-100 * COUNTS_PER_MM);
            direitaTarget = (int) (-100 * COUNTS_PER_MM);
            LTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;
            RTPS = (175 / 60) * COUNTS_PER_WHEEL_REV;

            motoref.setTargetPosition(esquerdaTarget);
            motoret.setTargetPosition(esquerdaTarget);
            motordf.setTargetPosition(direitaTarget);
            motordt.setTargetPosition(direitaTarget);

            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motordf.setPower(LTPS * 0.2);
            motordt.setPower(RTPS * 0.2);
            motoref.setPower(LTPS * 0.2);
            motoret.setPower(RTPS * 0.2);

            while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                telemetry.addData("motordf:", motordf.getCurrentPosition());
                telemetry.addData("motordt:", motordt.getCurrentPosition());
                telemetry.addData("motoref:", motoref.getCurrentPosition());
                telemetry.addData("motoret:", motoret.getCurrentPosition());
                telemetry.update();
            }

            coletor.setPosition(0);
            sleep(100);
            */
        }
    }
}
