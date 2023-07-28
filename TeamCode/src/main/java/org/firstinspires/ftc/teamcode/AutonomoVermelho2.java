package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="AutonomoVermelhoDireito", group="Linear Opmode")
public class AutonomoVermelho2 extends LinearOpMode {

    private DcMotor motoref = null;
    private DcMotor motoret = null;
    private DcMotor motordf = null;
    private DcMotor motordt = null;
    private DcMotor garra = null;
    private Servo coletor = null;
    TouchSensor sensor_toque;
    RevColorSensorV3 sensor_cor;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .50, correction;

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

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motordt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motoref.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motoret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        garra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        garra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        garra.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }


        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        coletor.setPosition(0);
        waitForStart();

        if (opModeIsActive()) {

            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            while (garra.getCurrentPosition() < 1200) {
                garra.setTargetPosition(1200);
                garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                garra.setPower(0.8);
            }


            motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            int esquerdaTarget = (int) (-650 * COUNTS_PER_MM);
            int direitaTarget = (int) (-650 * COUNTS_PER_MM);


            motoref.setTargetPosition(esquerdaTarget);
            motoret.setTargetPosition(esquerdaTarget);
            motordf.setTargetPosition(direitaTarget);
            motordt.setTargetPosition(direitaTarget);

            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motordf.setPower(-power + correction);
            motordt.setPower(-power + correction);
            motoref.setPower(power - correction);
            motoret.setPower(power - correction);



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

            if (sensor_cor.green() > sensor_cor.blue() && sensor_cor.green() > sensor_cor.red()){
                telemetry.addData("Nivel:", 1);
                telemetry.update();

                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                 esquerdaTarget = (int) (-400 * COUNTS_PER_MM);
                 direitaTarget = (int) (-400 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);


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

                esquerdaTarget = (int) (-420 * COUNTS_PER_MM);
                direitaTarget = (int) (420 * COUNTS_PER_MM);

                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

                while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                    telemetry.addData("motordf:", motordf.getCurrentPosition());
                    telemetry.addData("motordt:", motordt.getCurrentPosition());
                    telemetry.addData("motoref:", motoref.getCurrentPosition());
                    telemetry.addData("motoret:", motoret.getCurrentPosition());
                    telemetry.update();
                }

                while (garra.getCurrentPosition() < 3390) {
                    garra.setTargetPosition(3390);
                    garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    garra.setPower(0.5);
                }

                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (-125 * COUNTS_PER_MM);
                direitaTarget = (int) (-125 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

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

                //Tras
                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (140 * COUNTS_PER_MM);
                direitaTarget = (int) (140 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

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
                    garra.setPower(0.5);
                }

                //Giro
                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (400 * COUNTS_PER_MM);
                direitaTarget = (int) (-400 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

                //Trás
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

                esquerdaTarget = (int) (-280 * COUNTS_PER_MM);
                direitaTarget = (int) (-280 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

                while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                    telemetry.addData("motordf:", motordf.getCurrentPosition());
                    telemetry.addData("motordt:", motordt.getCurrentPosition());
                    telemetry.addData("motoref:", motoref.getCurrentPosition());
                    telemetry.addData("motoret:", motoret.getCurrentPosition());
                    telemetry.update();
                }

                //Giro
                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (-400 * COUNTS_PER_MM);
                direitaTarget = (int) (400 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);


                while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                    telemetry.addData("motordf:", motordf.getCurrentPosition());
                    telemetry.addData("motordt:", motordt.getCurrentPosition());
                    telemetry.addData("motoref:", motoref.getCurrentPosition());
                    telemetry.addData("motoret:", motoret.getCurrentPosition());
                    telemetry.update();
                }

                //Giro
                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (-600 * COUNTS_PER_MM);
                direitaTarget = (int) (-600 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

                //Trás
                while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                    telemetry.addData("motordf:", motordf.getCurrentPosition());
                    telemetry.addData("motordt:", motordt.getCurrentPosition());
                    telemetry.addData("motoref:", motoref.getCurrentPosition());
                    telemetry.addData("motoret:", motoret.getCurrentPosition());
                    telemetry.update();
                }

            }
            if (sensor_cor.red() > sensor_cor.green() && sensor_cor.red() > sensor_cor.blue()){
                telemetry.addData("Nivel:", 2);
                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (-400 * COUNTS_PER_MM);
                direitaTarget = (int) (-400 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);


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

                esquerdaTarget = (int) (-420 * COUNTS_PER_MM);
                direitaTarget = (int) (420 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

                while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                    telemetry.addData("motordf:", motordf.getCurrentPosition());
                    telemetry.addData("motordt:", motordt.getCurrentPosition());
                    telemetry.addData("motoref:", motoref.getCurrentPosition());
                    telemetry.addData("motoret:", motoret.getCurrentPosition());
                    telemetry.update();
                }

                while (garra.getCurrentPosition() < 3390) {
                    garra.setTargetPosition(3390);
                    garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    garra.setPower(0.5);
                }

                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (-125 * COUNTS_PER_MM);
                direitaTarget = (int) (-125 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

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

                //Tras
                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (140 * COUNTS_PER_MM);
                direitaTarget = (int) (140 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

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
                    garra.setPower(0.5);
                }

            }
            if (sensor_cor.blue() > sensor_cor.green() && sensor_cor.blue() > sensor_cor.red()){
                telemetry.addData("Nivel:", 3);
                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (-400 * COUNTS_PER_MM);
                direitaTarget = (int) (-400 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);


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

                esquerdaTarget = (int) (-420 * COUNTS_PER_MM);
                direitaTarget = (int) (420 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

                while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                    telemetry.addData("motordf:", motordf.getCurrentPosition());
                    telemetry.addData("motordt:", motordt.getCurrentPosition());
                    telemetry.addData("motoref:", motoref.getCurrentPosition());
                    telemetry.addData("motoret:", motoret.getCurrentPosition());
                    telemetry.update();
                }

                while (garra.getCurrentPosition() < 3390) {
                    garra.setTargetPosition(3390);
                    garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    garra.setPower(0.5);
                }

                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (-125 * COUNTS_PER_MM);
                direitaTarget = (int) (-125 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

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

                //Tras
                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (140 * COUNTS_PER_MM);
                direitaTarget = (int) (140 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

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
                    garra.setPower(0.5);
                }

                //Giro
                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (400 * COUNTS_PER_MM);
                direitaTarget = (int) (-400 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

                //Trás
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

                esquerdaTarget = (int) (-280 * COUNTS_PER_MM);
                direitaTarget = (int) (-280 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

                while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                    telemetry.addData("motordf:", motordf.getCurrentPosition());
                    telemetry.addData("motordt:", motordt.getCurrentPosition());
                    telemetry.addData("motoref:", motoref.getCurrentPosition());
                    telemetry.addData("motoret:", motoret.getCurrentPosition());
                    telemetry.update();
                }

                //Giro
                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (500 * COUNTS_PER_MM);
                direitaTarget = (int) (-500 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);


                while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                    telemetry.addData("motordf:", motordf.getCurrentPosition());
                    telemetry.addData("motordt:", motordt.getCurrentPosition());
                    telemetry.addData("motoref:", motoref.getCurrentPosition());
                    telemetry.addData("motoret:", motoret.getCurrentPosition());
                    telemetry.update();
                }

                //Giro
                motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                esquerdaTarget = (int) (-600 * COUNTS_PER_MM);
                direitaTarget = (int) (-600 * COUNTS_PER_MM);


                motoref.setTargetPosition(esquerdaTarget);
                motoret.setTargetPosition(esquerdaTarget);
                motordf.setTargetPosition(direitaTarget);
                motordt.setTargetPosition(direitaTarget);

                motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motordf.setPower(-power + correction);
                motordt.setPower(-power + correction);
                motoref.setPower(power - correction);
                motoret.setPower(power - correction);

                //Trás
                while (opModeIsActive() && (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {
                    telemetry.addData("motordf:", motordf.getCurrentPosition());
                    telemetry.addData("motordt:", motordt.getCurrentPosition());
                    telemetry.addData("motoref:", motoref.getCurrentPosition());
                    telemetry.addData("motoret:", motoret.getCurrentPosition());
                    telemetry.update();
                }
            }
        }
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        motoref.setPower(leftPower);
        motordf.setPower(rightPower);
        motoret.setPower(leftPower);
        motordt.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        motordf.setPower(0);
        motordf.setPower(0);
        motoref.setPower(0);
        motoret.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}