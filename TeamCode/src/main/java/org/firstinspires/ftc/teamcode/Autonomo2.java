package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Autonomo1.0", group="Robot")
public class Autonomo2 extends LinearOpMode {

    private DcMotor         motordf   = null;
    private DcMotor         motordt  = null;
    private DcMotor         motoref   = null;
    private DcMotor         motoret  = null;
    private DcMotor         garra = null;
    private CRServo         coletor = null;
    //RevColorSensorV3 sensor_cor;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28.0 ;
    static final double     DRIVE_GEAR_REDUCTION    = 30.24 ;
    static final double     WHEEL_CIRCUMFERENCE_MM = 90.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_CIRCUMFERENCE_MM * 3.1415);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        motordf  = hardwareMap.get(DcMotor.class, "df");
        motordt = hardwareMap.get(DcMotor.class, "dt");
        motoref  = hardwareMap.get(DcMotor.class, "ef");
        motoret = hardwareMap.get(DcMotor.class, "et");
        //sensor_cor = hardwareMap.get(RevColorSensorV3.class, "sensor_cor");

        motordf.setDirection(DcMotor.Direction.REVERSE);
        motoref.setDirection(DcMotor.Direction.FORWARD);
        motordt.setDirection(DcMotor.Direction.REVERSE);
        motoret.setDirection(DcMotor.Direction.FORWARD);

        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motordt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motoref.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motoret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at",  "%7d :%7d",
                          motordf.getCurrentPosition(),
                          motordt.getCurrentPosition(),
                          motoref.getCurrentPosition(),
                          motoret.getCurrentPosition());
        telemetry.update();

        waitForStart();

        encoderDrive(DRIVE_SPEED,  20,  20, 20, 20,5.0);
        encoderDrive(TURN_SPEED,   12, -12, 12, -12, 4.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    public void encoderDrive(double speed,
                             double motorefT, double motordfT, double motordtT, double motoretT,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget1;
        int newRightTarget1;

        if (opModeIsActive()) {

            newLeftTarget1 = motordf.getCurrentPosition() + (int)(motordfT * COUNTS_PER_INCH);
            newRightTarget1 = motordt.getCurrentPosition() + (int)(motordtT * COUNTS_PER_INCH);
            newLeftTarget = motoref.getCurrentPosition() + (int)(motorefT * COUNTS_PER_INCH);
            newRightTarget = motoret.getCurrentPosition() + (int)(motoretT * COUNTS_PER_INCH);
            motordf.setTargetPosition(newLeftTarget1);
            motordt.setTargetPosition(newRightTarget1);
            motoref.setTargetPosition(newLeftTarget1);
            motoret.setTargetPosition(newRightTarget1);


            motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motordt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoref.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motoret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motordf.setPower(Math.abs(speed));
            motordt.setPower(Math.abs(speed));
            motoref.setPower(Math.abs(speed));
            motoret.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (motordf.isBusy() && motordt.isBusy() && motoref.isBusy() && motoret.isBusy())) {

                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            motordf.getCurrentPosition(), motordt.getCurrentPosition(),
                                            motoref.getCurrentPosition(), motoret.getCurrentPosition());
                telemetry.update();
            }

            motordf.setPower(0);
            motordt.setPower(0);
            motoref.setPower(0);
            motoret.setPower(0);

            motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motordt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motoref.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motoret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
