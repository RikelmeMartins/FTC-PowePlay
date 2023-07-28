package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="TestePID", group="Linear Opmode")
public class Pid extends LinearOpMode
{

    DcMotorEx       motoref = null;
    DcMotorEx       motoret = null;
    DcMotorEx       motordf = null;
    DcMotorEx       motordt = null;
    private         BNO055IMU imu;
    double          intregalSum = 0;
    double          Kp = 1;
    double          Ki = 0;
    double          Kd = 0;
    double          Kf = 0;
    ElapsedTime     timer = new ElapsedTime();
    private double  LastErro = 0;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .80, correction;

    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     DRIVE_GEAR_REDUCTION    = 30.24;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 90.0 * 3.14;

    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;

    @Override
    public void runOpMode()
    {

        motoref = hardwareMap.get(DcMotorEx.class, "ef");
        motoref.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motoret = hardwareMap.get(DcMotorEx.class, "et");
        motoret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motordf = hardwareMap.get(DcMotorEx.class, "df");
        motordf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motordt = hardwareMap.get(DcMotorEx.class, "dt");
        motordt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motoref.setDirection(DcMotor.Direction.REVERSE);
        motoret.setDirection(DcMotor.Direction.REVERSE);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motordt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motoref.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motoret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motordt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motoref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motoret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);



        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        double refenceAngle = Math.toRadians(90);

        if (opModeIsActive())
        {
            correction = checkDirection();

            power = PidControl(refenceAngle, imu.getAngularOrientation().firstAngle);

            motordf.setPower(-power + correction);
            motordt.setPower(-power + correction);
            motoref.setPower(power - correction);
            motoret.setPower(power - correction);

        }
    }

    public  double PidControl(double reference, double state)
    {
        double erro = angleWrap(reference - state);
        intregalSum  += erro * timer.seconds();
        double devirative = (erro - LastErro) / timer.seconds();
        LastErro = erro;

        timer.reset();

        double output = (erro * Kp) + (devirative * Kd) + (intregalSum * Ki) + (reference * Kf);

        return output;
    }

    public double angleWrap (double radians)
    {
        while (radians > Math.PI)
        {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI)
        {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

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

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

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
