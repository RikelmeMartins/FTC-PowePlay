package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Pid extends LinearOpMode
{

    DcMotorEx motoref = null;
    DcMotorEx motoret = null;
    DcMotorEx motordf = null;
    DcMotorEx motordt = null;

    private BNO055IMU imu;

    double intregalSum = 0;
    double Kp = 1;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;

    ElapsedTime timer = new ElapsedTime();

    private double LastErro = 0;

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


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(parameters);

        waitForStart();

        double refenceAngle = Math.toRadians(90);
        while (opModeIsActive())
        {

            double power = PidControl(refenceAngle, imu.getAngularOrientation().firstAngle);
            motordf.setPower(power);
            motordt.setPower(power);
            motoref.setPower(power);
            motoret.setPower(power);

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
}
