package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Movimentacao{

    private DcMotor motordf = null;
    private DcMotor motoref = null;
    private DcMotor motordt = null;
    private DcMotor motoret = null;

    @SuppressLint("NotConstructor")
    public void Movimentacao(){

        double max;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double eixo = gamepad1.right_stick_x;

        double motorefP  = y + x + eixo;
        double motordfP = y - x - eixo;
        double motoretP  = y - x + eixo;
        double motordtP  = y + x - eixo;

        max = Math.max(Math.abs(motorefP), Math.abs(motordfP));
        max = Math.max(max, Math.abs(motoretP));
        max = Math.max(max, Math.abs(motordtP));

        if (max > 1.0) {
            motorefP  /= max;
            motordfP /= max;
            motoretP   /= max;
            motordtP  /= max;
        }

        motoref.setPower(motorefP);
        motordf.setPower(motordfP);
        motoret.setPower(motoretP);
        motordt.setPower(motordtP);

    }
}
