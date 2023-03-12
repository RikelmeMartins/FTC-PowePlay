package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="TeleOp2023", group="Linear Opmode")

public class TeleOp1 extends LinearOpMode {

    private DcMotor motoref = null;
    private DcMotor motoret = null;
    private DcMotor motordf = null;
    private DcMotor motordt = null;
    private DcMotor garra = null;
    private Servo coletor = null;
    TouchSensor sensor_toque;
    int upPositionH = 4420;
    int upPositionM = 3330;
    int upPositionL = 2030;
    int position;
    int erro;
    double kp = 0.02;
    double power;
    Boolean manual = Boolean.FALSE;

    @Override
    public void runOpMode() {

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
        garra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        garra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (garra.getCurrentPosition() != 0) {
            idle();
        }

        garra.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        coletor.setPosition(1);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
//-------------------------------MOBILIDADE------------------------------------------------
            telemetry.addData("Garra", garra.getCurrentPosition());
            telemetry.update();

            double max;

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double eixo = gamepad1.right_stick_x;

            double motoretP = y + x + eixo;
            double motordtP = y - x - eixo;
            double motorefP = y - x + eixo;
            double motordfP = y + x - eixo;

            max = Math.max(Math.abs(motorefP), Math.abs(motordfP));
            max = Math.max(max, Math.abs(motoretP));
            max = Math.max(max, Math.abs(motordtP));
            if (max > 1.0) {
                motorefP /= max;
                motordfP /= max;
                motoretP /= max;
                motordtP /= max;
            }
            motoref.setPower(motorefP);
            motordf.setPower(motordfP);
            motoret.setPower(motoretP);
            motordt.setPower(motordtP);

//-------------------------------MOBILIDADE GARRA------------------------------------------------MovimentacaoMovimentacao

                if (sensor_toque.isPressed()) {
                    garra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    garra.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    garra.setPower(0);
                    telemetry.addData("Encoder resetado!", "1");
                    telemetry.update();
                }
                //Opções de Altura da Garra
                if (gamepad2.y) {//Junção Alta
                    position = garra.getCurrentPosition();
                    erro = position - (4420);
                    garra.setTargetPosition(upPositionH);
                    garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.b) { //Junção media
                    position = garra.getCurrentPosition();
                    erro = position - (3330);
                    garra.setTargetPosition(upPositionM);
                    garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.x) { //Junção baixa
                    position = garra.getCurrentPosition();
                    erro = position - (2030);
                    garra.setTargetPosition(upPositionL);
                    garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.a) {
                    garra.setTargetPosition(-15);
                    garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    garra.setPower(0.4);
                } else if (gamepad2.dpad_up) {
                    position = garra.getCurrentPosition();
                    erro = position - (750);
                    garra.setTargetPosition(750);
                    garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.dpad_right) {
                    position = garra.getCurrentPosition();
                    erro = position - (600);
                    garra.setTargetPosition(600);
                    garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.dpad_down) {
                    position = garra.getCurrentPosition();
                    erro = position - (400);
                    garra.setTargetPosition(400);
                    garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.dpad_left) {
                    position = garra.getCurrentPosition();
                    erro = position - (200);
                    garra.setTargetPosition(200);
                    garra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                power = erro * kp;
                if (garra.isBusy()) {
                    garra.setPower(power);
                }

            if(gamepad2.left_bumper){
                coletor.setPosition(1);
                sleep(200);
            }
            if(gamepad2.right_bumper){
                coletor.setPosition(0);
                sleep(200);
            }
//----------------------------------------------------------------------------------------------
        }
    }
}