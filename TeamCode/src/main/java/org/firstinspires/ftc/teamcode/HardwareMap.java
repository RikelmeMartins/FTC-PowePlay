package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

public class HardwareMap {

        public DcMotor         motordf = null;
        public DcMotor         motordt = null;
        public DcMotor         motoref = null;
        public DcMotor         motoret = null;
        public DcMotor         garra = null;
        public CRServo         coletor = null;
        public SensorColor     sensor_cor = null;
        public DigitalChannel  sensor_toque = null;

        HardwareMap hardwareMap;

        public void init(HardwareMap hardwareMap){

        }
}
