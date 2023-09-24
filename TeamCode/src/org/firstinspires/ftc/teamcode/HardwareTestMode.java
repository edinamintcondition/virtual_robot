package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous
public class HardwareTestMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        long start = System.nanoTime();

        waitForStart();
        while (opModeIsActive()) {
            long t = (System.nanoTime() - start) / 1_000_000_000;
            if (t % 5 != 0)
                continue;

            boolean pos = (t % 10) < 5;

            for (String k : hardwareMap.dcMotor.keySet()) {
                DcMotor m = hardwareMap.dcMotor.get(k);
                m.setPower(pos ? 1 : -1);
                telemetry.addData(m.getDeviceName(), k + "=" + m.getPower());
            }

            for (String k : hardwareMap.servo.keySet()) {
                Servo s = hardwareMap.servo.get(k);
                s.setPosition(pos ? 0.1 : -0.1);
                telemetry.addData(s.getDeviceName(), k + "=" + s.getPosition());
            }

            for (String k : hardwareMap.crservo.keySet()) {
                CRServo s = hardwareMap.crservo.get(k);
                s.setPower(pos ? 0.1 : -0.1);
                telemetry.addData(s.getDeviceName(), k + "=" + s.getPower());
            }

            for (String k : hardwareMap.colorSensor.keySet()) {
                ColorSensor s = hardwareMap.colorSensor.get(k);
                telemetry.addData(s.getDeviceName(), k + "=" + s.red() + "," + s.green() + "," + s.blue());
            }

            for (String k : hardwareMap.gyroSensor.keySet()) {
                GyroSensor g = hardwareMap.gyroSensor.get(k);
                telemetry.addData(g.getDeviceName(), k + "=" + g.getHeading());
            }

            for (String k : hardwareMap.voltageSensor.keySet()) {
                VoltageSensor g = hardwareMap.voltageSensor.get(k);
                telemetry.addData(g.getDeviceName(), k + "=" + g.getVoltage());
            }

            telemetry.update();
        }
    }
}
