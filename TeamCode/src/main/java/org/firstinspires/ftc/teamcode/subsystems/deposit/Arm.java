package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Arm {
    public final Sensors sensors;

    public final nPriorityServo armRotation;
    public final nPriorityServo extendo;

    public static double openRad = 1.2456, closeRad = 0.10, closeLooseRad = 0.25;

    public Arm(Robot robot) {
        this.sensors = robot.sensors;

        armRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "depositArmRotationL"), robot.hardwareMap.get(Servo.class, "depositArmRotationR")},
                "depositArmRotation",
                nPriorityServo.ServoType.AXON_MINI, // TODO: find out the type of the servo
                0.335, // TODO: find parameters
                0.982, // TODO
                0.871, // TODO
                new boolean[] {false, true},
                1.0,
                2.0
        );
        robot.hardwareQueue.addDevice(armRotation);

        extendo = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "depositArmSlidesServo0"), robot.hardwareMap.get(ServoImpl.class, "depositArmSlidesServo1")},
                "depositArmSlidesMotor",
                nPriorityServo.ServoType.AXON_MAX, // TODO: find out the type of servo used here
                0, // TODO: find parameters
                1, // TODO
                0.32, // TODO
                new boolean[] {false, true}, // TODO: idk the proper order here, figure that out
                1.0,
                2.0
        );
        robot.hardwareQueue.addDevices(armRotation, extendo);
    }

    public void setArmRotation(double targetRad, double power) { armRotation.setTargetAngle(targetRad, power); }

    public boolean inPosition() { return armRotation.inPosition() && extendo.inPosition(); }
}
