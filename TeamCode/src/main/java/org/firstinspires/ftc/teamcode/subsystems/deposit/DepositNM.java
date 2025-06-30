package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotNM;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class DepositNM {

    private PriorityMotor vert1, vert2;

    private nPriorityServo clawGrab, clawRotation, lateralV4Bar1, lateralV4Bar2, lateralExtendo1, lateralExtendo2;

    private RobotNM robot;

    public DepositNM(RobotNM robot) {
        this.robot = robot;
        vert1 = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "vert1"), "vert1", 5,5, robot.sensors);
        vert2 = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "vert2"), "vert2", 5,5, robot.sensors);

        clawGrab = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "clawGrab")}, "clawGrab", nPriorityServo.ServoType.SPEED, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        clawRotation = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "clawRotation")}, "clawRotation", nPriorityServo.ServoType.SPEED, 0, 1, 0.5, new boolean[]{false}, 3, 5);

        lateralV4Bar1 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "lateralV4Bar1")}, "lateralV4Bar1", nPriorityServo.ServoType.SPEED, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        lateralV4Bar2 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "lateralV4Bar2")}, "lateralV4Bar2", nPriorityServo.ServoType.SPEED, 0, 1, 0.5, new boolean[]{false}, 3, 5);

        lateralExtendo1 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "lateralExtendo1")}, "lateralExtendo1", nPriorityServo.ServoType.SPEED, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        lateralExtendo2 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "lateralExtendo2")}, "lateralExtendo2", nPriorityServo.ServoType.SPEED, 0, 1, 0.5, new boolean[]{false}, 3, 5);

        robot.hardwareQueue.addDevices(vert1, vert2, clawGrab, clawRotation, lateralV4Bar1, lateralV4Bar2, lateralExtendo1, lateralExtendo2);

    }

    public void update() {



    }


}
