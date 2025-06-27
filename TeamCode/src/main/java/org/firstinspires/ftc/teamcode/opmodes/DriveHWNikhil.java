package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;



public class DriveHWNikhil extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // create 4 motors, leftFront, left Rear, right front, and right rear
        // read the prioritymotor class if you don't know how to make one
        // remember hardware map exists :3
        Robot robot = new Robot(hardwareMap);

        PriorityMotor leftFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftFont"), "leftFront", 4, 5, robot.sensors);
        PriorityMotor rightFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightFont"), "rightFront", 4, 5, robot.sensors);
        PriorityMotor leftRear = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftRear"), "leftRear", 4, 5, robot.sensors);
        PriorityMotor rightRear = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightRear"), "rightRear", 4, 5, robot.sensors);

        // init
        while(opModeInInit()) robot.update();

        // going to need a while loop for updating
        // use the left and right joysticks
        // left is for magnitude of power supplied
        // right is for turning
        double x, y, r, lf, rf, lr, rr, absMax;
        while(!isStopRequested()) {

            x = -gamepad1.left_stick_y;
            y = gamepad1.left_stick_x;
            r = gamepad1.right_stick_x;
            lf = x + y + r;
            rf = x - y - r;
            lr = x - y + r;
            rr = x + y - r;
            absMax = findAbsMax(lf, rf, lr, rr);
            if (absMax > 1.0) {
                lf /= absMax;
                rf /= absMax;
                lr /= absMax;
                rr /= absMax;
            }
            leftFront.setTargetPower(lf);
            rightFront.setTargetPower(rf);
            leftRear.setTargetPower(lr);
            rightRear.setTargetPower(rr);

        }

    }

    private double findAbsMax(double... arr) {
        if (arr.length == 0) return -1;
        double max = Math.abs(arr[0]);
        for(int i = 1; i < arr.length; i++) {
            if (Math.abs(arr[i]) > max) max = Math.abs(arr[i]);
        }

        return max;
    }



}
