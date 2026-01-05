
package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;

/**
 * An OpMode that finds servo values. Copied from last year's code.
 */
@TeleOp(name = "Old Servo Value Finder", group = "testing")
public class OldServoValueFinder extends LinearOpMode {
    private static String[] SERVO_LIST;
    private static int num = 0;
    private boolean dpad_right_prev = false;
    private boolean dpad_left_prev = false;

    /**
     * Runs the OpMode.
     */
    @Override
    public void runOpMode() {
        SERVO_LIST = hardwareMap.servo.entrySet().stream().map(Map.Entry::getKey).toArray(String[]::new);
        waitForStart();
        Servo servo = getServo();
        while(opModeIsActive()){
            if(gamepad2.b){
                servo = getServo();
            }else if(gamepad2.dpad_up){
                servo.setPosition(servo.getPosition()+0.0001);
            }else if(gamepad2.dpad_down){
                servo.setPosition(servo.getPosition()-0.0001);
            }
            telemetry.addData("servo pos: ",servo.getPosition());
            telemetry.update();
        }
    }
    public Servo getServo(){
        while(!gamepad2.a && !isStopRequested()){
            if(gamepad2.dpad_right && !dpad_right_prev){
                num = (num + 1) % SERVO_LIST.length;
                dpad_right_prev = true;
            }else if(gamepad2.dpad_left && !dpad_left_prev){
                num = (num + SERVO_LIST.length - 1) % SERVO_LIST.length;
                dpad_left_prev = true;
            }
            if(!gamepad2.dpad_right){
                dpad_right_prev = false;
            }
            if(!gamepad2.dpad_left){
                dpad_left_prev = false;
            }
            telemetry.addData("selected servo: ", SERVO_LIST[num]);
            telemetry.update();
        }
        return hardwareMap.get(Servo.class, SERVO_LIST[num]);
    }
}