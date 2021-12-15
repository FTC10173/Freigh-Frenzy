package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class JarrettZane extends LinearOpMode {
    //sticks are -1 - 1 triggers are 0/1 everything else is true/false DPad is true/false but in each direction
    //Driver
    //-1 - 1  return 0 if don't want to use one of these
    public double movex(){ return gamepad1.left_stick_x; } //Move left/right
    public double movey(){ return -gamepad1.left_stick_y; } //Move forward/backward
    public double mover(){ return gamepad1.right_stick_x;} //Rotate
    //true/false return false if don't want ot use one of these
    public boolean moveL(){return false;} //Move left
    public boolean moveR(){return false;} //Move right
    public boolean moveF(){return false;} //Move Forward
    public boolean moveB(){return false;} //Move Backward
    public boolean StartForward(){return gamepad1.dpad_up;}
    public boolean StartLeft(){return gamepad1.dpad_left;}
    public boolean StartRight(){return gamepad1.dpad_right;}
    public boolean StartBackwards(){return gamepad1.dpad_down;}
    //if you want movement to be based off your POV (true) or robot POV (false)
    public boolean DriverMode() { return false;}
    public double DriveSpeed(){ return .75; } //% of max speed
    public double RotationSpeed(){return .45;} //% of max rotation speed

    //Extra
    //wants to move up down left joystick y
    //wants to rotate (R-1 L+1 up)(R+1 L-1 down)
    public double CarouselSpeed(){ return 1; } //% of max speed
    // -1 - 1
    public boolean CarouselMan(){return false;} // toggles manual carousel speed or pre set per side
    public double Carousel1(){ return 0; } //Carousel wants to have blue team x and red team b
    public boolean CarouselBlue(){return gamepad2.x;}
    public boolean CarouselRed(){return gamepad2.b;}
    public boolean LevelMan(){return true;}//toggles manual levels or pre set levels
    public double Levely(){return gamepad2.left_stick_y;} //min of 20%
    public double Levelr(){return -gamepad2.right_stick_y;}
    // true/false return false if don't want to use one of these
    public boolean Level0(){ return false;}
    public boolean Level1(){ return false;}
    public boolean Level2(){ return false;}
    public boolean Level3(){ return false;}
    public boolean Leveladd(){ return false;} //increase level return
    public boolean Levelsub(){ return false;} //decrease level return
    public boolean ArmToggle(){return gamepad2.dpad_up;} //will toggle Open vs Closed return
    public boolean Open(){return gamepad2.a;}
    public boolean Close(){return gamepad2.y;}





    //this is not used however it must be here (will never run)
    public void runOpMode(){
    }
}






