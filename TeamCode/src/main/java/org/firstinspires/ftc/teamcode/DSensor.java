package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DSensor {

    boolean isBlue;
    float[] posOnRobot;
    float angle;
    double modAngle;
    com.qualcomm.robotcore.hardware.DistanceSensor sensor;
    public enum directions //red is west; blue is east
    {
        E, NE, N, NW, W, SW, S, SE
    }
    public static directions[] dArray = new directions[]{directions.E, directions.NE, directions.N, directions.NW, directions.W, directions.SW, directions.S, directions.SE};
    directions direction;

    public DSensor(boolean _isBlue, float[] _posOnRobot, int _angle, com.qualcomm.robotcore.hardware.DistanceSensor _sensor)
    {
        isBlue = _isBlue;
        posOnRobot = _posOnRobot;
        angle = _angle;
        sensor = _sensor;
    }
    public directions getDirection(double _angle)
    {
        directions e;
        getModAngle(_angle);
        if(0 <= modAngle && modAngle < 45)
        {
            e = directions.E;
        }else if(45 <= modAngle && modAngle < 135)
        {
            e = directions.N;
        }
        else if(135 <= modAngle && modAngle < 225)
        {
            e = directions.W;
        }else if(225 <= modAngle && modAngle < 315)
        {
            e = directions.S;
        }else{
            e = directions.E;
        }
        direction = e;
        return e;
    }
    public directions getDirection()
    {
        return direction;
    }

    public void setDirection(directions _d)
    {
        direction = _d;
    }

    public double getValue()
    {
        return sensor.getDistance(DistanceUnit.INCH);
    }
    public double getModAngle(double _rotation)
    {
        if(isBlue)
        {
            modAngle = mod(_rotation + angle + 180, 360d);
        }else
        {
            modAngle = mod(_rotation + angle, 360d);
        }

        return modAngle;
    }

    public static directions incNinety(directions d)
    {
        return dArray[((d.ordinal() + 2) % 8)];
    }
    public static int mod(int _a, int _b)//isn't set up for negatives because it shouldn't run into them
    {
            int r = _a;
            for(int i = _a; i > _b;)
            {
                i -= _b;
                r = i;
            }
            return  r;
    }
    public static double mod(double _a, double _b)//this is set up for negatives
    {
        if(_a > _b)
        {
            double p = _a;
            for(double i = _a; i > _b;)
            {
                i -= _b;
                p = i;
            }
            return  p;
        }else if(_a < 0)
        {
            double n = _a;
            for(double i = _a; i < 0;)
            {
                i += _b;
                n = i;
            }
            return  n;
        }else
        {
            return  _a;
        }
    }
    public double getX(double _rotation)//returns the x-distance from the object the sensor sees
    {
        double angle = getModAngle(_rotation);
        return Math.abs(sensor.getDistance(DistanceUnit.INCH) * Math.cos(Math.toRadians(angle)));
    }

    public double getDistance(double _rotation) //returns the distance from the sensor to a wall, assuming that the wall is perpendicular to the cardinal direction the sensor is facing
    {
        double angle = getModAngle(_rotation);
        if(direction == directions.E || direction == directions.W)
        {
            return Math.abs(sensor.getDistance(DistanceUnit.INCH) * Math.cos(Math.toRadians(angle)));
        }else
        {
            return Math.abs(sensor.getDistance(DistanceUnit.INCH) * Math.sin(Math.toRadians(angle)));
        }
    }

    public double getCenterDistance(double _rotation) //returns the distance from the center of the robot to a wall, assuming that the wall is perpendicular to the cardinal direction the sensor is facing
    {
        double angle = getModAngle(_rotation);
        if(direction == directions.E || direction == directions.W)
        {
            return Math.abs((sensor.getDistance(DistanceUnit.INCH) + posOnRobot[0]) * Math.cos(Math.toRadians(angle)));
        }else
        {
            return Math.abs((sensor.getDistance(DistanceUnit.INCH) - posOnRobot[1]) * Math.sin(Math.toRadians(angle)));
        }
    }

    public double getPos(double _rotation)//returns the x-or-y-position of the bot
    {
        double angle = getModAngle(_rotation);
        if(getDirection() == directions.E)
        {
            return 72 - (sensor.getDistance(DistanceUnit.INCH) + posOnRobot[0]) * Math.cos(Math.toRadians(angle));
        }
        else if(getDirection() == directions.W){
            return Math.abs((sensor.getDistance(DistanceUnit.INCH) - posOnRobot[0]) * Math.cos(Math.toRadians(angle))) - 72;
        }else if(getDirection() == directions.N){
            return 72 - (sensor.getDistance(DistanceUnit.INCH) + posOnRobot[1]) * Math.sin(Math.toRadians(angle));
        }else {
            return Math.abs((sensor.getDistance(DistanceUnit.INCH) - posOnRobot[1]) * Math.sin(Math.toRadians(angle))) - 72;
        }
    }

    public double getPosX(double _rotation)//returns the x-position of the bot. Make sure this is only called on a east-or-west-facing sensor
    {
        double angle = getModAngle(_rotation);
        if(getDirection() == directions.E)
        {
            return 72 - (posOnRobot[0] + sensor.getDistance(DistanceUnit.INCH)) * Math.cos(Math.toRadians(angle));
        }
        else{
            return (sensor.getDistance(DistanceUnit.INCH) - posOnRobot[0]) * Math.cos(Math.toRadians(angle)) - 72;
        }
    }

}
