package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController
{
    private double Kp; // Proportional gain
    private double Ki; // Integral gain
    private double Kd; // Derivative gain
    private double Kf;

    private double setpoint;
    private double previousError;
    private double integralSum;

    private double tolerance = 0;

    private static double minOutput; // Minimum output limit
    private static double maxOutput; // Maximum output limit

    private ElapsedTime Timer = new ElapsedTime();

    /**
     * Constructor for the PIDController.
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param minOutput Minimum output value
     * @param maxOutput Maximum output value
     */
    public PIDFController(double Kp, double Ki, double Kd, double Kf, double minOutput, double maxOutput)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.previousError = 0;
        this.integralSum = 0;
    }

    /**
     * Sets the desired setpoint for the controller.
     * @param setpoint The target value
     */
    public void setTarget(double setpoint)
    {
        this.setpoint = setpoint;
    }

    /**
     * Calculates the control output based on the current process variable.
     * @param processVariable The current measured value
     * @return The calculated control output
     */
    public double calculate(double processVariable)
    {
        if (processVariable > setpoint - tolerance && processVariable < setpoint + tolerance)
        {
            Timer.reset();
            return 0;
        }
        double deltaTime = Timer.seconds();
        double error = setpoint - processVariable;

        // Proportional term
        double proportionalTerm = Kp * error;

        // Integral term
        integralSum += error * deltaTime;
        double integralTerm = Ki * integralSum;

        // Derivative term
        double derivativeTerm = Kd * (error - previousError) / deltaTime;

        // FeedForward Term
        double feedforwardTerm = Kf * setpoint;

        // Calculate total output
        double output = proportionalTerm + integralTerm + derivativeTerm + feedforwardTerm;

        // Clamp output to defined limits
        if (output > maxOutput)
        {
            output = maxOutput;
        }
        else if (output < minOutput)
        {
            output = minOutput;
        }

        // Store current error for next iteration
        previousError = error;

        Timer.reset();
        return output;
    }

    public void setTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }

    public void setPIDFCoefficients(double p, double i, double d, double f)
    {
        Kp = p;
        Ki = i;
        Kd = d;
        Kf = f;
    }

    /**
     * Resets the integral sum and previous error, useful when the system state changes significantly.
     */
    public void reset()
    {
        integralSum = 0;
        previousError = 0;
    }

    // You can add methods to get/set Kp, Ki, Kd, etc. if needed.
}
