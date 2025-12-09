package org.firstinspires.ftc.teamcode.Utilities;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import java.util.ArrayList;

public class LinearRegression {

    /**

     WORK IN PROGRESS

     **/
    public LinearRegression(){}




    /**
     * lineOfBestFit
     * @param x
     * @param y
     * @return {m, b} in terms of y = mx + b
     */
    public double[] lineOfBestFit(double[] x, double[] y)
    {
        if (x.length != y.length)
        {
            throw new IllegalArgumentException("Arrays must have the same length to perform linear regression.");
        }

        int n = x.length;
        double sumX = 0;
        double sumY = 0;
        double sumXY = 0;
        double sumX2 = 0;

        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }

        double m = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        double b = (sumY - m * sumX) / n;

        return new double[]{m, b};
    }

}
