package org.firstinspires.ftc.teamcode.Utilities;

import java.util.ArrayList;
import java.util.Scanner;
import java.io.File;
import java.io.FileNotFoundException;

public class CSVReader {

    public CSVReader(){}

    public ArrayList xValues = new ArrayList(1);
    public ArrayList yValues = new ArrayList(1);
    public ArrayList zValues = new ArrayList(1);

    String fileName = "AimTable.csv";
    private File readFile = new File(fileName);



}
