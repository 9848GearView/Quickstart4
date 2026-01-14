///*
// * Copyright (c) 2020 OpenFTC Team
// *
// * Permission is hereby granted, free of charge, to any person obtaining a copy
// * of this software and associated documentation files (the "Software"), to deal
// * in the Software without restriction, including without limitation the rights
// * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// * copies of the Software, and to permit persons to whom the Software is
// * furnished to do so, subject to the following conditions:
// *
// * The above copyright notice and this permission notice shall be included in all
// * copies or substantial portions of the Software.
// * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// * SOFTWARE.
// */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.mech.ColorSensor;
import org.firstinspires.ftc.teamcode.mech.Intake;
import org.firstinspires.ftc.teamcode.mech.MecanumDrive;

@Autonomous(name = "Big Tri")

public class BtAuto extends LinearOpMode {
    MecanumDrive chassis = null;
    Intake cannon = null;
    ColorSensor colSens = null;
    ElapsedTime runtime = null;


    @Override
    public void runOpMode() throws InterruptedException {


        chassis = new MecanumDrive(hardwareMap);
        cannon = new Intake(hardwareMap);
        colSens = new ColorSensor(hardwareMap);
        runtime = new ElapsedTime();

        waitForStart();
            runtime.reset();

            //moveout of the funny box
            while (opModeIsActive() &&  runtime.milliseconds() < 750) {
                chassis.drive(0.2,0,0);
            }

        chassis.drive(0.2,0,0);

    }//closes method
} //closes class
