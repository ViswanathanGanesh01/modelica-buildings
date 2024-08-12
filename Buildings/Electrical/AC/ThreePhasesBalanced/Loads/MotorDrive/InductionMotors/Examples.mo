within Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.InductionMotors;
package Examples
  "Collection of models that illustrate model use and test models"
  extends Modelica.Icons.ExamplesPackage;

  model SquirrelCage "This example shows how to use the squirrel cage induction motor"

    extends Modelica.Icons.Example;
    parameter Modelica.Units.SI.Resistance R_s=0.641 "Electric resistance of stator";
    parameter Modelica.Units.SI.Resistance R_r=0.332 "Electric resistance of rotor";
    parameter Modelica.Units.SI.Reactance X_s=1.106 "Complex component of the impedance of stator";
    parameter Modelica.Units.SI.Reactance X_r=0.464 "Complex component of the impedance of rotor";
    parameter Modelica.Units.SI.Reactance X_m=26.3 "Complex component of the magnetizing reactance";
    parameter Real Tl= 0;
    Real Reactive_pow;
    Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid sou(f=50, V=400)
      "Voltage source"
      annotation (Placement(transformation(extent={{-10,40},{10,60}})));
    Modelica.Blocks.Sources.RealExpression tau_m(y=26.5)
      "Load torque"
      annotation (Placement(transformation(extent={{-80,-18},{-60,2}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.InductionMotors.SquirrelCage simMot(
      J=0.17,
      R_s=1,
      R_r=1.145,
      X_s=0.1455,
      X_r=0.1458,
      X_m=0.1406,
      PF=0.85) "Motor"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

  equation
   Reactive_pow = sqrt(sou.P.apparent^2-sou.P.real^2)/1000;
    connect(sou.terminal, simMot.terminal) annotation (Line(points={{0,40},{0,
            10}},
            color={0,120,120}));
    connect(tau_m.y, simMot.tau_m) annotation (Line(points={{-59,-8},{-12,-8}},
            color={0,0,127}));
    annotation (experiment(
        StopTime=1800,
        Interval=0.002,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
  __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Electrical/AC/ThreePhasesBalanced/Loads/MotorDrive/InductionMotors/Examples/SquirrelCage.mos"
          "Simulate and plot"),
      Documentation(info="<html>
<p>
Example that simulates a induction motor using the balanced three-phase power 
supply and quadratic type load signal.
</p>
</html>",   revisions="<html>
<ul>
<li>
October 15, 2021, by Mingzhe Liu:<br/>
First implementation. 
</li>
</ul>
</html>"));
  end SquirrelCage;

  model SquirrelCageDrive "This example shows how to use the squirrel cage induction motor with built-in speed control"

    extends Modelica.Icons.Example;
    parameter Modelica.Units.SI.Resistance R_s=0.641 "Electric resistance of stator";
    parameter Modelica.Units.SI.Resistance R_r=0.332 "Electric resistance of rotor";
    parameter Modelica.Units.SI.Reactance X_s=1.106 "Complex component of the impedance of stator";
    parameter Modelica.Units.SI.Reactance X_r=0.464 "Complex component of the impedance of rotor";
    parameter Modelica.Units.SI.Reactance X_m=26.3 "Complex component of the magnetizing reactance";

    Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid sou(f=50, V=400)
      "Voltage source"
      annotation (Placement(transformation(extent={{-10,40},{10,60}})));
    Modelica.Blocks.Sources.RealExpression mea(y=simMot.omega_r)
      "Measured value of control target"
      annotation (Placement(transformation(extent={{-80,10},{-60,30}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.InductionMotors.SquirrelCageDrive simMot(
      J=0.0131,
      R_s=1.405,
      R_r=1.395,
      X_s=0.178,
      X_r=0.178,
      X_m=0.1722)
               "Motor with speed control"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

    Modelica.Blocks.Sources.Step temSet(
      height=750/9.55,
      offset=0,
      startTime=1)   "Set point of control target"
      annotation (Placement(transformation(extent={{-78,40},{-58,60}})));
    Modelica.Blocks.Sources.Ramp ramp(
      height=750/9.55,
      duration=0.4,
      startTime=0.1)
      annotation (Placement(transformation(extent={{-78,74},{-58,94}})));
    Modelica.Blocks.Math.Add add
      annotation (Placement(transformation(extent={{-44,56},{-24,76}})));
    Modelica.Blocks.Sources.Step tau_m(
      height=26.5,
      offset=0,
      startTime=0.4) "Set point of control target"
      annotation (Placement(transformation(extent={{-70,-42},{-50,-22}})));
  equation
    connect(sou.terminal, simMot.terminal) annotation (Line(points={{0,40},{0,10}},
            color={0,120,120}));
    connect(mea.y, simMot.mea) annotation (Line(points={{-59,20},{-40,20},{-40,4},
            {-12,4}}, color={0,0,127}));
    connect(add.u1,ramp. y) annotation (Line(points={{-46,72},{-54,72},{-54,78},
            {-52,78},{-52,84},{-57,84}},     color={0,0,127}));
    connect(add.u2,temSet. y) annotation (Line(points={{-46,60},{-52,60},{-52,
            50},{-57,50}},     color={0,0,127}));
    connect(add.y, simMot.setPoi) annotation (Line(points={{-23,66},{-16,66},{
            -16,14},{-18,14},{-18,8},{-12,8}}, color={0,0,127}));
    connect(tau_m.y, simMot.tau_m) annotation (Line(points={{-49,-32},{-20,-32},
            {-20,-8},{-12,-8}}, color={0,0,127}));
    annotation (experiment(
        StopTime=2,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
  __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Electrical/AC/ThreePhasesBalanced/Loads/MotorDrive/InductionMotors/Examples/SquirrelCageDrive.mos"
          "Simulate and plot"),
      Documentation(info="<html>
<p>
Example that simulates an induction motor with variable speed control to track 
a step signal. 
</p>
</html>",   revisions="<html>
<ul>
<li>
October 15, 2021, by Mingzhe Liu:<br/>
First implementation.
</li>
</ul>
</html>"));
  end SquirrelCageDrive;

  model SquirrelCageDriveValidation "This model validates the squirrel cage induction motor with built-in speed control"

    extends Modelica.Icons.Example;
    parameter Modelica.Units.SI.Resistance R_s=0.641 "Electric resistance of stator";
    parameter Modelica.Units.SI.Resistance R_r=0.332 "Electric resistance of rotor";
    parameter Modelica.Units.SI.Reactance X_s=1.106 "Complex component of the impedance of stator";
    parameter Modelica.Units.SI.Reactance X_r=0.464 "Complex component of the impedance of rotor";
    parameter Modelica.Units.SI.Reactance X_m=26.3 "Complex component of the magnetizing reactance";

    Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid sou(f=50, V=380)
      "Voltage source"
      annotation (Placement(transformation(extent={{-10,40},{10,60}})));
    Modelica.Blocks.Sources.RealExpression tau_m(y=0)
      "Load torque"
      annotation (Placement(transformation(extent={{-58,-32},{-38,-12}})));
    Modelica.Blocks.Sources.RealExpression mea(y=simMot.omega_r)
      "Measured value of control target"
      annotation (Placement(transformation(extent={{-62,-2},{-42,18}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.InductionMotors.SquirrelCageDrive simMot(
      pole=4,
      J=0.046,
      R_s=1.7,
      R_r=2.68,
      X_s=0.008,
      X_r=0.008,
      X_m=0.217)
               "Motor with speed control"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

    Modelica.Blocks.Sources.Step Speed_ref(
      height=90,
      offset=10,
      startTime=1.7) "Set point of control target"
      annotation (Placement(transformation(extent={{-102,60},{-82,80}})));
    Modelica.Blocks.Sources.Trapezoid trapezoid(
      amplitude=200,
      rising=6,
      width=4,
      falling=6,
      period=25,
      offset=-100,
      startTime=4)
      annotation (Placement(transformation(extent={{-102,22},{-82,42}})));
    Modelica.Blocks.Sources.Pulse pulse(
      amplitude=60,
      width=50,
      period=2,
      offset=-30,
      startTime=1.4)
      annotation (Placement(transformation(extent={{-102,-22},{-82,-2}})));
  equation
    connect(sou.terminal, simMot.terminal) annotation (Line(points={{0,40},{0,10}},
            color={0,120,120}));
    connect(tau_m.y, simMot.tau_m) annotation (Line(points={{-37,-22},{-20,-22},
            {-20,-8},{-12,-8}},
            color={0,0,127}));
    connect(mea.y, simMot.mea) annotation (Line(points={{-41,8},{-20,8},{-20,4},
            {-12,4}}, color={0,0,127}));
    connect(pulse.y, simMot.setPoi) annotation (Line(points={{-81,-12},{-66,-12},
            {-66,22},{-12,22},{-12,8}}, color={0,0,127}));
    annotation (experiment(
        StopTime=3600,
        Interval=0.002,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
  __Dymola_Commands(file=
            "modelica://Buildings/Resources/Scripts/Dymola/Electrical/AC/ThreePhasesBalanced/Loads/MotorDrive/InductionMotors/Examples/SquirrelCageDriveValidation.mos"
          "Simulate and plot"),
      Documentation(info="<html>
<p>
Example that simulates an induction motor with variable speed control to track 
a step signal. 
</p>
</html>",   revisions="<html>
<ul>
<li>
September 15, 2023, by Viswanathan Ganesh:<br/>
First implementation.
</li>
</ul>
</html>"));
  end SquirrelCageDriveValidation;

  model SquirrelCageDriveValidationFullDay
    "This model validates the squirrel cage induction motor with built-in speed control"

    extends Modelica.Icons.Example;
    parameter Modelica.Units.SI.Resistance R_s=0.641 "Electric resistance of stator";
    parameter Modelica.Units.SI.Resistance R_r=0.332 "Electric resistance of rotor";
    parameter Modelica.Units.SI.Reactance X_s=1.106 "Complex component of the impedance of stator";
    parameter Modelica.Units.SI.Reactance X_r=0.464 "Complex component of the impedance of rotor";
    parameter Modelica.Units.SI.Reactance X_m=26.3 "Complex component of the magnetizing reactance";

    Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid sou(f=50, V=400)
      "Voltage source"
      annotation (Placement(transformation(extent={{-10,40},{10,60}})));
    Modelica.Blocks.Sources.RealExpression mea(y=simMot.omega_r)
      "Measured value of control target"
      annotation (Placement(transformation(extent={{-46,-6},{-26,14}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.InductionMotors.SquirrelCageDrive simMot(
      pole=4,
      J=0.17,
      R_s=1,
      R_r=1.145,
      X_s=0.0051,
      X_r=0.0051,
      X_m=0.1406,
      N=1500,
      PF=0.9)  "Motor with speed control"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

    Modelica.Blocks.Sources.CombiTimeTable case3(table=[0,878,11; 1,1328,5; 2,
          1431,5; 3,1432,5; 4,1193,12; 5,1169,6; 6,1203,6; 7,726,9; 8,1212,6; 9,
          991,10; 10,756,11; 11,1233,10; 12,964,9; 13,1098,7; 14,775,11; 15,
          1003,8; 16,1074,10; 17,1096,5; 18,856,11; 19,1085,7; 20,1110,6; 21,
          1231,9; 22,755,12; 23,825,6; 24,878,11], timeScale=3600)
      annotation (Placement(transformation(extent={{-86,-58},{-66,-38}})));
    Modelica.Blocks.Sources.CombiTimeTable case2(table=[0,878,5; 1,1328,5; 2,
          1431,5; 3,1432,5; 4,1193,5; 5,1169,5; 6,1203,5; 7,726,5; 8,1212,5; 9,
          991,5; 10,756,5; 11,1233,5; 12,964,5; 13,1098,5; 14,775,5; 15,1003,5;
          16,1074,5; 17,1096,5; 18,856,5; 19,1085,5; 20,1110,5; 21,1231,5; 22,
          755,5; 23,825,5; 24,878,5], timeScale=3600)
      annotation (Placement(transformation(extent={{-86,-18},{-66,2}})));
    Modelica.Blocks.Sources.CombiTimeTable case1(table=[0,1440,11; 1,1440,5; 2,
          1440,5; 3,1440,5; 4,1440,12; 5,1440,6; 6,1440,6; 7,1440,9; 8,1440,6;
          9,1440,10; 10,1440,11; 11,1440,10; 12,1440,9; 13,1440,7; 14,1440,11;
          15,1440,8; 16,1440,10; 17,1440,5; 18,1440,11; 19,1440,7; 20,1440,6;
          21,1440,9; 22,1440,12; 23,1440,6; 24,1440,11], timeScale=3600)
      annotation (Placement(transformation(extent={{-86,22},{-66,42}})));
    Modelica.Blocks.Math.Gain rpm_rad(k=1/9.55)
      annotation (Placement(transformation(extent={{-36,24},{-26,34}})));
    Modelica.Blocks.Math.Gain rpm_rad1(k=3)
      annotation (Placement(transformation(extent={{-34,-14},{-24,-4}})));
  equation
    connect(sou.terminal, simMot.terminal) annotation (Line(points={{0,40},{0,10}},
            color={0,120,120}));
    connect(mea.y, simMot.mea) annotation (Line(points={{-25,4},{-12,4}},
                      color={0,0,127}));
    connect(rpm_rad.y, simMot.setPoi) annotation (Line(points={{-25.5,29},{-20,
            29},{-20,8},{-12,8}}, color={0,0,127}));
    connect(rpm_rad1.y, simMot.tau_m) annotation (Line(points={{-23.5,-9},{
            -17.75,-9},{-17.75,-8},{-12,-8}}, color={0,0,127}));
    connect(case3.y[1], rpm_rad.u) annotation (Line(points={{-65,-48},{-52,-48},
            {-52,29},{-37,29}}, color={0,0,127}));
    connect(rpm_rad1.u, case3.y[2]) annotation (Line(points={{-35,-9},{-35,-48},
            {-65,-48}}, color={0,0,127}));
    annotation (experiment(
        StopTime=86400,
        Interval=1,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
  __Dymola_Commands(file=
            "modelica://Buildings/Resources/Scripts/Dymola/Electrical/AC/ThreePhasesBalanced/Loads/MotorDrive/InductionMotors/Examples/SquirrelCageDriveValidation.mos"
          "Simulate and plot"),
      Documentation(info="<html>
<p>
Example that simulates an induction motor with variable speed control to track 
a step signal. 
</p>
</html>",   revisions="<html>
<ul>
<li>
September 15, 2023, by Viswanathan Ganesh:<br/>
First implementation.
</li>
</ul>
</html>"));
  end SquirrelCageDriveValidationFullDay;
annotation (Documentation(info="<html>
<p>
This package contains examples for the use of models that can be found in 
<a href=\"modelica://Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.InductionMotors\">
Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.InductionMotors</a>. 
</p>
</html>"));
end Examples;
