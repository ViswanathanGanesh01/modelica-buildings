within Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.Coupled;
package Examples
extends Modelica.Icons.ExamplesPackage;

  model Chiller "This example shows how to use the motor coupled chiller model"
    extends Modelica.Icons.Example;
    package MediumW = Buildings.Media.Water "Medium model";

    parameter Modelica.Units.SI.Power P_nominal=10E3
      "Nominal compressor power (at y=1)";
    parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal=-10
      "Temperature difference evaporator outlet-inlet";
    parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal=10
      "Temperature difference condenser outlet-inlet";
    parameter Real COP_nominal=3 "Chiller COP";
    parameter Modelica.Units.SI.MassFlowRate m2_flow_nominal=
       -P_nominal*COP_nominal/dTEva_nominal/4200
      "Nominal mass flow rate at chilled water side";
    parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal=
      m2_flow_nominal*(COP_nominal+1)/COP_nominal
      "Nominal mass flow rate at condenser water wide";

    Buildings.Fluid.Sources.MassFlowSource_T sou1(
      redeclare package Medium = Buildings.Media.Water,
      use_T_in=true,
      m_flow=m1_flow_nominal,
      T=298.15,
      nPorts=1) "Water source 1"
      annotation (Placement(transformation(extent={{-60,0},{-40,20}})));
    Buildings.Fluid.Sources.MassFlowSource_T sou2(
      redeclare package Medium = Buildings.Media.Water,
      use_T_in=true,
      m_flow=m2_flow_nominal,
      T=291.15,
      nPorts=1) "Water source 2"
      annotation (Placement(transformation(extent={{60,-40},{40,-20}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid Sou(f=50, V=230*
          1.414)
      "Voltage source"
      annotation (Placement(transformation(extent={{-8,56},{12,76}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium =
          Buildings.Media.Water,
        m_flow_nominal=m2_flow_nominal) "Temperature sensor"
      annotation (Placement(transformation(extent={{-24,-40},{-44,-20}})));
    Buildings.Fluid.Sources.Boundary_pT sin2(redeclare package Medium =
          Buildings.Media.Water, nPorts=1)
                  "Water sink 2" annotation (Placement(transformation(
        extent={{-80,-40},{-60,-20}})));
    Buildings.Fluid.Sources.Boundary_pT sin1(redeclare package Medium =
          Buildings.Media.Water, nPorts=1)
                  "Water sink 1" annotation (Placement(transformation(
        extent={{60,0},{40,20}})));

    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.Coupled.Chiller
      chi(
      redeclare package Medium1 = Buildings.Media.Water,
      redeclare package Medium2 = Buildings.Media.Water,
      dTEva_nominal=dTEva_nominal,
      dTCon_nominal=dTCon_nominal,
      P_nominal=P_nominal,
      Nrpm_nominal=1800,
      dp1_nominal=1000,
      dp2_nominal=1000,
      etaCarnot_nominal=0.5,
      pole=4,
      R_s=1,
      R_r=1.145,
      X_s=0.1457,
      X_r=0.1458,
      X_m=0.1406,
      JLoad=2,
      JMotor=2,
      have_controller=true)
      annotation (Placement(transformation(extent={{-8,-12},{12,8}})));
    Modelica.Blocks.Sources.Ramp TEva_in1(
      height=0,
      duration=600,
      offset=273.15 + 15,
      startTime=0)  "Condenser inlet temperature"
      annotation (Placement(transformation(extent={{94,-36},{74,-16}})));
    Modelica.Blocks.Sources.Step TSet1(
      height=-2,
      offset=273.15 + 5,
      startTime=500) "Condenser side leaving water temperature set point"
      annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
    Modelica.Blocks.Sources.Ramp TCon_in(
      height=0,
      duration=60,
      offset=273.15 + 20,
      startTime=0)  "Condenser inlet temperature"
      annotation (Placement(transformation(extent={{-96,4},{-76,24}})));
  equation
    connect(Sou.terminal, chi.terminal)
      annotation (Line(points={{2,56},{2,8}},         color={0,120,120}));
    connect(sou2.T_in, TEva_in1.y)
      annotation (Line(points={{62,-26},{73,-26}},       color={0,0,127}));
    connect(sou1.T_in, TCon_in.y)
      annotation (Line(points={{-62,14},{-75,14}}, color={0,0,127}));
    connect(senTem.T, chi.meaPoi)
      annotation (Line(points={{-34,-19},{-34,0.222222},{-9,0.222222}},
                                                          color={238,46,47}));
    connect(sou2.ports[1], chi.port_a2) annotation (Line(points={{40,-30},{18,
            -30},{18,-9.77778},{12,-9.77778}},
                                  color={0,127,255}));
    connect(chi.port_b1, sin1.ports[1]) annotation (Line(points={{12,3.55556},{
            32,3.55556},{32,10},{40,10}},
                              color={0,127,255}));
    connect(sou1.ports[1], chi.port_a1) annotation (Line(points={{-40,10},{-36,
            10},{-36,3.55556},{-8,3.55556}},
                                     color={0,127,255}));
    connect(chi.port_b2, senTem.port_a) annotation (Line(points={{-8,-9.77778},
            {-18,-9.77778},{-18,-30},{-24,-30}},
                                      color={0,127,255}));
    connect(senTem.port_b, sin2.ports[1])
      annotation (Line(points={{-44,-30},{-60,-30}}, color={0,127,255}));
    connect(TSet1.y, chi.setPoi) annotation (Line(
        points={{-39,50},{-9,50},{-9,6.88889}},
        color={0,0,127}));
    annotation (experiment(
        StopTime=600,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
  __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Electrical/AC/ThreePhasesBalanced/Loads/MotorDrive/Coupled/Examples/Chiller.mos"
          "Simulate and plot"),
      Documentation(info="<html>
<p>
Example that simulates a motor coupled chiller to track the set point signal 
as the evaporator entering temperate changes.
</p>
</html>",   revisions="<html>
<ul>
<li>May 07, 2024, by Viswanathan Ganesh and Zhanwei He:<br>Updated Implementation. </li>
<li>October 15, 2021, by Mingzhe Liu:<br>First implementation. </li>
</ul>
</html>"));
  end Chiller;

  model HeatPump
    "This example shows how to use the motor coupled heat pump model"
    extends Modelica.Icons.Example;
    package MediumW = Buildings.Media.Water "Medium model";

    parameter Modelica.Units.SI.Power P_nominal=10E3
      "Nominal compressor power (at y=1)";
    parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal=-10
      "Temperature difference evaporator outlet-inlet";
    parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal=10
      "Temperature difference condenser outlet-inlet";
    parameter Real COP_nominal=3 "Chiller COP";
    parameter Modelica.Units.SI.MassFlowRate m2_flow_nominal=
       -P_nominal*COP_nominal/dTEva_nominal/4200
      "Nominal mass flow rate at chilled water side";
    parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal=
      m2_flow_nominal*(COP_nominal+1)/COP_nominal
      "Nominal mass flow rate at condenser water wide";

    Modelica.Blocks.Sources.Ramp TCon_in(
      height=0,
      duration=60,
      offset=273.15 + 20,
      startTime=60) "Condenser inlet temperature"
      annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
    Buildings.Fluid.Sources.MassFlowSource_T sou2(
      redeclare package Medium = Buildings.Media.Water,
      use_T_in=true,
      m_flow=m2_flow_nominal,
      T=291.15,
      nPorts=1) "Water source 2"
      annotation (Placement(transformation(extent={{60,-60},{40,-40}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid Sou(f=50, V=230)
      "Voltage source"
      annotation (Placement(transformation(extent={{-10,16},{10,36}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium =
          Buildings.Media.Water,
      m_flow_nominal=m2_flow_nominal) "Temperature sensor"
      annotation (Placement(transformation(extent={{34,-14},{54,6}})));
    Modelica.Blocks.Sources.Step TSet(
      height=5,
      offset=273.15 + 20,
      startTime=500) "Condenser side leaving water temperature set point"
      annotation (Placement(transformation(extent={{-62,22},{-42,42}})));
    Modelica.Blocks.Sources.Ramp TEva_in(
      height=0,
      duration=60,
      offset=273.15 + 15,
      startTime=60) "Condenser inlet temperature"
      annotation (Placement(transformation(extent={{96,-56},{76,-36}})));
    Buildings.Fluid.Sources.Boundary_pT sin2(redeclare package Medium =
          Buildings.Media.Water, nPorts=1)
                "Water sink 2"
      annotation (Placement(transformation(extent={{-60,-60},{-40,-40}})));
    Buildings.Fluid.Sources.Boundary_pT sin1(redeclare package Medium =
          Buildings.Media.Water,
      nPorts=1) "Water sink 1"
      annotation (Placement(transformation(extent={{90,-14},{70,6}})));

    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.Coupled.HeatPump
      hea(
      redeclare package Medium1 = Buildings.Media.Water,
      redeclare package Medium2 = Buildings.Media.Water,
      dTEva_nominal=dTEva_nominal,
      dTCon_nominal=dTCon_nominal,
      P_nominal=P_nominal,
      Nrpm_nominal=1800,
      dp1_nominal=1000,
      dp2_nominal=1000,
      etaCarnot_nominal=0.5,
      pole=4,
      R_s=1,
      R_r=1.145,
      X_s=0.1457,
      X_r=0.1458,
      X_m=0.1406,
      JLoad=2,
      JMotor=2,
      have_controller=true)
      annotation (Placement(transformation(extent={{-10,-20},{10,0}})));
    Fluid.Sources.MassFlowSource_T           sou1(
      redeclare package Medium = MediumW,
      use_T_in=true,
      m_flow=m1_flow_nominal,
      T=298.15,
      nPorts=1) "Water source 1"
      annotation (Placement(transformation(extent={{-50,-14},{-30,6}})));
  equation
    connect(sou2.T_in, TEva_in.y) annotation (Line(points={{62,-46},{75,-46}},
                                color={0,0,127}));
    connect(senTem.port_b, sin1.ports[1]) annotation (Line(points={{54,-4},{70,-4}},
                              color={0,127,255}));
    connect(hea.port_b2, sin2.ports[1]) annotation (Line(points={{-10,-17.7778},
            {-34,-17.7778},{-34,-50},{-40,-50}},
                                  color={0,127,255}));
    connect(TSet.y, hea.setPoi) annotation (Line(points={{-41,32},{-20,32},{-20,
            -1.11111},{-11,-1.11111}},
                             color={0,140,72}));
    connect(senTem.T, hea.meaPoi) annotation (Line(points={{44,7},{44,40},{-22,
            40},{-22,-7.77778},{-11,-7.77778}},
                            color={238,46,47}));
    connect(hea.port_a2, sou2.ports[1]) annotation (Line(points={{10,-17.7778},
            {34,-17.7778},{34,-50},{40,-50}},
                                color={0,127,255}));
    connect(hea.port_b1, senTem.port_a) annotation (Line(points={{10,-4.44444},
            {22,-4.44444},{22,-4},{34,-4}},
                             color={0,127,255}));
    connect(Sou.terminal, hea.terminal)
      annotation (Line(points={{0,16},{0,8.88178e-16}},
                                                color={0,120,120}));
    connect(sou1.T_in, TCon_in.y)
      annotation (Line(points={{-52,0},{-73,0}}, color={0,0,127}));
    connect(sou1.ports[1], hea.port_a1)
      annotation (Line(points={{-30,-4},{-20,-4},{-20,-4.44444},{-10,-4.44444}},
                                                   color={0,127,255}));
    annotation (experiment(
        StartTime=500,
        StopTime=510,
        Interval=0.001,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
  __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Electrical/AC/ThreePhasesBalanced/Loads/MotorDrive/Coupled/Examples/HeatPump.mos"
          "Simulate and plot"),
      Documentation(info="<html>
<p>
Example that simulates a motor coupled heat pump to track the set point signal 
as the condenser entering temperate changes. 
</p>
</html>",   revisions="<html>
<ul>
<li>May 07, 2024, by Viswanathan Ganesh and Zhanwei He:<br>Updated Implementation. </li>
<li>October 15, 2021, by Mingzhe Liu:<br>First implementation. </li>
</ul>
</html>"));
  end HeatPump;

  model HeatPump_OnOff
    "This example shows how to use the motor coupled heat pump model"
    extends Modelica.Icons.Example;
    package MediumW = Buildings.Media.Water "Medium model";

    parameter Modelica.Units.SI.Power P_nominal=10E3
      "Nominal compressor power (at y=1)";
    parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal=-10
      "Temperature difference evaporator outlet-inlet";
    parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal=10
      "Temperature difference condenser outlet-inlet";
    parameter Real COP_nominal=3 "Chiller COP";
    parameter Modelica.Units.SI.MassFlowRate m2_flow_nominal=
       -P_nominal*COP_nominal/dTEva_nominal/4200
      "Nominal mass flow rate at chilled water side";
    parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal=
      m2_flow_nominal*(COP_nominal+1)/COP_nominal
      "Nominal mass flow rate at condenser water wide";

    Modelica.Blocks.Sources.Ramp TCon_in(
      height=0,
      duration=60,
      offset=273.15 + 20,
      startTime=60) "Condenser inlet temperature"
      annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
    Buildings.Fluid.Sources.MassFlowSource_T sou2(
      redeclare package Medium = Buildings.Media.Water,
      use_T_in=true,
      m_flow=m2_flow_nominal,
      T=291.15,
      nPorts=1) "Water source 2"
      annotation (Placement(transformation(extent={{60,-60},{40,-40}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid Sou(f=50, V=230*
          1.414)
      "Voltage source"
      annotation (Placement(transformation(extent={{-10,16},{10,36}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium =
          Buildings.Media.Water,
      m_flow_nominal=m2_flow_nominal) "Temperature sensor"
      annotation (Placement(transformation(extent={{34,-14},{54,6}})));
    Modelica.Blocks.Sources.Step TSet(
      height=5,
      offset=273.15 + 20,
      startTime=500) "Condenser side leaving water temperature set point"
      annotation (Placement(transformation(extent={{-62,22},{-42,42}})));
    Modelica.Blocks.Sources.Ramp TEva_in(
      height=0,
      duration=60,
      offset=273.15 + 15,
      startTime=60) "Condenser inlet temperature"
      annotation (Placement(transformation(extent={{96,-56},{76,-36}})));
    Buildings.Fluid.Sources.Boundary_pT sin2(redeclare package Medium =
          Buildings.Media.Water, nPorts=1)
                "Water sink 2"
      annotation (Placement(transformation(extent={{-60,-60},{-40,-40}})));
    Buildings.Fluid.Sources.Boundary_pT sin1(redeclare package Medium =
          Buildings.Media.Water,
      nPorts=1) "Water sink 1"
      annotation (Placement(transformation(extent={{90,-14},{70,6}})));

    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.Coupled.HeatPump_OnOff
      hea(
      redeclare package Medium1 = Buildings.Media.Water,
      redeclare package Medium2 = Buildings.Media.Water,
      dTEva_nominal=dTEva_nominal,
      dTCon_nominal=dTCon_nominal,
      P_nominal=P_nominal,
      Nrpm_nominal=1800,
      dp1_nominal=1000,
      dp2_nominal=1000,
      etaCarnot_nominal=0.5,
      pole=4,
      R_s=1,
      R_r=1.145,
      X_s=0.1457,
      X_r=0.1458,
      X_m=0.1406,
      JLoad=2,
      JMotor=2,
      have_controller=true)
      annotation (Placement(transformation(extent={{-10,-20},{10,0}})));
    Fluid.Sources.MassFlowSource_T           sou1(
      redeclare package Medium = MediumW,
      use_T_in=true,
      m_flow=m1_flow_nominal,
      T=298.15,
      nPorts=1) "Water source 1"
      annotation (Placement(transformation(extent={{-50,-14},{-30,6}})));
    Modelica.Blocks.Sources.BooleanConstant booleanConstant(k=true)
      annotation (Placement(transformation(extent={{-94,-54},{-74,-34}})));
  equation
    connect(sou2.T_in, TEva_in.y) annotation (Line(points={{62,-46},{75,-46}},
                                color={0,0,127}));
    connect(senTem.port_b, sin1.ports[1]) annotation (Line(points={{54,-4},{70,-4}},
                              color={0,127,255}));
    connect(hea.port_b2, sin2.ports[1]) annotation (Line(points={{-10,-17.7778},
            {-34,-17.7778},{-34,-50},{-40,-50}},
                                  color={0,127,255}));
    connect(TSet.y, hea.setPoi) annotation (Line(points={{-41,32},{-20,32},{-20,
            -2.22222},{-11,-2.22222}},
                             color={0,140,72}));
    connect(senTem.T, hea.meaPoi) annotation (Line(points={{44,7},{44,40},{-22,
            40},{-22,-7.77778},{-11,-7.77778}},
                            color={238,46,47}));
    connect(hea.port_a2, sou2.ports[1]) annotation (Line(points={{10,-17.7778},
            {34,-17.7778},{34,-50},{40,-50}},
                                color={0,127,255}));
    connect(hea.port_b1, senTem.port_a) annotation (Line(points={{10,-4.44444},
            {22,-4.44444},{22,-4},{34,-4}},
                             color={0,127,255}));
    connect(Sou.terminal, hea.terminal)
      annotation (Line(points={{0,16},{0,8.88178e-16}},
                                                color={0,120,120}));
    connect(sou1.T_in, TCon_in.y)
      annotation (Line(points={{-52,0},{-73,0}}, color={0,0,127}));
    connect(sou1.ports[1], hea.port_a1)
      annotation (Line(points={{-30,-4},{-20,-4},{-20,-4.44444},{-10,-4.44444}},
                                                   color={0,127,255}));
    connect(booleanConstant.y, hea.u) annotation (Line(points={{-73,-44},{-66,
            -44},{-66,-11.2222},{-11.5,-11.2222}}, color={255,0,255}));
    annotation (experiment(
        StopTime=1000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
      __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Electrical/AC/ThreePhasesBalanced/Loads/MotorDrive/Coupled/Examples/HeatPump.mos"
          "Simulate and plot"),
      Documentation(info="<html>
<p>
Example that simulates a motor coupled heat pump to track the set point signal 
as the condenser entering temperate changes. 
</p>
</html>",   revisions="<html>
<ul>
<li>May 07, 2024, by Viswanathan Ganesh and Zhanwei He:<br>Updated Implementation. </li>
<li>October 15, 2021, by Mingzhe Liu:<br>First implementation. </li>
</ul>
</html>"));
  end HeatPump_OnOff;

  model OneRoomRadiator_MotorCoupled
    "Heat pump with scroll compressor connected to a simple room model with radiator"
    extends Modelica.Icons.Example;
    replaceable package MediumA =
        Buildings.Media.Air "Medium model for air";
    replaceable package MediumW =
        Buildings.Media.Water "Medium model for water";

    parameter Modelica.Units.SI.HeatFlowRate Q_flow_nominal=20000
      "Nominal heat flow rate of radiator";
    parameter Modelica.Units.SI.Temperature TRadSup_nominal=273.15 + 50
      "Radiator nominal supply water temperature";
    parameter Modelica.Units.SI.Temperature TRadRet_nominal=273.15 + 45
      "Radiator nominal return water temperature";
    parameter Modelica.Units.SI.MassFlowRate mHeaPum_flow_nominal=Q_flow_nominal/
        4200/5 "Heat pump nominal mass flow rate";
    parameter Modelica.Units.SI.Volume V=6*10*3 "Room volume";
    parameter Modelica.Units.SI.MassFlowRate mA_flow_nominal=V*1.2*6/3600
      "Nominal mass flow rate";
    parameter Modelica.Units.SI.HeatFlowRate QRooInt_flow=4000
      "Internal heat gains of the room";
    //------------------------------------------------------------------------------//

    Buildings.Fluid.MixingVolumes.MixingVolume vol(
      redeclare package Medium = MediumA,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      m_flow_nominal=mA_flow_nominal,
      V=V)
      annotation (Placement(transformation(extent={{60,20},{80,40}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor theCon(G=20000/40)
      "Thermal conductance with the ambient"
      annotation (Placement(transformation(extent={{20,40},{40,60}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHea
      "Prescribed heat flow"
      annotation (Placement(transformation(extent={{20,70},{40,90}})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heaCap(C=2*V*1.2*1006)
      "Heat capacity for furniture and walls"
      annotation (Placement(transformation(extent={{60,50},{80,70}})));
    Modelica.Blocks.Sources.CombiTimeTable timTab(
        extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
        smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
        table=[-6*3600, 0;
                8*3600, QRooInt_flow;
               18*3600, 0]) "Time table for internal heat gain"
      annotation (Placement(transformation(extent={{-20,70},{0,90}})));
    Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 rad(
      redeclare package Medium = MediumW,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      Q_flow_nominal=Q_flow_nominal,
      T_a_nominal=TRadSup_nominal,
      T_b_nominal=TRadRet_nominal,
      m_flow_nominal=mHeaPum_flow_nominal,
      T_start=TRadSup_nominal)     "Radiator"
      annotation (Placement(transformation(extent={{0,-10},{20,10}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort temSup(
      redeclare package Medium = MediumW,
      m_flow_nominal=mHeaPum_flow_nominal,
      T_start=TRadSup_nominal)            "Supply water temperature"
        annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-76,-20})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temRoo
      "Room temperature" annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          origin={-40,30})));

    //----------------------------------------------------------------------------//

    //----------------------------------------------------------------------------//

    Buildings.Fluid.Sensors.TemperatureTwoPort temRet(
      redeclare package Medium = MediumW,
      m_flow_nominal=mHeaPum_flow_nominal,
      T_start=TRadSup_nominal) "Return water temperature"
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={60,-20})));

    //------------------------------------------------------------------------------------//

    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
          Modelica.Utilities.Files.loadResource(
          "modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
      "Weather data reader"
      annotation (Placement(transformation(extent={{-220,40},{-200,60}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
      annotation (Placement(transformation(extent={{-160,40},{-140,60}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature TOut
      "Outside temperature"
      annotation (Placement(transformation(extent={{-20,40},{0,60}})));

    //--------------------------------------------------------------------------------------//

    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.Coupled.HeatPump_OnOff
                                                 heaPum(
      redeclare package Medium1 = MediumW,
      redeclare package Medium2 = MediumW,
      dTEva_nominal=-5,
      dTCon_nominal=5,
      P_nominal(displayUnit="W") = 11700,
      Nrpm_nominal=1800,
      dp1_nominal=2000,
      dp2_nominal=2000,
      show_T=true,
      m1_flow_nominal=mHeaPum_flow_nominal,
      m2_flow_nominal=mHeaPum_flow_nominal,
      etaCarnot_nominal=0.3,
      R_s=0.2761,
      R_r=0.1645,
      X_s=0.078331,
      X_r=0.078331,
      X_m=0.07614,
      JLoad=0.5,
      JMotor=0.4,
      mecHea(allowFlowReversal1=false, allowFlowReversal2=false))
      "Heat pump"
      annotation (Placement(transformation(extent={{22,-164},{2,-144}})));

    Modelica.Blocks.Logical.Hysteresis hysteresis(
      uLow=273.15 + 19,
      uHigh=273.15 + 21) "Hysteresis controller"
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          origin={-190,-40})));
    Modelica.Blocks.Logical.Not not2 "Negate output of hysteresis"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          origin={-160,-40})));
    Modelica.Blocks.Math.BooleanToReal booToReaPum(realTrue=2, y(start=0))
      "Pump signal" annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=180,
          origin={-122,-110})));
    Modelica.Blocks.Logical.And and1
      annotation (Placement(transformation(extent={{-20,-90},{0,-70}})));
    Modelica.Blocks.Logical.And and2
      annotation (Placement(transformation(extent={{20,-50},{40,-30}})));
    Modelica.Blocks.Logical.Hysteresis tesConHea(
      uHigh=0.25*mHeaPum_flow_nominal,
      uLow=0.20*mHeaPum_flow_nominal) "Test for flow rate of condenser pump"
      annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=180,
          origin={-50,-80})));
    Modelica.Blocks.Logical.Hysteresis tesEvaPum(
      uLow=0.20*mHeaPum_flow_nominal,
      uHigh=0.25*mHeaPum_flow_nominal) "Test for flow rate of evaporator pump"
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=-90,
          origin={-38,-110})));
    Buildings.Fluid.Sources.Boundary_pT sou(
      redeclare package Medium = MediumW,
      T=281.15,
      nPorts=1) "Fluid source on source side"
      annotation (Placement(transformation(extent={{-134,-210},{-114,-190}})));
    Buildings.Fluid.Sources.Boundary_pT sin(
      redeclare package Medium = MediumW,
      T=283.15,
      nPorts=1) "Fluid sink on source side"
      annotation (Placement(transformation(extent={{80,-210},{60,-190}})));
    Buildings.Fluid.Sources.Boundary_pT preSou(
      redeclare package Medium = MediumW,
      T=TRadSup_nominal,
      nPorts=1)
      "Source for pressure and to account for thermal expansion of water"
      annotation (Placement(transformation(extent={{90,-118},{70,-98}})));

    Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid Sou(f=60, V=240)
      "Voltage source"
      annotation (Placement(transformation(extent={{-210,-122},{-190,-102}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=273.13 + 50)
      annotation (Placement(transformation(extent={{92,-156},{72,-136}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.Coupled.Pump
      pum(
      redeclare package Medium = Buildings.Media.Water,
      addPowerToMedium=false,
      pole=4,
      R_s=1.115,
      R_r=1.083,
      X_s=0.2096,
      X_r=0.2096,
      X_m=0.2037,
      JLoad=0.5,
      JMotor=0.051)
      annotation (Placement(transformation(extent={{-70,-208},{-50,-190}})));
    Fluid.Sensors.MassFlowRate           senMasFlo(redeclare package Medium =
          Media.Water)
      "Flow rate sensor"
      annotation (Placement(transformation(extent={{-34,-210},{-14,-190}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.Coupled.Pump
      pum1(
      redeclare package Medium = Buildings.Media.Water,
      addPowerToMedium=false,
      pole=4,
      R_s=1.115,
      R_r=1.083,
      X_s=0.209674,
      X_r=0.209674,
      X_m=0.2037,
      JLoad=0.5,
      JMotor=0.051)
      annotation (Placement(transformation(extent={{-50,-156},{-70,-138}})));
    Sensors.GeneralizedSensor Pump_Sensor
      annotation (Placement(transformation(extent={{-188,-192},{-168,-172}})));
    Sensors.GeneralizedSensor HeatPump_Sensor
      annotation (Placement(transformation(extent={{-188,-146},{-168,-126}})));
    Sources.Grid                                             Sou1(f=60, V=120)
      "Voltage source"
      annotation (Placement(transformation(extent={{-230,-144},{-210,-124}})));
  equation
    connect(theCon.port_b, vol.heatPort) annotation (Line(
        points={{40,50},{50,50},{50,30},{60,30}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(preHea.port, vol.heatPort) annotation (Line(
        points={{40,80},{50,80},{50,30},{60,30}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(heaCap.port, vol.heatPort) annotation (Line(
        points={{70,50},{50,50},{50,30},{60,30}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(timTab.y[1], preHea.Q_flow) annotation (Line(
        points={{1,80},{20,80}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(temRoo.port, vol.heatPort) annotation (Line(
        points={{-30,30},{60,30}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(rad.heatPortCon, vol.heatPort) annotation (Line(
        points={{8,7.2},{8,30},{60,30}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(rad.heatPortRad, vol.heatPort) annotation (Line(
        points={{12,7.2},{12,30},{60,30}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(weaDat.weaBus, weaBus) annotation (Line(
        points={{-200,50},{-150,50}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        textString="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(weaBus.TDryBul, TOut.T) annotation (Line(
        points={{-150,50},{-22,50}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        textString="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(TOut.port, theCon.port_a) annotation (Line(
        points={{0,50},{20,50}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(temRet.port_b, heaPum.port_a1) annotation (Line(points={{60,-30},{
            60,-148.444},{22,-148.444}},
                        color={0,127,255}));
    connect(hysteresis.y, not2.u) annotation (Line(points={{-179,-40},{-172,-40}},
                        color={255,0,255}));
    connect(temRoo.T, hysteresis.u)
      annotation (Line(points={{-51,30},{-220,30},{-220,-40},{-202,-40}},
                                                              color={0,0,127}));
    connect(temRet.port_a, rad.port_b)
      annotation (Line(points={{60,-10},{60,0},{20,0}},     color={0,127,255}));
    connect(tesConHea.y, and1.u1)
      annotation (Line(points={{-39,-80},{-22,-80}}, color={255,0,255}));
    connect(tesEvaPum.y, and1.u2) annotation (Line(points={{-38,-99},{-38,-88},
            {-22,-88}},              color={255,0,255}));
    connect(not2.y, and2.u1) annotation (Line(points={{-149,-40},{18,-40}},
                   color={255,0,255}));
    connect(not2.y, booToReaPum.u) annotation (Line(points={{-149,-40},{-142,
            -40},{-142,-110},{-134,-110}},
                                      color={255,0,255}));
    connect(sin.ports[1], heaPum.port_b2) annotation (Line(points={{60,-200},{
            40,-200},{40,-161.778},{22,-161.778}},
                                  color={0,127,255}));
    connect(and1.y, and2.u2) annotation (Line(points={{1,-80},{10,-80},{10,-48},{18,
            -48}}, color={255,0,255}));
    connect(preSou.ports[1], temRet.port_b) annotation (Line(points={{70,-108},
            {60,-108},{60,-30}},
                             color={0,127,255}));
    connect(heaPum.setPoi, realExpression.y) annotation (Line(points={{23,
            -146.222},{45,-146.222},{45,-146},{71,-146}},           color={0,0,127}));
    connect(and2.y, heaPum.u) annotation (Line(points={{41,-40},{46,-40},{46,
            -155.222},{23.5,-155.222}},
                              color={255,0,255}));
    connect(temSup.T, heaPum.meaPoi) annotation (Line(points={{-87,-20},{-90,
            -20},{-90,-132},{22,-132},{22,-151.778},{23,-151.778}},
                                            color={0,0,127}));
    connect(pum.port_a, sou.ports[1])
      annotation (Line(points={{-70,-200},{-114,-200}}, color={0,127,255}));
    connect(senMasFlo.m_flow, pum.meaPoi) annotation (Line(points={{-24,-189},{
            -24,-180},{-98,-180},{-98,-196},{-71,-196}}, color={0,0,127}));
    connect(senMasFlo.port_a, pum.port_b) annotation (Line(points={{-34,-200},{
            -50,-200}},                       color={0,127,255}));
    connect(senMasFlo.port_b, heaPum.port_a2) annotation (Line(points={{-14,
            -200},{-6,-200},{-6,-162},{-4,-162},{-4,-161.778},{2,-161.778}},
                                                         color={0,127,255}));
    connect(booToReaPum.y, pum1.setPoi) annotation (Line(points={{-111,-110},{
            -56,-110},{-56,-140},{-49,-140}}, color={0,0,127}));
    connect(tesConHea.u, pum1.setPoi) annotation (Line(points={{-62,-80},{-68,
            -80},{-68,-110},{-56,-110},{-56,-140},{-49,-140}}, color={0,0,127}));
    connect(pum.setPoi, pum1.setPoi) annotation (Line(points={{-71,-192},{-94,
            -192},{-94,-110},{-56,-110},{-56,-140},{-49,-140}},
                                                     color={0,0,127}));
    connect(HeatPump_Sensor.terminal_n, Sou.terminal) annotation (Line(points={{-188,
            -136},{-200,-136},{-200,-122}},       color={0,120,120}));
    connect(HeatPump_Sensor.terminal_p, heaPum.terminal) annotation (Line(
          points={{-168,-136},{12,-136},{12,-144}},                     color={
            0,120,120}));
    connect(Pump_Sensor.terminal_p, pum.terminal) annotation (Line(points={{-168,
            -182},{-60,-182},{-60,-190}},                              color={0,
            120,120}));
    connect(Sou1.terminal, Pump_Sensor.terminal_n) annotation (Line(points={{-220,
            -144},{-220,-182},{-188,-182}},      color={0,120,120}));
    connect(pum1.port_a, heaPum.port_b1) annotation (Line(points={{-50,-148},{
            -25,-148},{-25,-148.444},{2,-148.444}}, color={0,127,255}));
    connect(tesEvaPum.u, pum.meaPoi) annotation (Line(points={{-38,-122},{-38,
            -180},{-98,-180},{-98,-196},{-71,-196}}, color={0,0,127}));
    connect(pum1.meaPoi, pum.meaPoi) annotation (Line(points={{-49,-144},{-38,
            -144},{-38,-180},{-98,-180},{-98,-196},{-71,-196}}, color={0,0,127}));
    connect(temSup.port_b, pum1.port_b) annotation (Line(points={{-76,-30},{-76,
            -148},{-70,-148}}, color={0,127,255}));
    connect(temSup.port_a, rad.port_a)
      annotation (Line(points={{-76,-10},{-76,0},{0,0}}, color={0,127,255}));
    connect(pum1.terminal, Pump_Sensor.terminal_n) annotation (Line(points={{
            -60,-138},{-140,-138},{-140,-166},{-220,-166},{-220,-182},{-188,
            -182}}, color={0,120,120}));
    annotation (Documentation(info="<html>
<p>
Example that simulates one room equipped with a radiator. Hot water is produced
by a <i>24</i> kW nominal capacity heat pump. The source side water temperature to the
heat pump is constant at <i>10</i>&deg;C.
</p>
<p>
The heat pump is turned on when the room temperature falls below
<i>19</i>&deg;C and turned
off when the room temperature rises above <i>21</i>&deg;C.
</p>
</html>",   revisions="<html>
<ul>
<li>
July 22, 2021, by Michael Wetter:<br/>
Removed assignments <code>pumHeaPum(y_start=1)</code> and <code>pumHeaPumSou(y_start=1)</code>.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1498\">#1498</a>.
</li>
<li>
April 21, 2021, by Michael Wetter:<br/>
Corrected error in calculation of design mass flow rate.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2458\">#2458</a>.
</li>
<li>
May 2, 2019, by Jianjun Hu:<br/>
Replaced fluid source. This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1072\"> #1072</a>.
</li>
<li>
March 3, 2017, by Michael Wetter:<br/>
Changed mass flow test to use a hysteresis as a threshold test
can cause chattering.
</li>
<li>
January 27, 2017, by Massimo Cimmino:<br/>
First implementation.
</li>
</ul>
</html>"),
      Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-240,-220},{100,
              100}})),
      __Dymola_Commands(file=
       "modelica://Buildings/Resources/Scripts/Dymola/Fluid/HeatPumps/Examples/ScrollWaterToWater_OneRoomRadiator.mos"
          "Simulate and plot"),
      experiment(
        StopTime=86400,
        Tolerance=1e-08,
        __Dymola_Algorithm="Dassl"));
  end OneRoomRadiator_MotorCoupled;

  model Pump "This example shows how to use the motor coupled pump model"
    extends Modelica.Icons.Example;
    package MediumW = Buildings.Media.Water;
    parameter Modelica.Units.SI.MassFlowRate m_flow_nominal = 1 "Nominal mass flow rate";
    parameter Modelica.Units.SI.Pressure dp_nominal=500   "nominal pressure drop";

    Buildings.Fluid.Sources.Boundary_pT sou(redeclare package Medium =
          Buildings.Media.Water,
      nPorts=1) "Boundary"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0,
        origin={-90,20})));

    Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid gri(f=50, V=230*
          1.414)
      "Voltage source"
      annotation (Placement(transformation(extent={{-2,42},{18,62}})));
    Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare package Medium =
          Buildings.Media.Water)
      "Flow rate sensor"
      annotation (Placement(transformation(extent={{0,-50},{-20,-30}})));
    Buildings.Fluid.FixedResistances.PressureDrop dp1(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=m_flow_nominal,
      dp_nominal=1/2*dp_nominal) "Pressure loss"
      annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
    Buildings.Fluid.FixedResistances.PressureDrop dp2(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=m_flow_nominal,
      dp_nominal=1/2*dp_nominal) "Pressure loss"
      annotation (Placement(transformation(extent={{40,10},{60,30}})));

    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.MotorDrive.Coupled.Pump
      pum1(
      redeclare package Medium = Buildings.Media.Water,
      redeclare
        Buildings.Fluid.Movers.Data.Pumps.Wilo.VeroLine50slash150dash4slash2
        per,
      R_s=1,
      R_r=1.145,
      X_s=0.1457,
      X_r=0.1458,
      X_m=0.1406,
      JLoad=2,
      JMotor=2,
      have_controller=true,
      pum(pum(energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)))
      annotation (Placement(transformation(extent={{-2,10},{18,30}})));

    Modelica.Blocks.Sources.Step Set_Point(height=10, startTime=500)
      annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
  equation
    connect(dp1.port_a, senMasFlo.port_b) annotation (Line(points={{-40,20},{-60,20},
            {-60,-40},{-20,-40}}, color={0,127,255}));
    connect(dp2.port_b, senMasFlo.port_a) annotation (Line(points={{60,20},{80,20},
            {80,-40},{0,-40}}, color={0,127,255}));
    connect(sou.ports[1], dp1.port_a) annotation (Line(points={{-80,20},{-40,20}},
            color={0,127,255}));
    connect(pum1.port_a, dp1.port_b)
      annotation (Line(points={{-2,18.8889},{-12,18.8889},{-12,20},{-20,20}},
                                                  color={0,127,255}));
    connect(pum1.port_b, dp2.port_a)
      annotation (Line(points={{18,18.8889},{30,18.8889},{30,20},{40,20}},
                                                 color={0,127,255}));
    connect(senMasFlo.m_flow, pum1.meaPoi)
      annotation (Line(points={{-10,-29},{-10,23.3333},{-3,23.3333}},
                                                            color={238,46,47}));

    connect(gri.terminal, pum1.terminal)
      annotation (Line(points={{8,42},{8,30}},         color={0,120,120}));
    connect(Set_Point.y, pum1.setPoi) annotation (Line(
        points={{-39,50},{-26,50},{-26,27.7778},{-3,27.7778}},
        color={0,140,72}));
    annotation (experiment(
        StopTime=1000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
  __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Electrical/AC/ThreePhasesBalanced/Loads/MotorDrive/Coupled/Examples/Pump.mos"
          "Simulate and plot"),
      Documentation(info="<html>
<p>
Example that simulates a motor coupled pump to track the set point signal as 
the load changes.
</p>
</html>",   revisions="<html>
<ul>
<li>May 07, 2024, by Viswanathan Ganesh and Zhanwei He:<br>Updated Implementation. </li>
<li>October 15, 2021, by Mingzhe Liu:<br>First implementation. </li>
</ul>
</html>"));
  end Pump;
end Examples;
