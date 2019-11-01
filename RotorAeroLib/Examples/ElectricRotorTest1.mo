within RotorAeroLib.Examples;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model ElectricRotorTest1
  parameter Integer N = 3 annotation(
    Evaluate = true);
  parameter Real RPM1 = 720;
  parameter Real RPM2 = 720;
  parameter Real RPM3 = 720;
  parameter Real RPM4 = 720;
  //
  inner Modelica.Mechanics.MultiBody.World world(enableAnimation = true, g = 0.0, n = {-1, 0, 0}, nominalLength = 0.25) annotation(
    Placement(visible = true, transformation(origin = {-98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner DeployStructLib.DSL_Globals DSLglb(enableAnimation = true) annotation(
    Placement(visible = true, transformation(origin = {-102, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Examples.ElectricRotor rotor1(N = N) annotation(
    Placement(visible = true, transformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp voltage_setpoint(duration = 20, height = 100, offset = 1, startTime = 2) annotation(
    Placement(visible = true, transformation(origin = {-92, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm1(r = {0, 1, 0}) annotation(
    Placement(visible = true, transformation(origin = {16, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
initial equation
  rotor1.drive.w = Modelica.Constants.pi/2;
equation
  connect(arm1.frame_a, world.frame_b) annotation(
    Line(points = {{6, 70}, {-52, 70}, {-52, 0}, {-88, 0}, {-88, 0}}, color = {95, 95, 95}));
  connect(voltage_setpoint.y, rotor1.Voltage) annotation(
    Line(points = {{-80, 84}, {86, 84}, {86, 78}, {88, 78}}, color = {0, 0, 127}));
  connect(arm1.frame_b, rotor1.frame_a) annotation(
    Line(points = {{26, 70}, {90, 70}, {90, 70}, {90, 70}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 30, Tolerance = 1e-06, Interval = 0.02));

end ElectricRotorTest1;