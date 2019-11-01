within RotorAeroLib.Examples.QuadCopter;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model QuadCopter3
  parameter Integer N = 3 annotation(
    Evaluate = true);
  parameter Real Voltage1 = 459;
  parameter Real Voltage2 = 459;
  parameter Real Voltage3 = 459;
  parameter Real Voltage4 = 459;
  //
  inner Modelica.Mechanics.MultiBody.World world(enableAnimation = true, n = {-1, 0, 0}, nominalLength = 0.25) annotation(
    Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner DeployStructLib.DSL_Globals DSLglb(enableAnimation = true) annotation(
    Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Examples.ElectricRotor rotor1(N = N) annotation(
    Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Examples.ElectricRotor rotor2(N = N, orientation_ccw = false) annotation(
    Placement(visible = true, transformation(origin = {70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Examples.ElectricRotor rotor3(N = N) annotation(
    Placement(visible = true, transformation(origin = {70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Examples.ElectricRotor rotor4(N = N, orientation_ccw = false) annotation(
    Placement(visible = true, transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Step altitude_setpoint(height = 5.0, startTime = 2) annotation(
    Placement(visible = true, transformation(origin = {-86, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body bodyCM(I_11 = 1, I_22 = 1, I_33 = 1, angles_fixed = true, enforceStates = true, m = 372.8, r_0(each fixed = true), v_0(each fixed = true), w_0_fixed = true) annotation(
    Placement(visible = true, transformation(origin = {-30, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm1(r = {0, 1, 0}) annotation(
    Placement(visible = true, transformation(origin = {16, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm2(r = {0, 0, 1}) annotation(
    Placement(visible = true, transformation(origin = {18, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm3(r = {0, -1, 0}) annotation(
    Placement(visible = true, transformation(origin = {20, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm4(r = {0, 0, -1}) annotation(
    Placement(visible = true, transformation(origin = {18, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Examples.QuadCopter.AltitudeController altitudeController1(kd = 100, kp = 150, max_limit = 700, min_limit = 50, y_start = Voltage1) annotation(
    Placement(visible = true, transformation(origin = {-36, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteSensor absoluteSensor1(animation = false, get_r = true, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(visible = true, transformation(origin = {-62, -30}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  Modelica.Blocks.Routing.DeMultiplex3 deMultiplex31 annotation(
    Placement(visible = true, transformation(origin = {-64, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Math.Gain gain1(k = -1) annotation(
    Placement(visible = true, transformation(origin = {40, 38}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = -1) annotation(
    Placement(visible = true, transformation(origin = {40, -62}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
initial equation
  bodyCM.a_0[1] = 0;
  rotor2.drive.w = -rotor1.drive.w;
  rotor3.drive.w = rotor1.drive.w;
  rotor4.drive.w = -rotor1.drive.w;
equation
  connect(altitude_setpoint.y, altitudeController1.h_setpoint) annotation(
    Line(points = {{-75, 86}, {-62, 86}, {-62, 92}, {-48, 92}}, color = {0, 0, 127}));
  connect(altitudeController1.y, rotor1.Voltage) annotation(
    Line(points = {{-25, 84}, {58, 84}, {58, 78}}, color = {0, 0, 127}));
  connect(altitudeController1.y, rotor3.Voltage) annotation(
    Line(points = {{-25, 84}, {92, 84}, {92, -10}, {58, -10}, {58, -22}}, color = {0, 0, 127}));
  connect(altitudeController1.u, deMultiplex31.y1[1]) annotation(
    Line(points = {{-48, 84}, {-72, 84}, {-72, 50}}, color = {0, 0, 127}));
  connect(altitudeController1.y, gain1.u) annotation(
    Line(points = {{-25, 84}, {92, 84}, {92, 52}, {33, 52}, {33, 38}}, color = {0, 0, 127}));
  connect(altitudeController1.y, gain2.u) annotation(
    Line(points = {{-25, 84}, {92, 84}, {92, -48}, {33, -48}, {33, -62}}, color = {0, 0, 127}));
  connect(arm1.frame_b, rotor1.frame_a) annotation(
    Line(points = {{26, 70}, {60, 70}}, color = {95, 95, 95}));
  connect(arm4.frame_b, rotor4.frame_a) annotation(
    Line(points = {{28, -70}, {60, -70}}, color = {95, 95, 95}));
  connect(gain2.y, rotor4.Voltage) annotation(
    Line(points = {{47, -62}, {58, -62}}, color = {0, 0, 127}));
  connect(arm3.frame_b, rotor3.frame_a) annotation(
    Line(points = {{30, -30}, {60, -30}}, color = {95, 95, 95}));
  connect(arm2.frame_b, rotor2.frame_a) annotation(
    Line(points = {{28, 30}, {60, 30}}, color = {95, 95, 95}));
  connect(gain1.y, rotor2.Voltage) annotation(
    Line(points = {{47, 38}, {58, 38}}, color = {0, 0, 127}));
  connect(arm1.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{6, 70}, {-20, 70}, {-20, 0}}, color = {95, 95, 95}));
  connect(arm3.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{10, -30}, {-20, -30}, {-20, 0}}, color = {95, 95, 95}));
  connect(arm2.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{8, 30}, {-11, 30}, {-11, 0}, {-20, 0}}, color = {95, 95, 95}));
  connect(arm4.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{8, -70}, {-20, -70}, {-20, 0}}, color = {95, 95, 95}));
  connect(absoluteSensor1.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{-52, -30}, {-20, -30}, {-20, 0}}, color = {95, 95, 95}));
  connect(absoluteSensor1.r, deMultiplex31.u) annotation(
    Line(points = {{-52, -18}, {-52, -18}, {-52, 26}, {-64, 26}, {-64, 26}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-08, Interval = 0.02));
end QuadCopter3;
