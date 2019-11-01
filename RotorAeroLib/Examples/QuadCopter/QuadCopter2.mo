within RotorAeroLib.Examples.QuadCopter;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model QuadCopter2
  parameter Integer N = 3 annotation(
    Evaluate = true);
  parameter Real RPM1 = 720;
  parameter Real RPM2 = 720;
  parameter Real RPM3 = 720;
  parameter Real RPM4 = 720;
  //
  inner Modelica.Mechanics.MultiBody.World world(enableAnimation = true, n = {-1, 0, 0}, nominalLength = 0.25) annotation(
    Placement(visible = true, transformation(origin = {-98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner DeployStructLib.DSL_Globals DSLglb(enableAnimation = true) annotation(
    Placement(visible = true, transformation(origin = {-102, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Rotor rotor1(N = N) annotation(
    Placement(visible = true, transformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Rotor rotor2(N = N, orientation_ccw = false) annotation(
    Placement(visible = true, transformation(origin = {100, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Rotor rotor3(N = N) annotation(
    Placement(visible = true, transformation(origin = {100, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Rotor rotor4(N = N, orientation_ccw = false) annotation(
    Placement(visible = true, transformation(origin = {100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Step altitude_setpoint(height = 5.0, startTime = 2)  annotation(
    Placement(visible = true, transformation(origin = {-92, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body bodyCM(I_11 = 1, I_22 = 1, I_33 = 1, angles_fixed = true, enforceStates = true, m = 372.8, r_0(each fixed = true), v_0(each fixed = true), w_0_fixed = true) annotation(
    Placement(visible = true, transformation(origin = {-40, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm1(r = {0, 1, 0}) annotation(
    Placement(visible = true, transformation(origin = {16, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm2(r = {0, 0, 1}) annotation(
    Placement(visible = true, transformation(origin = {18, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm3(r = {0, -1, 0}) annotation(
    Placement(visible = true, transformation(origin = {20, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm4(r = {0, 0, -1}) annotation(
    Placement(visible = true, transformation(origin = {18, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Examples.QuadCopter.AltitudeController altitudeController1(kd = 100, kp = 150, y_start = 720)  annotation(
    Placement(visible = true, transformation(origin = {-54, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteSensor absoluteSensor1(animation = false, get_r = true, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)  annotation(
    Placement(visible = true, transformation(origin = {-62, -30}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  Modelica.Blocks.Routing.DeMultiplex3 deMultiplex31 annotation(
    Placement(visible = true, transformation(origin = {-64, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Math.Gain gain1(k = -1)  annotation(
    Placement(visible = true, transformation(origin = {72, 38}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = -1) annotation(
    Placement(visible = true, transformation(origin = {72, -62}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
initial equation
  bodyCM.a_0[1] = 0;
equation
  connect(gain1.y, rotor2.RPM) annotation(
    Line(points = {{78, 38}, {86, 38}, {86, 38}, {88, 38}}, color = {0, 0, 127}));
  connect(gain2.y, rotor4.RPM) annotation(
    Line(points = {{78, -62}, {88, -62}, {88, -62}, {88, -62}}, color = {0, 0, 127}));
  connect(altitudeController1.y, gain2.u) annotation(
    Line(points = {{-42, 84}, {124, 84}, {124, -48}, {64, -48}, {64, -62}, {64, -62}}, color = {0, 0, 127}));
  connect(altitudeController1.y, gain1.u) annotation(
    Line(points = {{-42, 84}, {124, 84}, {124, 52}, {64, 52}, {64, 38}, {64, 38}}, color = {0, 0, 127}));
  connect(altitudeController1.u, deMultiplex31.y1[1]) annotation(
    Line(points = {{-66, 84}, {-72, 84}, {-72, 50}, {-72, 50}}, color = {0, 0, 127}));
  connect(absoluteSensor1.r, deMultiplex31.u) annotation(
    Line(points = {{-52, -18}, {-52, -18}, {-52, 26}, {-64, 26}, {-64, 26}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absoluteSensor1.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{-52, -30}, {-30, -30}, {-30, 0}, {-30, 0}}, color = {95, 95, 95}));
  connect(altitudeController1.y, rotor3.RPM) annotation(
    Line(points = {{-42, 84}, {124, 84}, {124, -10}, {88, -10}, {88, -22}, {88, -22}}, color = {0, 0, 127}));
  connect(altitudeController1.y, rotor1.RPM) annotation(
    Line(points = {{-42, 84}, {88, 84}, {88, 78}, {88, 78}}, color = {0, 0, 127}));
  connect(altitude_setpoint.y, altitudeController1.h_setpoint) annotation(
    Line(points = {{-80, 84}, {-76, 84}, {-76, 92}, {-66, 92}, {-66, 92}}, color = {0, 0, 127}));
  connect(arm4.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{8, -70}, {-30, -70}, {-30, 0}, {-30, 0}}, color = {95, 95, 95}));
  connect(arm4.frame_b, rotor4.frame_a) annotation(
    Line(points = {{28, -70}, {90, -70}}, color = {95, 95, 95}));
  connect(arm2.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{8, 30}, {-11, 30}, {-11, 0}, {-30, 0}}, color = {95, 95, 95}));
  connect(arm2.frame_b, rotor2.frame_a) annotation(
    Line(points = {{28, 30}, {90, 30}}, color = {95, 95, 95}));
  connect(arm3.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{10, -30}, {-30, -30}, {-30, 0}}, color = {95, 95, 95}));
  connect(arm3.frame_b, rotor3.frame_a) annotation(
    Line(points = {{30, -30}, {90, -30}}, color = {95, 95, 95}));
  connect(arm1.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{6, 70}, {-30, 70}, {-30, 0}, {-30, 0}, {-30, 0}}, color = {95, 95, 95}));
  connect(arm1.frame_b, rotor1.frame_a) annotation(
    Line(points = {{26, 70}, {90, 70}, {90, 70}, {90, 70}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-08, Interval = 0.02));
end QuadCopter2;
