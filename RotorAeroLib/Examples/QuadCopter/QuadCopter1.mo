within RotorAeroLib.Examples.QuadCopter;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model QuadCopter1
  parameter Integer N = 3 annotation(
    Evaluate = true);
  parameter Real RPM1 = 720;
  parameter Real RPM2 = 720;
  parameter Real RPM3 = 720;
  parameter Real RPM4 = 720;
  //
  inner Modelica.Mechanics.MultiBody.World world(enableAnimation = true, n = {-1, 0, 0}, nominalLength = 0.25) annotation(
    Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner DeployStructLib.DSL_Globals DSLglb(enableAnimation = true) annotation(
    Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Rotor rotor1(N = N)  annotation(
    Placement(visible = true, transformation(origin = {86, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Rotor rotor2(N = N, orientation_ccw = false)  annotation(
    Placement(visible = true, transformation(origin = {86, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Rotor rotor3(N = N)  annotation(
    Placement(visible = true, transformation(origin = {86, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.Rotor rotor4(N = N, orientation_ccw = false)  annotation(
    Placement(visible = true, transformation(origin = {86, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant rotor1_RPM(k = RPM1)  annotation(
    Placement(visible = true, transformation(origin = {56, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant rotor2_RPM(k = -RPM2) annotation(
    Placement(visible = true, transformation(origin = {54, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant rotor3_RPM(k = RPM3) annotation(
    Placement(visible = true, transformation(origin = {54, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body bodyCM(I_11 = 1, I_22 = 1, I_33 = 1, angles_fixed = true, enforceStates = true, m = 372.8, r_0(each fixed = true), v_0(each fixed = true), w_0_fixed = true)  annotation(
    Placement(visible = true, transformation(origin = {-40, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm1(r = {0, 1, 0})  annotation(
    Placement(visible = true, transformation(origin = {16, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm2(r = {0, 0, 1})  annotation(
    Placement(visible = true, transformation(origin = {18, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm3(r = {0, -1, 0})  annotation(
    Placement(visible = true, transformation(origin = {20, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation arm4(r = {0, 0, -1})  annotation(
    Placement(visible = true, transformation(origin = {18, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant rotor4_RPM(k = -RPM4) annotation(
    Placement(visible = true, transformation(origin = {56, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(rotor1_RPM.y, rotor1.RPM) annotation(
    Line(points = {{67, 80}, {69, 80}, {69, 66}, {73, 66}}, color = {0, 0, 127}));
  connect(rotor4_RPM.y, rotor4.RPM) annotation(
    Line(points = {{67, -60}, {73, -60}, {73, -74}}, color = {0, 0, 127}));
  connect(rotor3_RPM.y, rotor3.RPM) annotation(
    Line(points = {{65, -20}, {69.5, -20}, {69.5, -30}, {71.75, -30}, {71.75, -32}, {74, -32}}, color = {0, 0, 127}));
  connect(arm4.frame_b, rotor4.frame_a) annotation(
    Line(points = {{28, -80}, {76, -80}}, color = {95, 95, 95}));
  connect(arm4.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{8, -80}, {-30, -80}, {-30, 0}}, color = {95, 95, 95}));
  connect(arm3.frame_b, rotor3.frame_a) annotation(
    Line(points = {{30, -40}, {76, -40}}, color = {95, 95, 95}));
  connect(arm3.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{10, -40}, {-30, -40}, {-30, 0}}, color = {95, 95, 95}));
  connect(arm2.frame_b, rotor2.frame_a) annotation(
    Line(points = {{28, 20}, {76, 20}}, color = {95, 95, 95}));
  connect(arm2.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{8, 20}, {-11, 20}, {-11, 0}, {-30, 0}}, color = {95, 95, 95}));
  connect(arm1.frame_b, rotor1.frame_a) annotation(
    Line(points = {{26, 60}, {76, 60}}, color = {95, 95, 95}));
  connect(arm1.frame_a, bodyCM.frame_a) annotation(
    Line(points = {{6, 60}, {-30, 60}, {-30, 0}}, color = {95, 95, 95}));
  connect(rotor2_RPM.y, rotor2.RPM) annotation(
    Line(points = {{65, 40}, {67.25, 40}, {67.25, 40}, {69.5, 40}, {69.5, 30}, {71.75, 30}, {71.75, 30}, {74, 30}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-08, Interval = 0.02));
end QuadCopter1;
