within RotorAeroLib.Examples.QuadCopter;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model AltitudeController
  extends Modelica.Blocks.Interfaces.SISO;
  parameter Real kp = 1;
  parameter Real ki = 0;
  parameter Real kd = 1;
  parameter Real y_start = 1;
  parameter Real max_limit = 1000;
  parameter Real min_limit = 10;
  Modelica.Blocks.Interfaces.RealInput h_setpoint annotation(
    Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {-40, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = kp)  annotation(
    Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator1(initType = Modelica.Blocks.Types.Init.NoInit, k = ki, y_start = y_start) annotation(
    Placement(visible = true, transformation(origin = {-2, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Derivative derivative1(T = 0.01, initType = Modelica.Blocks.Types.Init.InitialState, k = kd)  annotation(
    Placement(visible = true, transformation(origin = {-2, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add3 add31 annotation(
    Placement(visible = true, transformation(origin = {34, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter1(limitsAtInit = true, uMax = max_limit, uMin = 0)  annotation(
    Placement(visible = true, transformation(origin = {74, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(limiter1.y, y) annotation(
    Line(points = {{86, 0}, {104, 0}, {104, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(add31.y, limiter1.u) annotation(
    Line(points = {{46, 0}, {62, 0}, {62, 0}, {62, 0}}, color = {0, 0, 127}));
  connect(derivative1.y, add31.u3) annotation(
    Line(points = {{10, -52}, {20, -52}, {20, -8}, {22, -8}}, color = {0, 0, 127}));
  connect(add31.u1, integrator1.y) annotation(
    Line(points = {{22, 8}, {22, 80}, {9, 80}}, color = {0, 0, 127}));
  connect(gain1.y, add31.u2) annotation(
    Line(points = {{9, 0}, {22, 0}}, color = {0, 0, 127}));
  connect(derivative1.u, feedback1.y) annotation(
    Line(points = {{-14, -52}, {-32, -52}, {-32, 80}, {-30, 80}}, color = {0, 0, 127}));
  connect(feedback1.y, integrator1.u) annotation(
    Line(points = {{-30, 80}, {-14, 80}}, color = {0, 0, 127}));
  connect(gain1.u, feedback1.y) annotation(
    Line(points = {{-14, 0}, {-32, 0}, {-32, 80}, {-30, 80}}, color = {0, 0, 127}));
  connect(u, feedback1.u2) annotation(
    Line(points = {{-120, 0}, {-40, 0}, {-40, 72}, {-40, 72}}, color = {0, 0, 127}));
  connect(h_setpoint, feedback1.u1) annotation(
    Line(points = {{-120, 80}, {-48, 80}, {-48, 80}, {-48, 80}}, color = {0, 0, 127}));
end AltitudeController;
