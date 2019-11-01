within RotorAeroLib;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model Rotor
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
  import SI = Modelica.SIunits;
  parameter Boolean rigid = true annotation(Evaluate = true);
  parameter Integer N = 6 "Blade discretization" annotation(Evaluate = true);
  parameter Boolean orientation_ccw = true "Rotor orientation (=true for counter-clockwise)"   annotation(Evaluate = true);
  parameter SI.Velocity v_freestream = 0.0 "Free-stream velocity";
  parameter Real[3] n_freestream = {1,0,0} "Free-stream velocity direction (in world coordinates)";
  parameter Real shaft_length = 0.08898774 "Length of shaft from gimbal to blades(m)";
  parameter Real twist75 = 35.0 "Twist at 3/4 span (deg) (for setting collective shift)";
  Real AdvRatio;
  //
    inner Thrust thrust(N_blade = 4, R = 0.25719, R_cutout = 0.0707, blade_chord = 0.05557, mu = 0.99, n_freestream = n_freestream, rho(displayUnit = "kg/m3") = 1.1596, useInflowCorrection = false, useMachCorrection = false, useTipLossCorrection = false, useUnsteadyAero = false, useUnsteadyNoncircAero = false, v_freestream = v_freestream) annotation(
    Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput RPM annotation(
    Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute drive(a(fixed = false), n = {1, 0, 0}, phi(fixed = true), useAxisFlange = true) annotation(
    Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorBladeAssembly bladeAssm(N = N, numBlade = 4, orientation_ccw = orientation_ccw, rigid = rigid, shaft_length = 0) annotation(
    Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body pylonMass(I_11 = 1e-10, I_22 = 1e-10, I_33 = 1e-10, animation = false, m = 1e-10) annotation(
    Placement(visible = true, transformation(origin = {88, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant collectiveSource(k = twist75) annotation(
    Placement(visible = true, transformation(origin = {60, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute driveFollower(animation = false, n = {1, 0, 0}, useAxisFlange = true) annotation(
    Placement(visible = true, transformation(origin = {-4, -32}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor1 annotation(
    Placement(visible = true, transformation(origin = {32, -54}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Position position1(exact = true, useSupport = true) annotation(
    Placement(visible = true, transformation(origin = {-16, -68}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation shaft(r = {shaft_length, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {14, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Speed speed(exact = true, phi(fixed = false), useSupport = true) annotation(
    Placement(visible = true, transformation(origin = {-4, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(collectiveSource.y, bladeAssm.collective) annotation(
    Line(points = {{71, -82}, {78, -82}, {78, -8}}, color = {0, 0, 127}));
  connect(bladeAssm.frame_a, drive.frame_b) annotation(
    Line(points = {{80, 0}, {60, 0}}, color = {95, 95, 95}));
  connect(driveFollower.frame_b, bladeAssm.relativeFrame_resolve) annotation(
    Line(points = {{6, -32}, {72, -32}, {72, 8}, {80, 8}}, color = {95, 95, 95}));
  connect(speed.w_ref, RPM) annotation(
    Line(points = {{-16, 50}, {-108, 50}, {-108, 80}, {-120, 80}}, color = {0, 0, 127}));
  connect(speed.support, drive.support) annotation(
    Line(points = {{-4, 40}, {44, 40}, {44, 10}, {44, 10}}));
  connect(speed.flange, drive.axis) annotation(
    Line(points = {{6, 50}, {50, 50}, {50, 10}, {50, 10}}));
  connect(driveFollower.frame_a, frame_a) annotation(
    Line(points = {{-14, -32}, {-100, -32}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(shaft.frame_a, frame_a) annotation(
    Line(points = {{4, 0}, {-100, 0}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(shaft.frame_b, drive.frame_a) annotation(
    Line(points = {{24, 0}, {40, 0}, {40, 0}, {40, 0}}, color = {95, 95, 95}));
  connect(angleSensor1.phi, position1.phi_ref) annotation(
    Line(points = {{20, -54}, {8, -54}, {8, -90}, {-28, -90}, {-28, -68}, {-28, -68}}, color = {0, 0, 127}));
  connect(position1.flange, driveFollower.axis) annotation(
    Line(points = {{-6, -68}, {-4, -68}, {-4, -42}, {-4, -42}}));
  connect(position1.support, driveFollower.support) annotation(
    Line(points = {{-16, -58}, {-10, -58}, {-10, -42}, {-10, -42}}));
  connect(angleSensor1.flange, drive.axis) annotation(
    Line(points = {{42, -54}, {50, -54}, {50, 10}, {50, 10}}));
  connect(pylonMass.frame_a, drive.frame_b) annotation(
    Line(points = {{78, 34}, {60, 34}, {60, 0}}, color = {95, 95, 95}));
  thrust.omega_in = drive.w;
  AdvRatio = -Modelica.Constants.pi * thrust.v_freestream / drive.w / thrust.R;
end Rotor;
