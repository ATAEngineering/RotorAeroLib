within RotorAeroLib;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model RotorBladeAssembly
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
  parameter Boolean rigid = true;
  parameter Integer N = 1 "Blade discretization" annotation(Evaluate = true);
  parameter Integer numBlade = 4 "Number of blades" annotation(Evaluate = true);
  parameter Boolean orientation_ccw = true "Rotor orientation (=true for counter-clockwise)" annotation(Evaluate = true);
  parameter Real Cl_0 = 0.0;
  parameter Real Cl_alpha = 6.28;
  parameter Real Cl_alpha2 = 0.0;
  parameter Real Cd_0 = 0.01;
  parameter Real Cd_alpha = 0.0;
  parameter Real Cd_alpha2 = 0.0;
  parameter Real blade_density = 671.801;
  parameter Real shaft_length = 0.08898774 "Distance from gimbal to blades and tip mass (m)";
  parameter Real tip_mass = 9.6919;
  //
  outer Thrust thrust;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_resolve relativeFrame_resolve annotation(
    Placement(visible = true, transformation(origin = {-100, 80}, extent = {{16, -16}, {-16, 16}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{16, -16}, {-16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation tipMassOffset(animation = false, r = {shaft_length, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-72, -26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.PointMass tipMass(animation = false, m = tip_mass, stateSelect = StateSelect.never) annotation(
    Placement(visible = true, transformation(origin = {-72, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput collective annotation(
    Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RotorAeroLib.RotorBlade[numBlade] blade(each Cd_0 = Cd_0, each Cd_alpha = Cd_alpha, each Cd_alpha2 = Cd_alpha2, each Cl_0 = Cl_0, each Cl_alpha = Cl_alpha, each Cl_alpha2 = Cl_alpha2, each N = N, each radius = thrust.R, each root_cutout = thrust.R_cutout, each chord = thrust.blade_chord, each blade_density = blade_density, each orientation_ccw = orientation_ccw, each rigid = rigid, each shaft_length = shaft_length) annotation(
    Placement(visible = true, transformation(origin = {-8, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    //
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation1a(angle = 90.0, animation = false, n = {0, 1, 0}, r = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-58, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation5a(angle = 90.0, animation = false, n = {0, 1, 0}, r = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-58, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation[numBlade] fixedRotation1b(each n = {0, 0, 1}, angle = array(i*360/numBlade for i in 0:numBlade-1), each animation = false) annotation(
    Placement(visible = true, transformation(origin = {-34, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation[numBlade] fixedRotation5b(each n = {0, 0, 1}, angle = array(i*360/numBlade for i in 0:numBlade-1), each animation = false, each r = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-28, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    //
equation
  for i in 1:numBlade loop
    connect(collective, blade[i].collective) annotation(
      Line(points = {{-120, -80}, {-20, -80}, {-20, -8}, {-20, -8}}, color = {0, 0, 127}));
    connect(fixedRotation5a.frame_b, fixedRotation5b[i].frame_a) annotation(
      Line(points = {{-100, 80}, {-48, 80}, {-48, 80}, {-48, 80}}));
    connect(fixedRotation5b[i].frame_b, blade[i].relativeFrame_resolve) annotation(
      Line(points = {{-28, 80}, {-18, 80}, {-18, 8}, {-18, 8}}, color = {95, 95, 95}, thickness = 0.5));
    connect(fixedRotation1a.frame_b, fixedRotation1b[i].frame_a) annotation(
      Line(points = {{-52, 0}, {-98, 0}, {-98, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(fixedRotation1b[i].frame_b, blade[i].frame_a) annotation(
      Line(points = {{-32, 0}, {-18, 0}, {-18, 0}, {-18, 0}}, color = {95, 95, 95}, thickness = 0.5));
  end for;
    connect(relativeFrame_resolve, fixedRotation5a.frame_a) annotation(
      Line(points = {{-100, 80}, {-48, 80}, {-48, 80}, {-48, 80}}));
    connect(frame_a, fixedRotation1a.frame_a) annotation(
      Line(points = {{-52, 0}, {-98, 0}, {-98, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(tipMassOffset.frame_b, tipMass.frame_a) annotation(
    Line(points = {{-72, -36}, {-72, -50}}, color = {95, 95, 95}));
  connect(tipMassOffset.frame_a, frame_a) annotation(
    Line(points = {{-72, -16}, {-71, -16}, {-71, 0}, {-100, 0}}, color = {95, 95, 95}));
end RotorBladeAssembly;