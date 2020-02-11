within RotorAeroLib;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model RotorAeroLib_Globals
  import SI = Modelica.SIunits;
  parameter SI.Length R "Blade tip radius";
  parameter SI.Length R_cutout "Root cutout radius";
  parameter SI.Area A = Modelica.Constants.pi*(R^2-R_cutout^2);
  parameter SI.Density rho = 1.0225 "Air density";
  parameter SI.Velocity c = 343.3 "Speed of sound";
  parameter Integer N_blade = 4 "Number of blades" annotation(Evaluate=true);
  parameter SI.Length blade_chord = 0.1 "Chord length";
  parameter Real sigma = N_blade*blade_chord/Modelica.Constants.pi/R "Solidity";
  parameter Real mu = 0.95 "Tip loss factor";
  parameter SI.Velocity v_freestream = 0.0 "Free-stream velocity";
  parameter Real[3] n_freestream = {1,0,0} "Free-stream velocity direction (in world coordinates)";
  parameter Boolean useMachCorrection = true;
  parameter Boolean useAspectRatioCorrection = true;
  parameter Boolean useTipLossCorrection = true;
  parameter Boolean useInflowCorrection = true;
  parameter Boolean useUnsteadyAero = false;
  parameter Boolean useUnsteadyNoncircAero = false;
  SI.AngularVelocity omega;
  SI.Velocity Vtip;
  Modelica.Blocks.Interfaces.RealInput omega_in annotation(
    Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  omega = omega_in;
  Vtip = R*abs(omega);
  annotation(
    defaultComponentName = "RALglb",
    defaultComponentPrefixes = "inner",
    missingInnerMessage = "No \"RALglb\" component is defined. A default RotorAeroLib_Globals component with the default parameters will be used. If this is not desired, drag a RotorAeroLib_Globals block into the top level of your model.",
    Documentation(info = "<html>
<p>This model provides full-rotor variables to be used by individual AirStation blocks.
</p>
</html>"));
end RotorAeroLib_Globals;
