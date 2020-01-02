within RotorAeroLib;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model ExternalAirStation
  extends PartialAirStation;
  
equation
  [delta_Fy, delta_Fz, delta_Tx] = AirStation_external(r,v_rotor,theta,thetadot);
  delta_Fx = 0;
  delta_Ty = 0;
  delta_Tz = 0;
  
end ExternalAirStation;
