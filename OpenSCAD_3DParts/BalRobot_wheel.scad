// B_ROBOT
// Arduino 3D printed Self Balancig Robot project
// Author: Jose Julio

// Robot wheels

//-- Wheel parameters
wheel_or_idiam = 90;                   //-- O-ring inner diameter
wheel_or_diam = 2;                     //-- O-ring section diameter
wheel_height = 3;    
wheel_center_drill = 5.1;
wheel_hub_diam = 24;
wheel_hub_height = 10;

M3_r = 1.68;
M3_nut_size = 5.9;
M3_nut_width = 2.6;


//-------------------------------------------------------
//--- Parameters:
//-- or_idiam: O-ring inner diameter
//-- or_diam: O-ring section diameter
//-- h: Height of the wheel
//--
//--  Module for generating a raw wheel, with no drills
//--  and no servo horn
//-------------------------------------------------------
module raw_wheel(or_idiam=50, or_diam=3, h=6)
{
   //-- Wheel parameters
   r = or_idiam/2;  

difference(){
  union(){
  translate([0,0,(h/2)]) cylinder (r=r, h=1, $fn=100,center=true); 
  cylinder (r=r-0.65, h=h, $fn=100,center=true); 
  translate([0,0,-(h/2)]) cylinder (r=r, h=1, $fn=100,center=true);
  //HUB
  translate([0,0,h/2+(wheel_hub_height/2)]) cylinder(r=wheel_hub_diam/2,h=wheel_hub_height,$n=32,center=true);
  }

  translate([2*r/5,2*r/5,0]) cylinder(r=(2*r/10),h=h+2,$fn=64,center=true);
  translate([-2*r/5,2*r/5,0]) cylinder(r=(2*r/10),h=h+2,$fn=64,center=true);
  translate([-2*r/5,-2*r/5,0]) cylinder(r=(2*r/10),h=h+2,$fn=64,center=true);
  translate([2*r/5,-2*r/5,0]) cylinder(r=(2*r/10),h=h+2,$fn=64,center=true);

  translate([6*r/8,0,0]) cylinder(r=(r/10),h=h+2,$fn=64,center=true);
  translate([-6*r/8,0,0]) cylinder(r=(r/10),h=h+2,$fn=64,center=true);
  translate([0,6*r/8,0]) cylinder(r=(r/10),h=h+2,$fn=64,center=true);
  translate([0,-6*r/8,0]) cylinder(r=(r/10),h=h+2,$fn=64,center=true);

  //M3 hole
  translate([0,0,(h/2)+(wheel_hub_height/2)+1]) rotate([90,0,0]) cylinder(r=M3_r,h=wheel_hub_diam*2,$fn=10,center=true);
  // M3 nut hole
  translate([0,wheel_hub_diam/4,(h/2)+(wheel_hub_height/2)]) cube([M3_nut_size,M3_nut_width,wheel_hub_height+2],center=true);
  translate([0,-wheel_hub_diam/4,(h/2)+(wheel_hub_height/2)]) cube([M3_nut_size,M3_nut_width,wheel_hub_height+2],center=true);
  }
}


difference() {
      raw_wheel(or_idiam=wheel_or_idiam, or_diam=wheel_or_diam, h=wheel_height);

      //-- Inner drill
      cylinder(center=true, h=2*wheel_height + wheel_hub_height*2 +10, r=wheel_center_drill/2,$fn=16);
}

