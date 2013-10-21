// B_ROBOT
// Arduino 3D printed Self Balancig Robot project
// Author: Jose Julio

// Arm for robot raiseup (conected to a mini servo)

module base()
{
	translate([-5,-3,0]) cube([60,6,3]);
	translate([55,-3,0]) rotate([0,0,35]) cube([30,6,3]);
	translate([52.5,-2.5,0]) rotate([0,0,325]) cube([30,6,3]);
	translate([-5,0,1.5]) cylinder(r=3,h=3,center=true,$fn=16);
	translate([78,16.8,1.5]) cylinder(r=3,h=3,center=true,$fn=16);
	translate([78,-16.8,1.5]) cylinder(r=3,h=3,center=true,$fn=16);
	//translate([42,0,2.5]) sphere(r=2.5,$fn=32);

}

module holes()
{
	cylinder(r=1.3,h=10,center=true,$fn=8);
	translate([7,0,0]) cylinder(r=1.2,h=10,center=true,$fn=8);
	translate([13.1,0,0]) cylinder(r=1.2,h=10,center=true,$fn=8);
}

difference()
{ 
	base();
	holes();
}