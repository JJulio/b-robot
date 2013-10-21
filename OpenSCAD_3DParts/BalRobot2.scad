// B_ROBOT
// Arduino 3D printed Self Balancig Robot project
// Author: Jose Julio


height = 140;
length = 102;
width = 50;


module plate1_base()
{
	translate([-(length+4)/2,0,0]) cube([length+4,width,3]);
	translate([-(length+4)/2,0,0]) cube([2,width,3+10]);
	translate([(length+4)/2-2,0,0]) cube([2,width,3+10]);

	translate([(length+4)/2-8,0,0]) cube([8,2,8]);
	translate([(length+4)/2-8,width-2,0]) cube([8,2,8]);
	translate([-(length+4)/2,0,0]) cube([8,2,8]);
	translate([-(length+4)/2,width-2,0]) cube([8,2,8]);

	translate([-8,(width/2)-11,0]) cube([16,22,15]);
	
}

module plate1_holes()
{
	// Motor shaft hole
	translate([length/2,width/2,24]) rotate([0,90,0]) cylinder(r=14,h=10,$fn=32,center=true);
	translate([-length/2,width/2,24]) rotate([0,90,0]) cylinder(r=14,h=10,$fn=32,center=true);

	// Motor mount holes
	translate([-(length+4)/2,9.5,5.5+3]) rotate([0,90,0]) cylinder(r=1.64,h=10,$fn=8,center=true);
	translate([-(length+4)/2,9.5+31,5.5+3]) rotate([0,90,0]) cylinder(r=1.64,h=10,$fn=8,center=true);
	translate([(length+4)/2,9.5,5.5+3]) rotate([0,90,0]) cylinder(r=1.64,h=10,$fn=8,center=true);
	translate([(length+4)/2,9.5+31,5.5+3]) rotate([0,90,0]) cylinder(r=1.64,h=10,$fn=8,center=true);

	//translate([0,width/2,0]) cylinder(r=12,h=10,$fn=24,center=true);
	translate([-30,width/2,0]) cylinder(r=14,h=10,$fn=24,center=true);
	translate([30,width/2,0]) cylinder(r=14,h=10,$fn=24,center=true);	
}

module plate2_base()
{
	translate([-(length)/2,0,0]) cube([length,width,5]);
	
}

module plate2_holes()
{
	// M3 lateral holes
	translate([(length)/2,9.5,5/2]) rotate([0,90,0]) cylinder(r=1.68,h=40,$fn=8,center=true);
	translate([(length)/2,9.5+31,5/2]) rotate([0,90,0]) cylinder(r=1.68,h=40,$fn=8,center=true);
	translate([-(length)/2,9.5,5/2]) rotate([0,90,0]) cylinder(r=1.68,h=40,$fn=8,center=true);
	translate([-(length)/2,9.5+31,5/2]) rotate([0,90,0]) cylinder(r=1.68,h=40,$fn=8,center=true);

	// M3 NUT holes
	translate([(length)/2-(4+1.26),9-2.9,-1]) cube([2.6,5.9,7]);
	translate([(length)/2-(4+1.26),9+31-2.9,-1]) cube([2.6,5.9,7]);
	translate([-(length)/2+(4-1.26),9-2.9,-1]) cube([2.6,5.9,7]);
	translate([-(length)/2+(4-1.26),9+31-2.9,-1]) cube([2.6,5.9,7]);

	// Board mount holes
	translate([45.5/2,width/2+35/2]) cylinder(r=1.72,h=20,$fn=8,center=true);
	translate([45.5/2,width/2-35/2]) cylinder(r=1.72,h=20,$fn=8,center=true);
	translate([-45.5/2,width/2+35/2]) cylinder(r=1.72,h=20,$fn=8,center=true);
	translate([-45.5/2,width/2-35/2]) cylinder(r=1.72,h=20,$fn=8,center=true);
	// Aditional holes
	translate([0,width/2+35/2]) cylinder(r=1.72,h=20,$fn=8,center=true);
	translate([0,width/2-35/2]) cylinder(r=1.72,h=20,$fn=8,center=true);
	
	// Light holes
	translate([0,width/2,0]) cylinder(r=14,h=20,$fn=24,center=true);
	translate([-32,width/2,0]) cylinder(r=10,h=20,$fn=24,center=true);
	translate([32,width/2,0]) cylinder(r=10,h=20,$fn=24,center=true);
}

module lateral_base()
{
	translate([-(height)/2,0,0]) cube([height,width,3]);
	translate([(height/2),width/2,1.5]) cylinder(r=width/2,h=3,$fn=32,center=true);
}

module lateral_holes()
{
	// Motor shaft hole
	translate([-height/2,width/2,0]) cylinder(r=12,h=10,$fn=24,center=true);
	// Motor mount holes
	translate([-(height-20)/2,9.5,0]) cylinder(r=1.68,h=10,$fn=8,center=true);
	translate([-(height-20)/2,9.5+31,0]) cylinder(r=1.68,h=10,$fn=8,center=true);

	//Holes
	for ( i = [0 : 8] ){
		translate([-(height-45-i*30)/2,9.5,0]) cylinder(r=1.7,h=10,$fn=8,center=true);
		translate([-(height-45-i*30)/2,9.5+31,0]) cylinder(r=1.7,h=10,$fn=8,center=true);
	}

	// Light holes
	for ( i = [0 : 8] ){
		translate([(height/2)-i*28,width/2,0]) cylinder(r=7,h=8,$fn=24,center=true);
	}
}


//translate([0,0,10]) rotate ([-90,0,0]) difference()

translate([0,55,0]) difference()
{
plate1_base();
plate1_holes();
}


translate([0,-55,0]) difference()
{
plate2_base();
plate2_holes();
}


translate([0,0,0]) difference()
{
lateral_base();
lateral_holes();
}

