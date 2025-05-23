// InspectorBot Base Link Mesh

module base_link() {
    // Main body
    difference() {
        // Outer body
        union() {
            // Main rectangular body
            cube([500, 500, 200], center=true);
            
            // Slight rounding of edges
            minkowski() {
                cube([450, 450, 180], center=true);
                sphere(r=10, $fn=20);
            }
        }
        
        // Cutouts for propeller mounts
        translate([250, 250, 0]) 
            cylinder(h=220, r=50, center=true, $fn=30);
        
        translate([250, -250, 0]) 
            cylinder(h=220, r=50, center=true, $fn=30);
        
        translate([-250, 250, 0]) 
            cylinder(h=220, r=50, center=true, $fn=30);
        
        translate([-250, -250, 0]) 
            cylinder(h=220, r=50, center=true, $fn=30);
    }
}

// Render and export
base_link();
