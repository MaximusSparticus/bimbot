// BuilderBot Base Link Mesh

module base_link() {
    // Main chassis body
    difference() {
        // Outer body
        union() {
            // Main rectangular body
            cube([600, 400, 200], center=true);
            
            // Slight rounding of edges
            minkowski() {
                cube([550, 350, 180], center=true);
                sphere(r=10, $fn=20);
            }
        }
        
        // Cutouts for tracks
        translate([0, 250, 0]) 
            cube([500, 100, 180], center=true);
        
        translate([0, -250, 0]) 
            cube([500, 100, 180], center=true);
    }
}

// Render and export
base_link();
