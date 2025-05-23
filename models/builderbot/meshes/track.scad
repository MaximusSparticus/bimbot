// BuilderBot Track Mesh

module track() {
    // Track body
    difference() {
        // Main track body
        union() {
            // Rectangular track base
            cube([600, 100, 100], center=true);
            
            // Add some surface detail
            for (x = [-250:100:250]) {
                translate([x, 0, 50]) 
                    rotate([90, 0, 0]) 
                    cylinder(h=80, r=20, center=true, $fn=30);
            }
        }
        
        // Hollow out for weight reduction
        translate([0, 0, 20])
            cube([550, 50, 60], center=true);
    }
}

// Render and export
track();
