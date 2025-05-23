// InspectorBot Propeller Mesh

module propeller() {
    difference() {
        // Main propeller body
        union() {
            // Cylindrical base
            cylinder(h=100, r=50, center=true, $fn=50);
            
            // Blade-like extensions
            for (i = [0:3]) {
                rotate([0, 0, i * 90]) {
                    translate([0, 40, 0]) {
                        // Blade shape
                        hull() {
                            cube([10, 20, 10], center=true);
                            translate([0, 40, 0]) 
                                cube([5, 10, 5], center=true);
                        }
                    }
                }
            }
        }
        
        // Central hole for mounting
        cylinder(h=110, r=10, center=true, $fn=30);
    }
}

// Render and export
propeller();
