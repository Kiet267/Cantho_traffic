/**
 * species_vehicle.gaml
 * Chua tat ca cac loai xe: vehicle (base), motobike, car, truck, ambulance
 * Copy nguyen van tu ctt.gaml - khong chinh sua gi ca
 */
model Traffic

species vehicle skills: [driving] {

	//obj for vehicle width - chieu ngang phuong tien, duoc ghi de boi tung loai con
	float vehicle_width <- 1.0;

	init {
		right_side_driving <- true;
		safety_distance_coeff <- 3.0;
	}
	road previous_road <- nil;
	//find road
	reflex move {
		if (final_target = nil or (location distance_to final_target.location < 5.0)) {
			do die;
		}
		else{
			if (current_path = nil) {
				do compute_path graph: road_network target: final_target;
				if (current_path = nil) {
					// remove if no path available
					//write "vehicle killed due to no path";
					do die;
					return;
				}
			}	
			
			// obj for throughput tracking - dem so xe roi khoi nga tu
			if (current_road != previous_road) {
				if (previous_road != nil) {
					intersection crossed_node <- intersection(road_network target_of road(previous_road));
					if (crossed_node != nil and crossed_node.is_traffic_signal) {
						ask crossed_node { throughput_count <- throughput_count + 1; }
					}
				}
				previous_road <- road(current_road);
			}
			// stop only if red traffic light is directly ahead within 90 degrees
			traffic_light_visual light_ahead <- traffic_light_visual closest_to self;
			bool should_stop <- false;
			bool should_slow <- false;
			float dist_to_light <- (light_ahead != nil) ? self distance_to light_ahead : #infinity;

			if (light_ahead != nil and light_ahead.state = "red") {
				float angle_to_light <- float(self towards light_ahead);
				float diff_ang <- abs(angle_to_light - heading) mod 360.0;
				if (diff_ang > 180.0) { diff_ang <- 360.0 - diff_ang; }
				if (diff_ang < 90.0) {
					if (dist_to_light < 5.0)  { should_stop <- true; }  // hard stop zone
					else if (dist_to_light < 18.0) { should_slow <- true; } // braking zone
				}
			}

			if (should_stop) {
				// hold position at red light
				speed <- 0.0;
			} else if (should_slow) {
				// gradual braking: reduce speed proportionally to distance
				float brake_ratio <- (dist_to_light - 5.0) / 13.0; // 1.0 far, 0.0 at stop line
				speed <- max_speed * brake_ratio * 0.4;
				do drive;
			} else {
				// restore speed in case it was held at 0 from previous red light
				if (speed = 0.0) { speed <- max_speed * 0.5; }
				do drive;
			}
		}
		
	}

	point compute_position {
		if (current_road != nil) {
			float road_width <- road(current_road).width;
			int n_lanes <- road(current_road).num_lanes;
			float lane_w <- road_width / n_lanes;

			
			float dist_from_left_edge <- (n_lanes - lowest_lane - 0.5) * lane_w;
			float center_offset <- dist_from_left_edge - (road_width / 2);
			float final_dist <- -center_offset;
			point shift_pt <- {cos(heading + 90) * final_dist, sin(heading + 90) * final_dist};
			return location + shift_pt;
		} else {
			return location;
		}

	}

}

species motobike parent: vehicle {

	init {
		vehicle_length <- 2.0;
		//obj for vehicle width
		vehicle_width <- 0.8; // chieu ngang xe may (m)
		max_speed <- rnd(40.0, 60.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(2, 1, 1) color: #green rotate: heading at: {pos.x, pos.y, 0.5};
	}

	// heatmap point display
	aspect heat_dot {
		draw circle(10) color: rgb(255, 60, 0, 80);
	}

}

species car parent: vehicle {

	init {
		vehicle_length <- 4.0;
		//obj for vehicle width
		vehicle_width <- 1.8; // chieu ngang o to (m)
		max_speed <- rnd(30.0, 50.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(4, 2, 2) color: #red rotate: heading at: {pos.x, pos.y, 1};
	}

	// larger heatmap point for car
	aspect heat_dot {
		draw circle(13) color: rgb(255, 60, 0, 90);
	}

}

species truck parent: vehicle {

	init {
		vehicle_length <- 6.0;
		//obj for vehicle width
		vehicle_width <- 2.4; // chieu ngang xe tai (m)
		max_speed <- rnd(20.0, 40.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(6, 3, 3) color: #blue rotate: heading at: {pos.x, pos.y, 1.5};
	}

	// largest heatmap point for truck
	aspect heat_dot {
		draw circle(16) color: rgb(255, 60, 0, 100);
	}

}

species ambulance parent: vehicle {
	init {
		vehicle_length <- 5.0;
		//obj for vehicle width
		vehicle_width <- 2.0; // chieu ngang xe cuu thuong
		max_speed <- rnd(50.0, 70.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(5, 2, 2.5) color: #white rotate: heading at: {pos.x, pos.y, 1.25};
		// den chop do
		draw box(1, 2.1, 0.5) color: #red rotate: heading at: {pos.x, pos.y, 2.5};
	}

	aspect heat_dot {
		draw circle(15) color: rgb(255, 0, 0, 150);
	}
}
