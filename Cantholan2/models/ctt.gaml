/**
* Name: NewModel
* Based on the internal empty template. 
* Author: smth
* Tags: 
*/
model Traffic

/* Insert your model definition here */
global {
	string appkey <- "KEY";
	image_file static_map_request;
	string map_center;
	point map_size;

	action load_map {
		float s <- world.shape.height / world.shape.width;
		map_size <- {500, 500 * s};
		string request <- "https://api.mapbox.com/styles/v1/mapbox/satellite-v9/static/" + "[" + map_center + "]/" + int(map_size.x) + "x" + int(map_size.y) + "@2x?" + "access_token=" + appkey;
		write "Request : " + request;
		static_map_request <- image_file(request, "JPEG");
	}
	file road_shp <- shape_file("../includes/road 3.shp");
	file building_shp <- shape_file("../includes/building.shp");
	file signal_shp <- shape_file("../includes/traffic_signals 6.shp");
	file roi_lane_shp <- shape_file("../includes/ROI_zones 3.shp"); // obj for ROI lane polygons from GIS
	geometry shape <- envelope(road_shp);
	graph road_network;
	float step <- 0.1 #s;
	int target_motobike <- 1000;
	int target_car <- 300;
	int target_truck <- 50;
	int target_ambulance <- 5;
	string routing_scenario <- "Bình thường";
	float spawn_rate <- 1.0;
	
	//obj for controller mode - chon che do dieu khien den giao thong
	// false = Fixed-time (timer co dinh 60s), true = CBMP (phan bo theo ap suat phi)
	bool use_cbmp <- true;
	
	//obj for KPIs (Đo lường hiệu năng)
	string csv_filename;
	
	list<intersection> spawn_nodes; // spawn points at the edge of the map
	map<road, float> road_heat;  // smoothed heat value for road density
	int heat_tick <- 0;          // step counter to update heatmap periodically

	// update road_heat using ema for smooth color transitions
	reflex update_road_counts {
		heat_tick <- heat_tick + 1;
		if (heat_tick mod 5 = 0) {
			// instant vehicle count per road
			map<road, int> cur <- map<road,int>([]);
			loop v over: (motobike as list) + (car as list) + (truck as list) + (ambulance as list) {
				if (v.current_road != nil) {
					road rd <- road(v.current_road);
					if (rd != nil) {
						cur[rd] <- (cur contains_key rd) ? cur[rd] + 1 : 1;
					}
				}
			}
			// apply ema smoothing for heat values
			loop r over: road {
				float new_val <- (cur contains_key r) ? min(float(cur[r]) / 40.0, 1.0) : 0.0;
				float old_val <- (road_heat contains_key r) ? road_heat[r] : 0.0;
				road_heat[r] <- 0.7 * old_val + 0.3 * new_val;
			}
		}
	}

	init {
		
		geometry loc <- (world.shape CRS_transform ("EPSG:4326"));
		map_center <- "" + loc.points[0].x + "," + loc.points[0].y + "," + loc.points[2].x + "," + loc.points[2].y;
		write loc;
		write map_center;
		
		if(appkey = "KEY") {
			map useless <- user_input_dialog("Please enter your MapBox access token as a value for the appkey variable in the code instead of \"KEY\".", []);			
		} else {
			do load_map;
		}
		
		write "read data";
		
		list<geometry> fixed_road <- clean_network(list<geometry>(road_shp.contents), 15.0, true, true);
		create road from: road_shp;

		create building from: building_shp;

		// obj for roi_lane - load vung quan sat lam duong tu GIS
		create roi_lane from: roi_lane_shp with: [
			link_id::string(read("link_id")),
			area_m2::float(read("area_m2")),
			u_node::string(read("u_node")),
			d_node::string(read("d_node")),
			direction::string(read("direction")),
			lane_id::string(read("lane_id")),
			phase_id::string(read("phase_id"))
		];

		graph temp_graph <- as_edge_graph(road);
		loop v over: temp_graph.vertices {
			create intersection with: [shape::point(v)] {
				is_traffic_signal <- false;
			}
		}
		road_network <- as_driving_graph(road, intersection);

		// =========================================================================
		// GIAI DOAN 1: Nap tat ca tin hieu GIS vao tac tu trung gian gis_signal_point
		// =========================================================================
		create gis_signal_point from: signal_shp with: [osm_id :: string(read("osm_id"))];

		// =========================================================================
		// GIAI DOAN 2: Gom cum cho NGA TU DAC BIET (Gan cung qua ID 1, 2, 3, 4)
		// =========================================================================
		list<gis_signal_point> special_signals <- list<gis_signal_point>(gis_signal_point) where (
			each.osm_id = "1" or each.osm_id = "2" or each.osm_id = "3" or each.osm_id = "4"
		);
		
		if (!empty(special_signals)) {
			point real_center <- mean(special_signals collect each.location);
			intersection target_node <- (intersection) closest_to(real_center);
			
			if (target_node != nil) {
				ask target_node {
					is_traffic_signal <- true;
					do compute_crossing(sig_pts: special_signals collect each.location, center_pt: real_center);
				}
				
				// Tao den visual va gan cung Truc theo cap doi dien 1-3 va 2-4
				loop sg_agent over: special_signals {
					create traffic_light_visual {
						location <- sg_agent.location;
						my_parent <- target_node;
						osm_id <- sg_agent.osm_id;
						// obj for hand-assigned axis - truc gan cung theo cap 1-3 va 2-4
						if (osm_id = "1" or osm_id = "3") {
							axis <- "axis_1";
						} else if (osm_id = "2" or osm_id = "4") {
							axis <- "axis_2";
						}
					}
				}
			}
		}

		// =========================================================================
		// GIAI DOAN 3: Gom cum cho cac NGA TU TU DONG con lai (tinh toan theo goc)
		// =========================================================================
		list<gis_signal_point> free_signals <- list<gis_signal_point>(gis_signal_point) where (
			each.osm_id != "1" and each.osm_id != "2" and each.osm_id != "3" and each.osm_id != "4"
		);
		
		loop while: !empty(free_signals) {
			gis_signal_point head_sg <- free_signals[0];
			
			// Gom cum ban kinh 70m cho cac den tu dong (Giu nguyen 70m an toan cua ctt)
			list<gis_signal_point> cluster_sg <- free_signals where (each distance_to head_sg < 70.0);
			
			if (length(cluster_sg) >= 2) {
				point real_center <- mean(cluster_sg collect each.location);
				intersection target_node <- (intersection) closest_to(real_center);
				
				if (target_node != nil) {
					ask target_node {
						is_traffic_signal <- true;
						do compute_crossing(sig_pts: cluster_sg collect each.location, center_pt: real_center);
					}
					
					// Tao den visual va phan truc dua tren goc hinh hoc
					loop sg_agent over: cluster_sg {
						create traffic_light_visual {
							location <- sg_agent.location;
							my_parent <- target_node;
							osm_id <- sg_agent.osm_id;
							// obj for auto axis assignment - phan truc theo goc tu den den tam nga tu
							float ang <- location towards real_center;
							float norm_ang <- ang mod 180;
							if (norm_ang > 45 and norm_ang < 135) {
								axis <- "axis_1";
							} else {
								axis <- "axis_2";
							}
						}
					}
				}
			}
			free_signals <- free_signals - cluster_sg;
		}
		// identify spawn nodes based on graph degree
		spawn_nodes <- intersection where (
			!each.is_traffic_signal and
			!empty(each.roads_out) and
			(length(each.roads_out) + length(each.roads_in) <= 2)
		);
		// fallback if too few spawn nodes found
		if (length(spawn_nodes) < 3) {
			spawn_nodes <- intersection where (!each.is_traffic_signal and !empty(each.roads_out));
		}
		


		int nb_moto <- 0;
		int nb_car <- 0;
		int nb_truck <- 0;
		write "done";

		// Khởi tạo file CSV
		string mode_str <- use_cbmp ? "CBMP" : "FixedTime";
		csv_filename <- "KPI_Result_" + mode_str + ".csv";
		save "Cycle,Time_Seconds,Queue_Length,Throughput_per_Cycle" to: csv_filename format: "csv" rewrite: true;

		
		list<intersection> signal_nodes <- intersection where (each.is_traffic_signal);
		loop while: not empty(signal_nodes){
			intersection seed <- signal_nodes[0];
			list<intersection> cluster <- signal_nodes where (each distance_to seed <= 50.0);
			
			create traffic_controller{
				my_nodes <- cluster;
				location <- cluster[0].location;
				ask my_nodes {
					do to_green;
				}
			}
			signal_nodes <- signal_nodes - cluster;
		}

	}

	reflex maintain_population {
		// Kiểm tra sự thiếu hụt cho từng loại xe riêng biệt
		int diff_moto <- target_motobike - length(motobike);
		int diff_car <- target_car - length(car);
		int diff_truck <- target_truck - length(truck);
		int diff_ambulance <- target_ambulance - length(ambulance);

		// Tổng số xe cần sinh ra
		int total_diff <- max(0, diff_moto) + max(0, diff_car) + max(0, diff_truck) + max(0, diff_ambulance);

		if (total_diff > 0) {
			int spawn_count <- min(total_diff, 3); // spawn up to 3 vehicles per step

			loop times: spawn_count {
				intersection end_node <- nil;
				float cx <- shape.location.x;
				float cy <- shape.location.y;
				switch routing_scenario {
					match "Trục dọc kẹt cứng" {
						list<intersection> ns <- spawn_nodes where (each.location.y < cy - 100 or each.location.y > cy + 100);
						list<intersection> ew <- spawn_nodes where (each.location.x < cx - 100 or each.location.x > cx + 100);
						if (flip(0.8) and !empty(ns)) {
							end_node <- one_of(ns);
						} else if (!empty(ew)) {
							end_node <- one_of(ew);
						} else {
							end_node <- one_of(spawn_nodes);
						}
					}
					match "Đổ dồn về phía Đông" {
						list<intersection> east <- spawn_nodes where (each.location.x > cx + 100);
						end_node <- !empty(east) ? one_of(east) : one_of(spawn_nodes);
					}
					match "Bình thường" {
						end_node <- one_of(spawn_nodes);
					}
				}

				if (end_node != nil) {
					intersection start_node <- one_of(spawn_nodes);
					if (start_node != nil and start_node != end_node) {
						// allow spawn if less than 2 vehicles within 8m
						if (length(vehicle overlapping circle(8.0, start_node.location)) < 2) {
							// Tính toán ngẫu nhiên có trọng số để ưu tiên sinh ra loại xe đang thiếu
							int rand_val <- rnd(total_diff - 1);
							if (rand_val < max(0, diff_moto)) {
								create motobike number: 1 { location <- start_node.location; final_target <- end_node; }
							} else if (rand_val < max(0, diff_moto) + max(0, diff_car)) {
								create car number: 1 { location <- start_node.location; final_target <- end_node; }
							} else if (rand_val < max(0, diff_moto) + max(0, diff_car) + max(0, diff_truck)) {
								create truck number: 1 { location <- start_node.location; final_target <- end_node; }
							} else {
								create ambulance number: 1 { location <- start_node.location; final_target <- end_node; }
							}
						}
					}
				}
			}
		}
		
		// Xóa bớt xe dư thừa ngẫu nhiên đối với từng loại riêng biệt nếu dân số vượt mốc
		if (diff_moto < 0) { ask abs(diff_moto) among (motobike as list) { do die; } }
		if (diff_car < 0) { ask abs(diff_car) among (car as list) { do die; } }
		if (diff_truck < 0) { ask abs(diff_truck) among (truck as list) { do die; } }
		if (diff_ambulance < 0) { ask abs(diff_ambulance) among (ambulance as list) { do die; } }
	}


species road skills: [road_skill] {
	int lanes <- 3;
	int num_lanes <- 3;
	float width <- 6.0;

	aspect default {
		draw shape + (width / 4) color: #black;
	}

	// base style for heatmap road display
	aspect heatmap_base {
		draw shape + (width / 4) color: rgb(35, 55, 90);
	}

	// 3-state heatmap color based on road density
	aspect heatmap_heat {
		float h <- (road_heat contains_key self) ? road_heat[self] : 0.0;
		// mapped value for max threshold
		if (h >= 0.5) {
			// high density color
			draw shape + (width / 2) color: rgb(220, 30, 30);
		} else if (h >= 0.2) {
			// medium density color
			draw shape + (width / 2) color: rgb(255, 190, 0);
		} else if (h >= 0.06) {
			// low density color
			draw shape + (width / 2) color: rgb(40, 200, 80);
		}
		// do not draw if below threshold
	}

}

species building {

	aspect default {
		draw shape color: #grey;
	}

}

// obj for roi_lane species - vung quan sat theo polygon lan duong tu shapefile
species roi_lane {
	// obj for GIS attributes - thuoc tinh tu shapefile
	string link_id;
	float area_m2;
	string u_node;
	string d_node;
	string direction;
	string lane_id;
	string phase_id;
	
	int current_vehicle_count <- 0;
	list<vehicle> vehicles_inside <- [];
	float occupancy_rate <- 0.0; // obj for occupancy - ty le chiem dung mat duong (0-100%)

	// obj for vehicle area constants - dien tich hinh chieu bang cua tung loai xe (m2)
	float motobike_area <- 1.9 * 0.7; // ~1.33 m2
	float car_area <- 4.5 * 1.8;      // ~8.1 m2
	float truck_area <- 8.0 * 2.4;    // ~19.2 m2

	// obj for occupancy monitoring - quet xe va tinh ty le chiem dung
	reflex monitor_lane_occupancy {
		vehicles_inside <- vehicle at_distance 1.0;
		current_vehicle_count <- length(vehicles_inside);
		
		float total_vehicle_area <- 0.0;
		loop v over: vehicles_inside {
			// obj for vehicle type check - dung toan tu is de dinh danh chinh xac lop cha/con
			if (v is motobike) {
				total_vehicle_area <- total_vehicle_area + motobike_area;
			} else if (v is car) {
				total_vehicle_area <- total_vehicle_area + car_area;
			} else if (v is truck) {
				total_vehicle_area <- total_vehicle_area + truck_area;
			}
		}
		
		// obj for occupancy calculation - tinh ty le lap day khong gian lien tuc
		float valid_area <- (area_m2 > 0) ? area_m2 : shape.area;
		if (valid_area > 0) {
			occupancy_rate <- min(100.0, (total_vehicle_area / valid_area) * 100);
		} else {
			occupancy_rate <- 0.0;
		}
	}

	aspect default {
		// obj for phase color - phan mau theo pha den giao thong
		string current_phase <- (phase_id != nil) ? string(phase_id) : "NONE";
		
		rgb base_color <- rgb(100, 100, 100); // Mac dinh: Xam dam
		if (current_phase = "NONE" or current_phase = "") {
			base_color <- rgb(50, 205, 50); // Re phai tu do -> Xanh la sang
		} else if (current_phase contains "_01" or current_phase contains "_02") {
			base_color <- rgb(0, 100, 255); // Truc chinh -> Xanh duong
		} else if (current_phase contains "_03" or current_phase contains "_04") {
			base_color <- rgb(255, 140, 0); // Truc phu -> Cam
		}
		
		// obj for dynamic color - do > 70% thi chuyen sang do canh bao
		rgb dynamic_lane_color <- (occupancy_rate > 70.0)
			? rgb(220, 30, 30, 160)
			: rgb(base_color.red, base_color.green, base_color.blue, 100);
		
		// obj for lane polygon drawing - ve da giac lan duong sat mat duong
		draw shape color: dynamic_lane_color border: rgb(50, 50, 50, 150) width: 1 depth: 0.02;
		
		// obj for occupancy label - hien thi nhan % khi co xe
		if (occupancy_rate > 0.0) {
			string txt <- string(occupancy_rate, "#0.0") + "%";
			draw txt at: location + {0, 0, 2.0} color: #black font: font("Arial", 12, #bold) perspective: true;
		}
	}
}

// obj for gis_signal_point - tac tu trung gian load du lieu diem den hieu GIS
species gis_signal_point {
	string osm_id; // obj for signal id - ma dinh danh tu shapefile (dung de gan tay)
}

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

species intersection skills: [intersection_skill] {
	bool is_green;
	bool is_traffic_signal;
	int throughput_count <- 0;
	
	map<road,int> queue_per_road;
	int queue_ways1 <- 0;
	int queue_ways2 <- 0;
	
	// 4 bien dem hang cho cho 4 vung nhanh (tinh tu den giao thong lui ve sau)
	int queue_N <- 0; // Nhánh từ Bắc tiến vào ngã tư (đi về hướng Nam)
	int queue_S <- 0; // Nhánh từ Nam tiến vào ngã tư (đi về hướng Bắc)
	int queue_E <- 0; // Nhánh từ Đông tiến vào ngã tư (đi về hướng Tây)
	int queue_W <- 0; // Nhánh từ Tây tiến vào ngã tư (đi về hướng Đông)
	
	//obj for area occupancy ratio (0.0 -> 1.0) cho tung nhanh - dung cho CBMP phase 1
	float phi_N <- 0.0; // φ nhánh Bắc
	float phi_S <- 0.0; // φ nhánh Nam
	float phi_E <- 0.0; // φ nhánh Đông
	float phi_W <- 0.0; // φ nhánh Tây
	float phi_out_N <- 0.0; // φ hạ lưu Bắc
	float phi_out_S <- 0.0; // φ hạ lưu Nam
	float phi_out_E <- 0.0; // φ hạ lưu Đông
	float phi_out_W <- 0.0; // φ hạ lưu Tây
	float phi_axis1 <- 0.0; // φ tổng hợp của axis_1 (ways1) - dùng cho bộ điều khiển
	float phi_axis2 <- 0.0; // φ tổng hợp của axis_2 (ways2) - dùng cho bộ điều khiển
	list<road> ways1 <- [];
	list<road> ways2 <- [];
	rgb color_fire;
	int start_phase <- 1;

	//caculate lane for intersection
	action compute_crossing(list<point> sig_pts, point center_pt) {
		ways1 <- [];
		ways2 <- [];
		if (empty(roads_in) or empty(sig_pts)) { return; }

		list<road> all_roads <- roads_in collect road(each);

		// Kiem tra xem day co phai cuoc goi tu cum dac biet duoc gan tay khong
		bool is_hand_assigned <- false;
		list<traffic_light_visual> test_lights <- traffic_light_visual where (each.my_parent = self);
		
		loop lg over: test_lights {
			if (lg.osm_id = "1" or lg.osm_id = "2" or lg.osm_id = "3" or lg.osm_id = "4") {
				is_hand_assigned <- true;
				break;
			}
		}

		if (is_hand_assigned) {
			// LOGIC CHO NGA TU LECH: Phan truc duong dua tren cap den gan tay 1-3 va 2-4
			loop lg over: test_lights {
				road best_rd <- all_roads closest_to lg;
				if (best_rd != nil) {
					if (lg.osm_id = "1" or lg.osm_id = "3") {
						if (!(ways1 contains best_rd)) { ways1 <- ways1 + [best_rd]; }
					} else if (lg.osm_id = "2" or lg.osm_id = "4") {
						if (!(ways2 contains best_rd)) { ways2 <- ways2 + [best_rd]; }
					}
				}
			}
		} else {
			// LOGIC NGUYEN BAN: Danh cho cac nga tu tu dong (tinh theo goc)
			loop sg_pt over: sig_pts {
				float ang <- sg_pt towards center_pt;
				float normalized_ang <- ang mod 180;

				road best_rd <- nil;
				float min_d <- #infinity;
				loop rd over: all_roads {
					float d <- sg_pt distance_to road(rd).shape;
					if (d < min_d) { min_d <- d; best_rd <- road(rd); }
				}
				if (best_rd != nil) {
					if (normalized_ang > 45 and normalized_ang < 135) {
						if (!(ways1 contains best_rd)) { ways1 <- ways1 + [best_rd]; }
					} else {
						if (!(ways2 contains best_rd)) { ways2 <- ways2 + [best_rd]; }
					}
				}
			}
		}

		// Tu dong bu tru nhung doan duong chua duoc map vao nhom nao
		loop rd over: all_roads {
			if (!(ways1 contains rd) and !(ways2 contains rd)) {
				if (length(ways1) <= length(ways2)) {
					ways1 <- ways1 + [rd];
				} else {
					ways2 <- ways2 + [rd];
				}
			}
		}
	}

	action to_green {
		// update visual state for green light
		color_fire <- #green;
		is_green <- true;
		// switch lights axis_1 to green and axis_2 to red
		ask traffic_light_visual where (each.my_parent = self) {
			state <- (axis = "axis_1") ? "green" : "red";
		}
	}

	action to_red {
		// update visual state for red light
		color_fire <- #red;
		is_green <- false;
		// switch lights axis_2 to green and axis_1 to red
		ask traffic_light_visual where (each.my_parent = self) {
			state <- (axis = "axis_2") ? "green" : "red";
		}
	}


	reflex calculate_queue when: is_traffic_signal {
		// Reset queue map
		loop rd over: ways1 + ways2 { queue_per_road[rd] <- 0; }
		
		list<vehicle> all_vehicles <- (motobike as list) + (car as list) + (truck as list) + (ambulance as list);
		list<vehicle> near_vehicles <- all_vehicles where (each distance_to self < 200.0);
		
		//obj for stop line boundary - lay danh sach cot den cua chinh ngo tu nay de xac dinh ranh gioi
		// Moi cot den la stop line cua mot nhanh duong di vao tuong ung
		list<traffic_light_visual> my_lights <- traffic_light_visual where (each.my_parent = self);
		
		int debug_total_near <- 0;
		int debug_on_road_in <- 0;
		int debug_is_slow <- 0;
		int debug_behind_line <- 0;
		
		int count_w1 <- 0;
		int count_w2 <- 0;
		int c_N <- 0; int c_S <- 0; int c_E <- 0; int c_W <- 0;
		
		//obj for area-based occupancy - tong dien tich chiem dung theo tung nhanh (m2)
		float area_N <- 0.0; float area_S <- 0.0;
		float area_E <- 0.0; float area_W <- 0.0;
		
		float area_out_N <- 0.0; float area_out_S <- 0.0;
		float area_out_E <- 0.0; float area_out_W <- 0.0;
		
		//obj for queue count - chi xe DUNG/CHAM (speed < 5km/h)
		int queue_c_N <- 0; int queue_c_S <- 0;
		int queue_c_E <- 0; int queue_c_W <- 0;
		
		//obj for detection zone area - dien tich Z_n se duoc cap nhat theo chieu rong duong thuc te
		float detect_length <- 150.0;
		float zone_N <- detect_length * 6.0;
		float zone_S <- detect_length * 6.0;
		float zone_E <- detect_length * 6.0;
		float zone_W <- detect_length * 6.0;
		
		loop v over: near_vehicles {
			debug_total_near <- debug_total_near + 1;
			if (v.current_road != nil) {
				road current_rd <- road(v.current_road);
				intersection dest_node <- intersection(road_network target_of current_rd);
				intersection src_node <- intersection(road_network source_of current_rd);
				
				bool is_incoming <- (dest_node != nil and (dest_node distance_to self < 50.0));
				bool is_outgoing <- (src_node != nil and (src_node distance_to self < 50.0));
				
				if (is_incoming or is_outgoing) {
					debug_on_road_in <- debug_on_road_in + 1;
					
					//obj for vehicle footprint area = length x width (m2) - cong thuc (1)
					float veh_area <- v.vehicle_length * v.vehicle_width;
					
					if (is_incoming) {
						//obj for stop line check - tim cot den gan nhat cung huong voi xe
						float ang_center_to_veh <- float(self.location towards v.location);
						traffic_light_visual stop_light <- nil;
						if (!empty(my_lights)) {
							stop_light <- my_lights with_min_of (
								abs(((float(self.location towards each.location) - ang_center_to_veh) + 360.0) mod 360.0)
							);
						}
						
						bool behind_stop_line <- true;
						if (stop_light != nil) {
							float dist_vehicle <- v distance_to self;
							float dist_light   <- stop_light distance_to self;
							behind_stop_line <- (dist_vehicle > dist_light);
						}
						
						if (behind_stop_line) {
							debug_behind_line <- debug_behind_line + 1;
							
							// Xac dinh xe uu tien (Cong thuc 5 & 6)
							float alpha_j <- 0.0;
							if (v is ambulance) { alpha_j <- 2.0; } // obj for ambulance amplification
							
							// obj for priority amplification - nhan (1 + alpha_j) vao dien tich tuong duong
							float effective_area <- veh_area * (1.0 + alpha_j);
							
							float ang_to_center <- float(v.location towards self.location);
							bool is_slow <- (v.speed < 5 #km/#h or v.real_speed < 5 #km/#h);
							if (is_slow) { debug_is_slow <- debug_is_slow + 1; }
							
							if (ang_to_center >= 315 or ang_to_center < 45) {
								count_w2 <- count_w2 + 1;
								area_W <- area_W + effective_area;
								zone_W <- detect_length * current_rd.width;
								if (is_slow) { c_W <- c_W + 1; queue_c_W <- queue_c_W + 1; }
							} else if (ang_to_center >= 45 and ang_to_center < 135) {
								count_w1 <- count_w1 + 1;
								area_N <- area_N + effective_area;
								zone_N <- detect_length * current_rd.width;
								if (is_slow) { c_N <- c_N + 1; queue_c_N <- queue_c_N + 1; }
							} else if (ang_to_center >= 135 and ang_to_center < 225) {
								count_w2 <- count_w2 + 1;
								area_E <- area_E + effective_area;
								zone_E <- detect_length * current_rd.width;
								if (is_slow) { c_E <- c_E + 1; queue_c_E <- queue_c_E + 1; }
							} else {
								count_w1 <- count_w1 + 1;
								area_S <- area_S + effective_area;
								zone_S <- detect_length * current_rd.width;
								if (is_slow) { c_S <- c_S + 1; queue_c_S <- queue_c_S + 1; }
							}
						}
					}
					
					if (is_outgoing) {
						// Xe dang roi khoi nga tu (ha luu)
						float ang_from_center <- float(self.location towards v.location);
						if (ang_from_center >= 315 or ang_from_center < 45) {
							area_out_E <- area_out_E + veh_area;
						} else if (ang_from_center >= 45 and ang_from_center < 135) {
							area_out_S <- area_out_S + veh_area;
						} else if (ang_from_center >= 135 and ang_from_center < 225) {
							area_out_W <- area_out_W + veh_area;
						} else {
							area_out_N <- area_out_N + veh_area;
						}
					}
				}
			}
		}
		
		// Cap nhat bien dem hang cho
		queue_ways1 <- count_w1;
		queue_ways2 <- count_w2;
		queue_N <- queue_c_N; queue_S <- queue_c_S;
		queue_E <- queue_c_E; queue_W <- queue_c_W;
		
		//obj for phi - cong thuc (1): phi = tong_dien_tich_xe / dien_tich_Z_n, rang buoc 0 <= phi <= 1
		phi_N <- (zone_N > 0) ? min(area_N / zone_N, 1.0) : 0.0;
		phi_S <- (zone_S > 0) ? min(area_S / zone_S, 1.0) : 0.0;
		phi_E <- (zone_E > 0) ? min(area_E / zone_E, 1.0) : 0.0;
		phi_W <- (zone_W > 0) ? min(area_W / zone_W, 1.0) : 0.0;
		
		// Tinh luon phi cho cac nhanh ha luu (xem nhu zone bang 150m * 6m de uoc luong)
		float zone_default <- 150.0 * 6.0;
		phi_out_N <- min(area_out_N / zone_default, 1.0);
		phi_out_S <- min(area_out_S / zone_default, 1.0);
		phi_out_E <- min(area_out_E / zone_default, 1.0);
		phi_out_W <- min(area_out_W / zone_default, 1.0);
		
		//obj for axis pressure - tong phi cac nhanh cung pha
		phi_axis1 <- phi_N + phi_S;
		phi_axis2 <- phi_E + phi_W;
		
		// Debug cho intersection33
		if (name = "intersection33" and cycle mod 10 = 0) {
			//write "--- Cycle " + cycle + " | " + name + " ---";
			//write "near: " + debug_total_near + " | on_road: " + debug_on_road_in
			    //+ " | behind_stop_line: " + debug_behind_line + " | slow: " + debug_is_slow;
			if (debug_behind_line > 0) {
//				write "  Queue  | N:" + queue_N + " S:" + queue_S + " E:" + queue_E + " W:" + queue_W;
				float pN <- round(phi_N * 1000) / 10.0;
				float pS <- round(phi_S * 1000) / 10.0;
				float pE <- round(phi_E * 1000) / 10.0;
				float pW <- round(phi_W * 1000) / 10.0;
				float pa1 <- round(phi_axis1 * 1000) / 10.0;
				float pa2 <- round(phi_axis2 * 1000) / 10.0;
//				write "  φ(%)   | N:" + pN + "% S:" + pS + "% E:" + pE + "% W:" + pW + "%";
//				write "  φ_axis | axis1(N+S):" + pa1 + "% | axis2(E+W):" + pa2 + "%";
			}
		}
	}


	aspect default {
		if (is_traffic_signal) {
			// Hiển thị tên các ngã tư có đèn giao thông trực tiếp lên map 3D
			//draw name at: {location.x, location.y, 10} color: #yellow font: font("Arial", 18, #bold);
			
//			rgb light_color <- is_green ? #green : #red;
//			draw cylinder(0.3, 5) at: location color: #black;
//	        draw sphere(1.5) at: {location.x, location.y, 5} color: light_color;
		}else{
//			draw circle(1) color: color;
		}
	}
}
species traffic_controller {
	list<intersection> my_nodes;
	
	//obj for KPIs
	int my_queue <- 0;
	int my_throughput <- 0;
	
	//obj for fixed-time mode - thoi gian chuyen pha co dinh
	float time_to_change <- 60 #s;
	float counter <- 0.0;
	bool is_green <- true;
	
	int queue_ways1 <- 0;
	int queue_ways2 <- 0;
	
	//obj for CBMP mode parameters
	// tau = 120s (chu ky), L = 4s (thoi gian mat mat), kappa = 10s (xanh toi thieu)
	float cycle_duration <- 120 #s;
	float lost_time <- 4 #s;
	float min_green <- 10 #s;
	float cbmp_counter <- 0.0;
	
	//obj for cycle-based green time - g1/g2 chi duoc tinh 1 LAN moi chu ky
	// (dung thiet ke CBMP: "Cycle-Based" = phan bo cho chu ky KE TIEP)
	float g1 <- 60 #s; // thoi gian xanh pha 1 trong chu ky hien tai
	float g2 <- 60 #s; // thoi gian xanh pha 2 trong chu ky hien tai
	
	int completed_cycles <- 0;
	
	action log_kpi {
		completed_cycles <- completed_cycles + 1;
		
		// Luu ra file CSV
		string row <- "" + completed_cycles + "," + round(time) + "," + my_queue + "," + my_throughput;
		save row to: csv_filename format: "csv" rewrite: false;
		
		// Reset throughput cho chu ky tiep theo
		loop node over: my_nodes {
			node.throughput_count <- 0; 
		}
		my_throughput <- 0;
	}
	
	// Cap nhat lien tuc de bieu do nhay tung giay
	reflex update_live_metrics {
		int temp_q <- 0;
		int temp_t <- 0;
		loop node over: my_nodes {
			temp_q <- temp_q + node.queue_N + node.queue_S + node.queue_E + node.queue_W;
			temp_t <- temp_t + node.throughput_count;
		}
		my_queue <- temp_q;
		my_throughput <- temp_t;
	}
	
	//obj for compute_green_time - tinh g1/g2 cho CHU KY KE TIEP dua vao phi hien tai
	action compute_green_time {
		// Tinh toan ap suat (Pressure) theo Cong thuc 4: w = phi_in - sum(R * phi_out)
		// Gia su ty le re co dinh: 70% di thang, 15% re trai, 15% re phai
		float p_straight <- 0.7;
		float p_left <- 0.15;
		float p_right <- 0.15;
		
		float w_N <- 0.0; float w_S <- 0.0; float w_E <- 0.0; float w_W <- 0.0;
		loop node over: my_nodes {
			w_N <- w_N + max(0.0, node.phi_N - (p_straight * node.phi_out_S + p_left * node.phi_out_E + p_right * node.phi_out_W));
			w_S <- w_S + max(0.0, node.phi_S - (p_straight * node.phi_out_N + p_left * node.phi_out_W + p_right * node.phi_out_E));
			w_E <- w_E + max(0.0, node.phi_E - (p_straight * node.phi_out_W + p_left * node.phi_out_S + p_right * node.phi_out_N));
			w_W <- w_W + max(0.0, node.phi_W - (p_straight * node.phi_out_E + p_left * node.phi_out_N + p_right * node.phi_out_S));
		}
		
		// Ap suat tong hop cua pha (Cong thuc 7)
		// Gia dinh: axis_1 la pha Bac-Nam, axis_2 la pha Dong-Tay
		// He so nang luc thong hanh Clm = 1.0 cho tat ca
		float gamma1 <- w_N + w_S;
		float gamma2 <- w_E + w_W;
		float gamma_total <- gamma1 + gamma2;
		
		//obj for available time ratio: 1 - L/tau (cong thuc 9)
		float available_ratio <- 1.0 - (lost_time / cycle_duration);
		float min_ratio <- min_green / cycle_duration; // kappa/tau

		float lam1 <- 0.0;
		float lam2 <- 0.0;
		
		if (gamma_total <= 0) {
			// Khong co ap suat: chia deu, van dam bao min
			lam1 <- available_ratio / 2.0;
			lam2 <- available_ratio / 2.0;
		} else {
			//obj for min green constraint - rang buoc kappa PHAI DUOC AP TRUOC
			float remainder <- available_ratio - 2.0 * min_ratio;
			
			if (remainder <= 0.0) {
				lam1 <- available_ratio / 2.0;
				lam2 <- available_ratio / 2.0;
			} else {
				// Buoc 2: phan bo phan con lai ty le theo ap suat gamma
				lam1 <- min_ratio + remainder * (gamma1 / gamma_total);
				lam2 <- min_ratio + remainder * (gamma2 / gamma_total);
			}
		}
		
		//obj for g_S calculation - cong thuc (11): g_S = lambda*_S x tau
		g1 <- lam1 * cycle_duration;
		g2 <- lam2 * cycle_duration;
		
		//obj for CBMP verification debug - in ra moi lan tinh chu ky moi
		if (!empty(my_nodes) and my_nodes[0].name = "intersection33") {
			float g_total <- round((g1 + g2) * 10) / 10.0;
			write "=== [CBMP] Cycle " + cycle + " | controller cho intersection33 ===";
			write "  γ1(axis1): " + (round(gamma1 * 1000) / 10.0) + "% | γ2(axis2): " + (round(gamma2 * 1000) / 10.0) + "%";
			if (gamma_total <= 0) {
				write "  [!] Canh bao: phi = 0, chia deu thoi gian (CBMP chua hoat dong, kiem tra detection zone)";
			}
			write "  g1(N+S xanh): " + (round(g1 * 10) / 10.0) + "s | g2(E+W xanh): " + (round(g2 * 10) / 10.0) + "s | tong: " + g_total + "s";
			bool g1_ok <- g1 >= min_green;
			bool g2_ok <- g2 >= min_green;
			bool total_ok <- abs(g1 + g2 - (cycle_duration - lost_time)) < 0.5;
			write "  Kiem tra: g1>=" + min_green + "s? " + (g1_ok ? "OK" : "FAIL") 
			    + " | g2>=" + min_green + "s? " + (g2_ok ? "OK" : "FAIL")
			    + " | tong hop le? " + (total_ok ? "OK" : "FAIL");
		}
	}
	
	reflex run_cycle {
		if (!use_cbmp) {
			//obj for fixed-time logic - logic cu giu nguyen
			counter <- counter + step;
			if (counter >= time_to_change) {
				counter <- 0.0;
				ask my_nodes {
					if (is_green) { do to_red; }
					else { do to_green; }
				}
				is_green <- !is_green; // cap nhat trang thai cua controller
				if (is_green) { do log_kpi; } // Ket thuc 1 chu ky
			}
		} else {
			cbmp_counter <- cbmp_counter + step;
			
			if (is_green) {
				// Pha 1 dang xanh - doi du g1 giay thi chuyen sang pha 2
				if (cbmp_counter >= g1) {
					cbmp_counter <- 0.0;
					ask my_nodes { do to_red; }  // pha 1 do -> pha 2 xanh
					is_green <- false;
					//obj for next cycle planning - tinh g1/g2 cho chu ky ke tiep ngay tai day
					do compute_green_time;
				}
			} else {
				// Pha 2 dang xanh - doi du g2 giay thi quay lai pha 1
				if (cbmp_counter >= g2) {
					cbmp_counter <- 0.0;
					ask my_nodes { do to_green; } // pha 2 do -> pha 1 xanh
					is_green <- true;
					do log_kpi; // Ket thuc 1 chu ky
					do compute_green_time;
				}
			}
		}
	}
}

species traffic_light_visual {
    intersection my_parent;
    road my_road; // optional variable for compatibility
    string axis;  // axis identifier
    string state <- "red"; // visual state of traffic light
    string osm_id; // obj for id - ma dinh danh tu shapefile

    aspect default {
    	rgb light_color <- (state = "green") ? #green : #red;
        draw cylinder(0.3, 5) color: #black;
        draw sphere(1.2) at: {location.x, location.y, 5} color: light_color;
    }
}

}
experiment test type: gui {
	parameter "Lưu lượng xe máy:" var: target_motobike min: 0 max: 3000;
	parameter "Lưu lượng ô tô:" var: target_car min: 0 max: 2000;
	parameter "Lưu lượng xe tải:" var: target_truck min: 0 max: 1000;
	//parameter "Lưu lượng cứu thương:" var: target_ambulance min: 0 max: 50;
	parameter "Cb-MP:" var: use_cbmp;
	//parameter "Kịch bản di chuyển:" var: routing_scenario among: ["Bình thường", "Trục dọc kẹt cứng", "Đổ dồn về phía Đông"];
	output {
		display main type: 3d background: #lightskyblue axes: false {
		
			//image static_map_request;
			species road refresh: false;
			species roi_lane refresh: true; // obj for roi_lane display - hien thi vung quan sat lan duong
			species motobike;
			species car;
			species truck;
			//species ambulance;
			species intersection;
			species traffic_light_visual;
		}
		
//		display heatmap type: 3d background: rgb(8, 12, 25) axes: false {
//			// base road layer
//			species road aspect: heatmap_base refresh: false;
//			// overlay heat dots based on actual vehicle positions
//			species motobike aspect: heat_dot;
//			species car aspect: heat_dot;
//			species truck aspect: heat_dot;
//			//species ambulance aspect: heat_dot;
//		}
		
//		display KPI_Charts type: java2D {
//			chart "Lưu lượng thông hành toàn mạng (Throughput / Chu kỳ)" type: series size: {1, 0.5} position: {0, 0} {
//				data "Số xe thoát (xe/chu kỳ)" value: sum(traffic_controller collect each.my_throughput) color: #green marker: false;
//			}
//			chart "Chiều dài hàng chờ toàn mạng (Queue Length)" type: series size: {1, 0.5} position: {0, 0.5} {
//				data "Số xe đang kẹt tại các ngã tư" value: sum(traffic_controller collect each.my_queue) color: #red marker: false;
//			}
//		}
	}
}