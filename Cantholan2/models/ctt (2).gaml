/**
* Name: NewModel
* Based on the internal empty template. 
* Author: smth
* Tags: 
*/
model Traffic

/* Insert your model definition here */
global {
	file road_shp <- shape_file("../includes/road 3.shp");
	file building_shp <- shape_file("../includes/building.shp");
	file signal_shp <- shape_file("../includes/traffic_signals 6.shp");
	file roi_lane_shp <- shape_file("../includes/ROI_zones 3.shp");
	
	geometry shape <- envelope(road_shp);
	graph road_network;
	float step <- 0.1 #s;
	int target_motobike <- 1000;
	int target_car <- 500;
	int target_truck <- 100;
	string routing_scenario <- "Bình thường";
	float spawn_rate <- 1.0;
	
	list<intersection> spawn_nodes; // spawn points at the edge of the map
	map<road, float> road_heat;  // smoothed heat value for road density
	int heat_tick <- 0;          // step counter to update heatmap periodically

	// update road_heat using ema for smooth color transitions
	reflex update_road_counts {
		heat_tick <- heat_tick + 1;
		if (heat_tick mod 5 = 0) {
			// instant vehicle count per road
			map<road, int> cur <- map<road,int>([]);
			loop v over: (motobike as list) + (car as list) + (truck as list) {
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
		write "read data";
		
		list<geometry> fixed_road <- clean_network(list<geometry>(road_shp.contents), 15.0, true, true);
		create road from: road_shp;

		create building from: building_shp;
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

		// 1. Nạp toàn bộ dữ liệu file SHP vào các tác tử trung gian để giữ nguyên bảng thuộc tính
		create gis_signal_point from: signal_shp with: [osm_id :: string(read("osm_id"))];

		// =========================================================================
		// GIAI ĐOẠN 1: GOM CỤM CHO NGÃ TƯ ĐẶC BIỆT (GÁN CỨNG QUA ID 1, 2, 3, 4)
		// =========================================================================
		list<gis_signal_point> special_signals <- list<gis_signal_point>(gis_signal_point) where (
			each.osm_id = "1" or each.osm_id = "2" or each.osm_id = "3" or each.osm_id = "4"
		);
		
		if (!empty(special_signals)) {
			// Xác định tâm thực tế và tìm nút giao gần nhất cho cụm đặc biệt này
			point real_center <- mean(special_signals collect each.location);
			intersection target_node <- (intersection) closest_to(real_center);
			
			if (target_node != nil) {
				ask target_node {
					is_traffic_signal <- true;
					// Đồng bộ các tuyến đường cho ngã tư đặc biệt
					do compute_crossing(sig_pts: special_signals collect each.location, center_pt: real_center);
				}
				
				// Tạo các thực thể đèn hiển thị và gán cứng Trục theo cặp đối diện 1-3 và 2-4
				loop sg_agent over: special_signals {
					create traffic_light_visual {
						location <- sg_agent.location;
						my_parent <- target_node;
						self.osm_id <- sg_agent.osm_id;
						
						if (self.osm_id = "1" or self.osm_id = "3") {
							axis <- "axis_1";
						} else if (self.osm_id = "2" or self.osm_id = "4") {
							axis <- "axis_2";
						}
					}
				}
			}
		}

		// =========================================================================
		// GIAI ĐOẠN 2: GOM CỤM CHO CÁC NGÃ TƯ TỰ ĐỘNG CÒN LẠI (TÍNH TOÁN THEO GÓC)
		// =========================================================================
		list<gis_signal_point> free_signals <- list<gis_signal_point>(gis_signal_point) where (
			each.osm_id != "1" and each.osm_id != "2" and each.osm_id != "3" and each.osm_id != "4"
		);
		
		loop while: !empty(free_signals) {
			gis_signal_point head_sg <- free_signals[0];
			
			// Gom cụm bán kính 300m cho các đèn tự động
			list<gis_signal_point> cluster_sg <- free_signals where (each distance_to head_sg < 300.0);
			
			if (length(cluster_sg) >= 2) {
				point real_center <- mean(cluster_sg collect each.location);
				intersection target_node <- (intersection) closest_to(real_center);
				
				if (target_node != nil) {
					ask target_node {
						is_traffic_signal <- true;
						// Chạy hàm tính toán liên kết đường bằng logic nguyên bản của bạn
						do compute_crossing(sig_pts: cluster_sg collect each.location, center_pt: real_center);
					}
					
					// Tạo đèn visual và phân trục dựa trên tính toán góc hình học (Không động tới ID)
					loop sg_agent over: cluster_sg {
						create traffic_light_visual {
							location <- sg_agent.location;
							my_parent <- target_node;
							self.osm_id <- sg_agent.osm_id;
							
							// Sử dụng đúng logic góc nguyên bản đã chạy đúng của bạn
							float ang <- self.location towards real_center;
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

		// Tổng số xe cần sinh ra
		int total_diff <- max(0, diff_moto) + max(0, diff_car) + max(0, diff_truck);

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
							} else {
								create truck number: 1 { location <- start_node.location; final_target <- end_node; }
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
	}

species roi_lane {
	string link_id;
	float area_m2;
	string u_node;
	string d_node;
	string direction;
	string lane_id;
	string phase_id;
	
	int current_vehicle_count <- 0;
	list<vehicle> vehicles_inside <- [];
	float occupancy_rate <- 0.0;

	// Hệ số diện tích thực tế hình chiếu bằng của từng loại xe (Dài x Rộng)
	float motobike_area <- 1.9 * 0.7; // ~1.33 m2
	float car_area <- 4.5 * 1.8;      // ~8.1 m2
	float truck_area <- 8.0 * 2.4;    // ~19.2 m2

	reflex monitor_lane_occupancy {
		vehicles_inside <- vehicle overlapping self;
		current_vehicle_count <- length(vehicles_inside);
		
		float total_vehicle_area <- 0.0;
		loop v over: vehicles_inside {
			// Sử dụng toán tử 'is' để định danh chuẩn xác lớp cha/con trong GAMA
			if (v is motobike) {
				total_vehicle_area <- total_vehicle_area + motobike_area;
			} else if (v is car) {
				total_vehicle_area <- total_vehicle_area + car_area;
			} else if (v is truck) {
				total_vehicle_area <- total_vehicle_area + truck_area;
			}
		}
		
		// Tính toán tỷ lệ phần trăm lấp đầy không gian liên tục
		if (area_m2 > 0) {
			occupancy_rate <- min(100.0, (total_vehicle_area / area_m2) * 100);
		} else {
			occupancy_rate <- 0.0;
		}
	}

	aspect default {
		// Bọc an toàn biến chuỗi từ QGIS đề phòng ô giá trị rỗng (nil)
		string current_phase <- (phase_id != nil) ? string(phase_id) : "NONE";
		
		rgb base_color <- rgb(100, 100, 100); // Mặc định: Xám đậm
		
		if (current_phase = "NONE" or current_phase = "") {
			base_color <- rgb(50, 205, 50); // Rẽ phải tự do -> Xanh lá sáng
		} else if (current_phase contains "_01" or current_phase contains "_02") {
			base_color <- rgb(0, 100, 255); // Trục chính -> Xanh dương
		} else if (current_phase contains "_03" or current_phase contains "_04") {
			base_color <- rgb(255, 140, 0); // Trục phụ -> Cam
		}
		
		// Động lực hóa màu sắc: Nếu mật độ chiếm dụng không gian > 70%, chuyển làn sang đỏ cảnh báo
		rgb dynamic_lane_color <- (occupancy_rate > 70.0) ? rgb(220, 30, 30, 160) : rgb(base_color.red, base_color.green, base_color.blue, 100);
		
		// Vẽ đa giác làn đường (Hơi dẹt trục z xuống sát mặt đường)
		draw shape color: dynamic_lane_color border: rgb(50, 50, 50, 150) width: 1 depth: 0.02;
		
		// Vẽ nhãn % văn bản hiển thị động
		if (occupancy_rate > 0.0) {
			string txt <- string(occupancy_rate, "#0.0") + "%";
			draw txt at: location + {0, 0, 2.0} color: #black font: font("Arial", 12, #bold) perspective: true;
		}
	}
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
species gis_signal_point {
	string osm_id;
}
species vehicle skills: [driving] {

	init {
		right_side_driving <- true;
		safety_distance_coeff <- 3.0;
	}
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
		vehicle_length <- 1.9;
		max_speed <- rnd(40.0, 60.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(1.9, 0.7, 1.2) color: #brown rotate: heading at: {pos.x, pos.y, 0.5};
	}

	// heatmap point display
	aspect heat_dot {
		draw circle(10) color: rgb(255, 60, 0, 80);
	}

}

species car parent: vehicle {

	init {
		vehicle_length <- 4.5;
		max_speed <- rnd(30.0, 50.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(4.5, 1.8, 1.5) color: #purple rotate: heading at: {pos.x, pos.y, 1};
	}

	// larger heatmap point for car
	aspect heat_dot {
		draw circle(13) color: rgb(255, 60, 0, 90);
	}

}

species truck parent: vehicle {

	init {
		vehicle_length <- 8.0;
		max_speed <- rnd(20.0, 40.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(8.0, 2.4, 2.8) color: #pink rotate: heading at: {pos.x, pos.y, 1.5};
	}

	// largest heatmap point for truck
	aspect heat_dot {
		draw circle(16) color: rgb(255, 60, 0, 100);
	}

}

species intersection skills: [intersection_skill] {
	bool is_green;
	bool is_traffic_signal;
	
	map<road,int> queue_per_road;
	int queue_ways1 <- 0;
	int queue_ways2 <- 0;
	
	// 4 biến đếm hàng chờ cho 4 vùng nhánh (tính từ đèn giao thông lùi về sau)
	int queue_N <- 0; // Nhánh từ Bắc tiến vào ngã tư (đi về hướng Nam)
	int queue_S <- 0; // Nhánh từ Nam tiến vào ngã tư (đi về hướng Bắc)
	int queue_E <- 0; // Nhánh từ Đông tiến vào ngã tư (đi về hướng Tây)
	int queue_W <- 0; // Nhánh từ Tây tiến vào ngã tư (đi về hướng Đông)
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

		// Kiểm tra nhanh xem đây có phải cuộc gọi từ cụm đặc biệt được gán tay không
		bool is_hand_assigned <- false;
		list<traffic_light_visual> test_lights <- traffic_light_visual where (each.my_parent = self);
		
		// Nếu đã tạo đèn visual trước đó hoặc kiểm tra gián tiếp qua trạng thái ID
		loop lg over: test_lights {
			if (lg.osm_id = "1" or lg.osm_id = "2" or lg.osm_id = "3" or lg.osm_id = "4") {
				is_hand_assigned <- true;
				break;
			}
		}

		if (is_hand_assigned) {
			// LOGIC CHO NGÃ TƯ LỆCH: Phân trục đường dựa chính xác theo cặp đèn gán tay 1-3 và 2-4
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
			// LOGIC NGUYÊN BẢN CỦA BẠN: Dành cho tất cả ngã tư tự động còn lại trong bản đồ
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

		// Tự động bù trừ và phân bổ nốt những đoạn đường lẻ chưa được map vào nhóm nào
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
		// Reset queue map (nếu cần dùng cho việc khác sau này)
		loop rd over: ways1 + ways2 { queue_per_road[rd] <- 0; }
		
		list<vehicle> all_vehicles <- (motobike as list) + (car as list) + (truck as list);
		list<vehicle> near_vehicles <- all_vehicles where (each distance_to self < 150.0);
		
		int debug_total_near <- 0;
		int debug_on_road_in <- 0;
		int debug_is_slow <- 0;
		
		int count_w1 <- 0;
		int count_w2 <- 0;
		int c_N <- 0;
		int c_S <- 0;
		int c_E <- 0;
		int c_W <- 0;
		
		loop v over: near_vehicles {
			debug_total_near <- debug_total_near + 1;
			if (v.current_road != nil) {
				road current_rd <- road(v.current_road);
				// Lấy node đích (ngã tư đến) của đoạn đường xe đang chạy
				intersection dest_node <- intersection(road_network target_of current_rd);
				
				// Kiểm tra: Nếu đích đến của đoạn đường nằm trong bán kính cụm ngã tư này
				// LƯU Ý QUAN TRỌNG: Logic này chính xác là "từ cây đèn lùi về sau" 
				// vì xe chỉ được tính khi nó ở đoạn đường CÓ HƯỚNG CHỈ VÀO ngã tư!
				if (dest_node != nil and (dest_node distance_to self < 50.0)) {
					debug_on_road_in <- debug_on_road_in + 1;
					
					// Nếu xe đang dừng hoặc di chuyển rất chậm
					if (v.speed < 5 #km/#h or v.real_speed < 5 #km/#h) {
						debug_is_slow <- debug_is_slow + 1;
						
						// Xác định góc của xe tới tâm ngã tư (từ 0 đến 360 độ)
						float ang_to_center <- float(v.location towards self.location);
						
						// Phân làm 4 vùng nhánh riêng biệt
						if (ang_to_center >= 315 or ang_to_center < 45) {
							// Góc hướng Đông (tức xe nằm ở nhánh Tây, đi về hướng Đông)
							c_W <- c_W + 1;
							count_w2 <- count_w2 + 1;
						} else if (ang_to_center >= 45 and ang_to_center < 135) {
							// Góc hướng Nam (tức xe nằm ở nhánh Bắc, đi về hướng Nam)
							c_N <- c_N + 1;
							count_w1 <- count_w1 + 1;
						} else if (ang_to_center >= 135 and ang_to_center < 225) {
							// Góc hướng Tây (tức xe nằm ở nhánh Đông, đi về hướng Tây)
							c_E <- c_E + 1;
							count_w2 <- count_w2 + 1;
						} else { 
							// 225 đến 315: Góc hướng Bắc (tức xe nằm ở nhánh Nam, đi về hướng Bắc)
							c_S <- c_S + 1;
							count_w1 <- count_w1 + 1;
						}
					}
				}
			}
		}
		
		queue_ways1 <- count_w1;
		queue_ways2 <- count_w2;
		queue_N <- c_N;
		queue_S <- c_S;
		queue_E <- c_E;
		queue_W <- c_W;
		
		// Chỉ theo dõi ngã tư 'intersection33' và chỉ in ra mỗi 10 bước
		if (name = "intersection33" and cycle mod 10 = 0) {
			write "--- Cycle " + cycle + " | " + name + " ---";
			write "Tổng xe bán kính 100m (near): " + debug_total_near;
			write "Số xe nằm trên đường đi vào ngã tư (on_road_in): " + debug_on_road_in;
			write "Số xe đang dừng/chậm trên đường đi vào (slow): " + debug_is_slow;
			if (debug_is_slow > 0) {
				write "=> CHI TIẾT 4 NHÁNH | Nhánh Bắc: " + queue_N + " | Nhánh Nam: " + queue_S + " | Nhánh Đông: " + queue_E + " | Nhánh Tây: " + queue_W;
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

species traffic_controller{
	list<intersection> my_nodes;
	float time_to_change <- 60 #s;
	float counter <- 0.0;
	bool is_green <- true;
	
	int queue_ways1 <- 0;
	int queue_ways2 <- 0;
	
	reflex run_cycle{
		counter <- counter + step;
		if(counter >= time_to_change){
			counter <- 0.0;
			ask my_nodes {
				if (is_green) { do to_red; } 
                else { do to_green; }
			}
		}
	}
}

species traffic_light_visual {
    intersection my_parent;
    road my_road; // optional variable for compatibility
    string axis;  // axis identifier
    string state <- "red"; // visual state of traffic light
	string osm_id;
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
	//parameter "Kịch bản di chuyển:" var: routing_scenario among: ["Bình thường", "Trục dọc kẹt cứng", "Đổ dồn về phía Đông"];
	output {
		display main type: 3d background: #lightskyblue axes: false {
			species road refresh: false;
			species roi_lane refresh: true;
			species motobike;
			species car;
			species truck;
			species intersection;
			species traffic_light_visual;
		}
		
		display heatmap type: 3d background: rgb(8, 12, 25) axes: false {
			// base road layer
			species road aspect: heatmap_base refresh: false;
			// overlay heat dots based on actual vehicle positions
			species motobike aspect: heat_dot;
			species car aspect: heat_dot;
			species truck aspect: heat_dot;
		}
	}
}