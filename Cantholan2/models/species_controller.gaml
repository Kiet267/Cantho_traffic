/**
 * species_controller.gaml
 * Chua cac species lien quan den he thong dieu khien den giao thong va thuat toan CBMP:
 *   - gis_signal_point: tac tu trung gian load diem den hieu GIS
 *   - traffic_light_visual: hien thi den giao thong 3D
 *   - traffic_controller: bo dieu khien CBMP / Fixed-time
 * Copy nguyen van tu ctt.gaml - khong chinh sua gi ca
 */
model Traffic

// obj for gis_signal_point - tac tu trung gian load du lieu diem den hieu GIS
species gis_signal_point {
	string osm_id; // obj for signal id - ma dinh danh tu shapefile (dung de gan tay)
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
		// Hệ số năng lực thông hành Clm = 1.0 cho tất cả
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
		// Kiem tra: g1+g2 phai xap xi cycle_duration - lost_time = 116s
		// Kiem tra: g1 va g2 phai >= min_green = 10s
		// Kiem tra: neu gamma1 > gamma2 thi g1 > g2 (pha dong xe duoc xanh nhieu hon)
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
					// (cuoi pha 1 = bat dau xem xet phan bo lai cho chu ky tiep)
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
