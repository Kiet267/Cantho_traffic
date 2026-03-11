/**
* Name: NewModel
* Based on the internal empty template. 
* Author: smth
* Tags: 
*/


model Traffic

/* Insert your model definition here */

global{
	file map_osm_file <- osm_file("../includes/map.osm");
	geometry shape <- envelope(map_osm_file);
	graph road_network;
	float step <- 0.036 #s;
	string scenario <- "High";
	
	init{
		write "read data";
		
		create osm_agent from: map_osm_file; //transfer raw data to agent
		
		//filter roads
		list<geometry> roads <- [];
		list<geometry> signals_geom <- [];
		ask osm_agent{
			if(shape.attributes["highway"] = "traffic_signals"){
				signals_geom << shape;
			}
			if(shape.attributes contains_key "highway" and not (string(shape.attributes["highway"]) contains "_link")){
				roads << shape;
			}
		}
		
		//create road
		list<geometry> fixed_road <- clean_network(roads,3.0,true,true);
		create road from: fixed_road;
		
		//create graph
		graph temp_graph <- as_edge_graph(road);
		loop v over: temp_graph.vertices{
			create intersection with: [shape::point(v)]{
				is_traffic_signal <- false;
			}
		}
		
		road_network <- as_driving_graph(road, intersection);
		loop sg over: signals_geom {
			intersection target_node <- (intersection ) closest_to sg;
			if (target_node != nil) {
				ask target_node {
					is_traffic_signal <- true;
					do compute_crossing;
					if (flip(0.5)) { do to_green; } else { do to_red; }
				}
			}
		}
		
		
		ask osm_agent{
			if(shape.attributes contains_key "building"){
				create building {
					shape <- myself.shape;
				}
			}
		}
		
		//free osm_agent
		ask osm_agent{do die;}
		
		int nb_moto <- 0;
		int nb_car <- 0;
		int nb_truck <- 0;
		write "done";
		if(scenario = "High"){
			nb_moto <- 200;
			nb_car <- 100;
			nb_truck <- 30;
		}else if(scenario = "Mid"){
			nb_moto <- 100;
			nb_car <- 50;
			nb_truck <- 15;
		}else{
			nb_moto <- 40;
			nb_car <- 15;
			nb_truck <- 5;
		}
		
		create motobike number: nb_moto{
			location <- any_location_in(one_of(road));
		}
		create car number: nb_car{
			location <- any_location_in(one_of(road));
		}
		create truck number: nb_truck{
			location <- any_location_in(one_of(road));
		}
	}
}

// ---- species
species osm_agent {}

species road skills: [road_skill]{
	int lanes <-3;
	int num_lanes <- 3;
	float width <- 9.0;
	aspect default{
		draw shape color: #black;
	}
}

species building{
	aspect default{
		draw shape color: #grey;
	}
}

species vehicle skills: [driving]{	
	init{
		right_side_driving <- true;
		safety_distance_coeff <- 3.0;
	}
	//find road
	reflex move {	
		//run random
		if(current_path = nil or final_target = nil){
			final_target <- one_of(intersection);
			if(final_target != nil){
				do compute_path graph: road_network target: final_target;
			}
		}
		//if end of road -> relocate 
		if(current_path = nil){
			location <- any_location_in(one_of(road));
			final_target <- nil;
		}else{
			do drive;
		}
	}
	point compute_position{
		if (current_road != nil) {
			float road_width <- road(current_road).width;
			int n_lanes <- road(current_road).num_lanes;
			float lane_w <- road_width / n_lanes;
			
			//calculate pos for the vehicle(shift from the center)
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

species motobike parent: vehicle{
	init{
		vehicle_length <- 2.0;
		max_speed <- rnd(40.0, 60.0) #km / #h;
		speed <- max_speed;
	}
	aspect default{
		point pos <- compute_position();
		draw box(2, 1, 1) color: #green rotate: heading at: {pos.x, pos.y, 0.5};
	}
}

species car parent: vehicle{
	init{
		vehicle_length <- 4.0;
		max_speed <- rnd(30.0, 50.0) #km / #h;
		speed <- max_speed;
	}
	aspect default{
		point pos <- compute_position();
		draw box(4,2,2) color: #red rotate: heading at: {pos.x, pos.y, 1};
	}
}

species truck parent: vehicle{
	init{
		vehicle_length <- 6.0;
		max_speed <- rnd(20.0, 40.0) #km / #h;
		speed <- max_speed;
	}
	aspect default{
		point pos <- compute_position();
		draw box(6,3,3) color: #blue rotate: heading at: {pos.x, pos.y, 1.5};
	}
}

species intersection skills: [intersection_skill]{
	bool is_green;
	bool is_traffic_signal;
	float time_to_change <- 60 #s;
	float counter <- rnd(time_to_change);
	list<road> ways1 <- [];
	list<road> ways2 <- [];
	rgb color_fire;

	//caculate lane for intersection
	action compute_crossing{
		ways1 <- [];
		ways2 <- [];
		if(length(roads_in) >= 1){
			road rd0 <- road(roads_in[0]);
			list<point> pts <- rd0.shape.points;
			float ref_angle <- last(pts) direction_to rd0.location;
			loop rd over: roads_in{
				list<point> pts2 <- road(rd).shape.points;
				float ang <- last(pts2) direction_to rd.location;
				float diff <- abs(ang - ref_angle);
				if((diff > 45 and diff < 135) or (diff > 225 and diff < 315)){
					ways2 << road(rd);
				}
			}
			loop rd over: roads_in{
				if not(rd in ways2){
					ways1 << road(rd);
				}
			}
		}
	}
	
	action to_green{	
		if (length(stop) = 0) { stop << []; }
		stop[0] <- ways2;
		color_fire <- #green;
		is_green <- true;
	}
	
	action to_red{
		if (length(stop) = 0) { stop << []; }
		stop[0] <- ways1;
		color_fire <- #red;
		is_green <- false;
	}
	
	//counter to change color of traffic_light
	reflex dynamic_node when: is_traffic_signal{
		counter <- counter + step;
		if(counter >= time_to_change){
			counter <- 0.0;
			if(is_green) {
				do to_red;
			}else{
				do to_green;
			}
		}
	}
	
	aspect default {
		if (is_traffic_signal) {
			draw sphere(3) color: (is_green ? #green : #red) at: {location.x, location.y, 5};
			draw cylinder(0.5, 5) color: #black at: {location.x, location.y, 0};
			draw circle(1) color: color_fire;
		} else {
			//draw circle(1) color: color;
		}

	}
}

experiment test type: gui {
	parameter "Choose scenario:" var: scenario among:[
		"High","Mid","Low"
	];
	
	output{
		display test type: 3d background: #lightskyblue axes: false{
			species road refresh: false;
			species building refresh: false;
			species motobike;
			species car;
			species truck;
			species intersection;
		}
	}
}