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
	
	init{
		write "read data";
		
		create osm_agent from: map_osm_file; //transfer raw data to agent
		
		//filter roads
		list<geometry> roads <- [];
		ask osm_agent{
			if(shape.attributes contains_key "highway"){
				roads << shape;
			}
		}
		
		//create road
		list<geometry> fixed_road <- clean_network(roads,3.0,true,true);
		create road from: fixed_road;
		
		//create graph
		graph temp_graph <- as_edge_graph(road);
		
		loop v over: temp_graph.vertices{
			create traffic_light with: [shape::point(v)];
		}
		
		road_network <- as_driving_graph(road, traffic_light);
		
		ask traffic_light{
			if(length(roads_in)>=2){
				is_traffic_signal <- true;
				do compute_crossing;
				stop << [];
				if(flip(0.5)){
					do to_green;
				}else{
					do to_red;
				}
			}else{
				is_traffic_signal <- false;
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
		
		write "done";
		create motobike number: 100{
			location <- any_location_in(one_of(road));
		}
		create car number: 50{
			location <- any_location_in(one_of(road));
		}
		create truck number: 20{
			location <- any_location_in(one_of(road));
		}
	}
}

// ---- species
species osm_agent {}

species road skills: [road_skill]{
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
	reflex pick_target when: final_target = nil{
		final_target <- one_of(traffic_light);
		do compute_path graph: road_network target: final_target;
	}
	
	reflex move when: final_target != nil{	
		do drive;
		if(current_path = nil){
			final_target <- nil;
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
		draw box(2, 1, 1) color: #green rotate: heading;
	}
}

species car parent: vehicle{
	init{
		vehicle_length <- 4.0;
		max_speed <- rnd(30.0, 50.0) #km / #h;
		speed <- max_speed;
	}
	aspect default{
		draw box(4,2,2) color: #red rotate: heading;
	}
}

species truck parent: vehicle{
	init{
		vehicle_length <- 6.0;
		max_speed <- rnd(20.0, 40.0) #km / #h;
		speed <- max_speed;
	}
	aspect default{
		draw box(6,3,3) color: #blue rotate: heading;
	}
}

species traffic_light skills: [intersection_skill]{
	bool is_green;
	bool is_traffic_signal;
	float time_to_change <- 60 #s;
	float counter <- rnd(time_to_change);
	list<road> ways1;
	list<road> ways2;
	rgb color_fire;
	
	
	//caculate lane for intersection
	action compute_crossing{
		if(length(roads_in) >= 2){
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
		stop[0] <- ways2;
		color_fire <- #green;
		is_green <- true;
	}
	
	action to_red{
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
	
	aspect base {
		draw sphere(3) color: (is_green ? #green : #red) at: {location.x, location.y, 5};
		draw cylinder(0.5, 5) color: #black at: {location.x, location.y, 0};
		if (is_traffic_signal) {
			draw circle(1) color: color_fire;
		} else {
			//draw circle(1) color: color;
		}

	}
}

experiment test type: gui {
	
	output{
		display test type: 3d background: #lightskyblue axes: false{
			species road refresh: false;
			species building refresh: false;
			species motobike;
			species car;
			species truck;
			species traffic_light;
		}
	}
}