#ifndef COURSE_A_H
#define COURSE_A_H

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <string>
#include <vector>

using namespace std;

struct position3d{
	double x=0.0;
	double y=0.0;
	double z=0.0;
};

struct course{
  position3d start_position;
  position3d spawn_position;
  position3d finish_position;
  bool heading_opposite=false;
  bool heading_clock=false;
  int score=0;
};


struct map_of_courses{
	string name;
	vector<course> courses;
	ros::Duration time_limit;
	bool if_passed_map=true;
};

struct spheres_struct{
	bool if_throw=false;
	rosgraph_msgs::Clock time_counter;
	position3d reference_position;
	string direction="back";
};

vector<position3d> cubes_poses, spheres_poses;
vector<string> cubes_names, spheres_names;
vector<spheres_struct> spheres_throw_vec;

map_of_courses refracted_corridor_map, manipulator_map, rough_terrain_map, disturbance_map, stair_map, finish_map;
vector<map_of_courses> courseAB;
int current_map=0;
int current_course=0;



///////////////////////////////////////////////////////////////////////////////////////////////
void course_initilization(){
	cubes_poses.push_back({34.6, -5.6, 0.28}); cubes_poses.push_back({34.85, -5.6, 0.28});
	cubes_poses.push_back({35.1, -5.6, 0.28}); cubes_poses.push_back({35.35, -5.6, 0.28});
	cubes_poses.push_back({35.6, -5.6, 0.28});
	cubes_names.push_back("b1"); cubes_names.push_back("b2"); cubes_names.push_back("b3");
	cubes_names.push_back("b4"); cubes_names.push_back("b5");

	spheres_poses.push_back({58.75, -4, 0.3}); spheres_poses.push_back({64.75, -4, 0.3});
	spheres_poses.push_back({70.75, -4, 0.3}); spheres_poses.push_back({76.75, -4, 0.3});
	spheres_poses.push_back({82.75, -4, 0.3});
	spheres_names.push_back("ds1"); spheres_names.push_back("ds2");
	spheres_names.push_back("ds3"); spheres_names.push_back("ds4");
	spheres_names.push_back("ds5");

	refracted_corridor_map.name = "corridor ";
	manipulator_map.name = "manipulator ";
	rough_terrain_map.name = "rough terrain ";
	disturbance_map.name = "disturbance ";
	stair_map.name = "stair ";
	finish_map.name = "finish ";

	refracted_corridor_map.time_limit = ros::Duration(600.0);
	manipulator_map.time_limit = ros::Duration(600.0);
	rough_terrain_map.time_limit = ros::Duration(600.0);
	disturbance_map.time_limit = ros::Duration(900.0);
	stair_map.time_limit = ros::Duration(1200.0);
	finish_map.time_limit = ros::Duration(300.0);


	course c1; c1.score=3; c1.start_position={3.7, 0.0, 0.0}; c1.finish_position={6.35, -10.0, 0.0}; c1.spawn_position={1.35, 0.2, 0.0};
	course c2; c2.score=3; c2.start_position={8.7, -10.0, 0.0}; c2.finish_position={11.35, -3.0, 0.0}; c2.spawn_position={6.35, -9.8, 0.0};
	course c3; c3.score=3; c3.start_position={13.7, -3.0, 0.0}; c3.finish_position={16.35, -8.0, 0.0}; c3.spawn_position={11.35, -2.8, 0.0};
	course c4; c4.score=3; c4.start_position={18.7, -8.0, 0.0}; c4.finish_position={21.35, -1.0, 0.0}; c4.spawn_position={16.35, -7.8, 0.0};
	course c5; c5.score=3; c5.start_position={23.7, -1.0, 0.0}; c5.finish_position={26.35, -6.0, 0.0}; c5.spawn_position={21.35, -0.8, 0.0};
	refracted_corridor_map.courses.push_back(c1);
	refracted_corridor_map.courses.push_back(c2);
	refracted_corridor_map.courses.push_back(c3);
	refracted_corridor_map.courses.push_back(c4);
	refracted_corridor_map.courses.push_back(c5);

	course m1; m1.score=15; m1.start_position={33.55, -6.0, 0.0}; m1.finish_position={36.75, -6.0, 0.0}; m1.spawn_position={31.25, -5.8, 0.0};
	manipulator_map.courses.push_back(m1);

	course r1; r1.score=3; r1.start_position={38.6, -6.0, 0.0}; r1.finish_position={42.3, -6.0, 0.0}; r1.spawn_position={36.75, -5.8, 0.0};
	course r2; r2.score=5; r2.start_position={44.5, -6.0, 0.0}; r2.finish_position={50.5, -6.0, 0.0}; r2.spawn_position={42.5, -5.8, 0.0};
	course r3; r3.score=7; r3.start_position={52.5, -6.0, 0.0}; r3.finish_position={55.35, -6.0, 0.0}; r3.spawn_position={50.5, -5.8, 0.0};
	rough_terrain_map.courses.push_back(r1);
	rough_terrain_map.courses.push_back(r2);
	rough_terrain_map.courses.push_back(r3);

	course d1; d1.score=5; d1.start_position={57.7, -6.0, 0.0}; d1.finish_position={61.45, -6.0, 0.0}; d1.spawn_position={55.75, -5.8, 0.0};
	course d2; d2.score=5; d2.start_position={63.7, -6.0, 0.0}; d2.finish_position={67.45, -6.0, 0.0}; d2.spawn_position={61.75, -5.8, 0.0};
	course d3; d3.score=5; d3.start_position={69.7, -6.0, 0.0}; d3.finish_position={73.45, -6.0, 0.0}; d3.spawn_position={67.75, -5.8, 0.0};
	course d4; d4.score=5; d4.start_position={75.7, -6.0, 0.0}; d4.finish_position={79.45, -6.0, 0.0}; d4.spawn_position={73.75, -5.8, 0.0};
	course d5; d5.score=5; d5.start_position={81.7, -6.0, 0.0}; d5.finish_position={85.45, -6.0, 0.0}; d5.spawn_position={79.75, -5.8, 0.0};
	disturbance_map.courses.push_back(d1);
	disturbance_map.courses.push_back(d2);
	disturbance_map.courses.push_back(d3);
	disturbance_map.courses.push_back(d4);
	disturbance_map.courses.push_back(d5);
	spheres_struct sp1; sp1.reference_position=d1.start_position; sp1.reference_position.x=(d1.start_position.x+d1.finish_position.x)*0.5; sp1.direction="back";
	spheres_struct sp2; sp2.reference_position=d2.start_position; sp2.reference_position.x=(d2.start_position.x+d2.finish_position.x)*0.5; sp2.direction="left";
	spheres_struct sp3; sp3.reference_position=d3.start_position; sp3.reference_position.x=(d3.start_position.x+d3.finish_position.x)*0.5; sp3.direction="front";
	spheres_struct sp4; sp4.reference_position=d4.start_position; sp4.reference_position.x=(d4.start_position.x+d4.finish_position.x)*0.5; sp4.direction="right";
	spheres_struct sp5; sp5.reference_position=d5.start_position; sp5.reference_position.x=(d5.start_position.x+d5.finish_position.x)*0.5; sp5.direction="random";
	spheres_throw_vec.push_back(sp1);
	spheres_throw_vec.push_back(sp2);
	spheres_throw_vec.push_back(sp3);
	spheres_throw_vec.push_back(sp4);
	spheres_throw_vec.push_back(sp5);

	course s1; s1.score=15; s1.start_position={88.45, -6.0, 0.0}; s1.finish_position={91.25, -6.0, 1.5}; s1.spawn_position={86.05, -5.8, 0.0};
	course s2; s2.score=15; s2.start_position={91.3, -5.15, 1.5}; s2.finish_position={88.55, -5.15, 3.0}; s2.spawn_position={91.65, -6.0, 1.5};
	s2.heading_clock=true;
	stair_map.courses.push_back(s1);
	stair_map.courses.push_back(s2);

	course f1; f1.start_position={88.55, -5.15, 3.0}; f1.finish_position={85.0, -5.15, 3.0}; f1.spawn_position={87.7, -5.15, 3.0};
	f1.heading_opposite=true;
	finish_map.courses.push_back(f1);
	
	courseAB.push_back(refracted_corridor_map);
	courseAB.push_back(manipulator_map);
	courseAB.push_back(rough_terrain_map);
	courseAB.push_back(disturbance_map);
	courseAB.push_back(stair_map);
	courseAB.push_back(finish_map);
}


#endif