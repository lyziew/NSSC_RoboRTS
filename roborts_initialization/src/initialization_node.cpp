#include "ros/ros.h"
#include "initialization/initialization.h"
#include <cstdio>

#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "initialization_node");

	ros::NodeHandle n;

	ros::Rate r(10);

	initialization l;

	ros::Time t = ros::Time::now();
	ros::Duration s;

	vector<match2> matches;

	float threshold = 0.9;

	bool is_match;
	while(ros::ok())
	{
		s=ros::Time::now()-t;
			
			if(s.toSec()>1)
			{

				matches = l.basitAlignment(threshold);

				int best = 0;
				float best_value = 0;

				for (int i = 0; i<matches.size(); i++)
				{
					float val = l.compareArtificialAndRealSensorData (matches[i].x, matches[i].y, matches[i].yaw);

					ROS_INFO("match: %f %f %f", matches[i].x, matches[i].y, matches[i].yaw);

					if(val > best_value)
					{
						best_value = val;
						best = i;
					} 
						
				}

				if(matches.size() > 0)
				{
					if(matches.size() > 150) threshold += 0.04;
					is_match = l.compareUwbMatch(matches[best].x,matches[best].y);
					if(!is_match){
						matches[best].x = 8-matches[best].x;
						matches[best].y = 5-matches[best].y;
						matches[best].yaw = matches[best].yaw - 3.14;
					}
					
					l.publishPose(matches[best].x, matches[best].y, matches[best].yaw);
					ROS_INFO("pose published: %f %f %f", matches[best].x, matches[best].y, matches[best].yaw);

					

					ROS_INFO("%f %f", matches[best].score, l.compareArtificialAndRealSensorData (matches[best].x, matches[best].y, matches[best].yaw));



				}
				else 
					{
						ROS_INFO("%.2f no match found aybalam", threshold);
						threshold -= 0.02;
					}

			
				t=ros::Time::now();
												
			}
		
		

		l.pub();

		r.sleep();

		ros::spinOnce();


	}
	


}