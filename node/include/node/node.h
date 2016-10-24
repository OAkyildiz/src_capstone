/*
 * node.h
 *
 *  Created on: Oct 24, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#ifndef INCLUDE_NODE_H_
#define INCLUDE_NODE_H_

#include <string.h>

#include <ros/ros.h>
#include <ros/console.h>

class Node {
public:
	Node(int argc, char** argv);
	virtual ~Node();

	bool init(const std::string &name = "node");
	bool init(const std::string &name, const std::string &master_url, const std::string &host_url);
	void run();


	/* Members */
	ros::NodeHandle* nh_;

	/* Getters & Setters */
	const ros::NodeHandle* get_nh() const {
		return nh_;
	}
protected:
	ros::NodeHandle* pnh_;

private:
	/* Members */
	int init_argc;
	char** init_argv;

	ros::Subscriber* sub_list[];
	ros::Publisher* pub_list[];

	bool setup();
	/* member functions to customize node*/
	virtual void operation(){}
	virtual void setup_params(){}

	/*hash map of values*/
	//HashMap<std::string, double> params;
	//static double RATE = 60;
	double RATE;
};

#endif /* INCLUDE_NODE_H_ */
