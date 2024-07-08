
#include <osm_planner/osm_parser.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <osm_planner/osm_planner.h>
#include <osm_planner/osm_localization.h>
#include <osm_planner/path_finder_algorithm/dijkstra.h>

class OsmParserNode: osm_planner::Parser{
public:

    OsmParserNode() : osm_planner::Parser(), nPrivate("~"){

        //init ros topics and services
        navGoalSub = n.subscribe("/move_base_simple/goal", 10, &OsmParserNode::navGoalCallback, this);
        startPointSub = n.subscribe("/clicked_point", 10, &OsmParserNode::startPointCallback, this);

        //publishers
        roadPathPub = n.advertise<nav_msgs::Path>("/road_network", 10);
        shortest_path_pub = n.advertise<nav_msgs::Path>("/shortest_path", 10);

        //Debug param
        double origin_lat, origin_lon;
        n.param<double>("origin_latitude", origin_lat, 31.3362180921);
        n.param<double>("origin_longitude",origin_lon, 121.00742128364);

        // Get params for map and parse
        //source of map
        std::string file = "/lio_sam/src/osm_planner/osm_example/lujia2.osm";
        nPrivate.getParam("osm_map_path", file);
        this->setNewMap(file);

        std::vector<std::string> types_of_ways;
        nPrivate.getParam("filter_of_ways",types_of_ways);
        types_of_ways = {"footway"};
        this->setTypeOfWays(types_of_ways);

        //Set the density of points
        double interpolation_max_distance;
        nPrivate.param<double>("interpolation_max_distance", interpolation_max_distance, 2.0);
        ROS_INFO("interpolation_max_distance : %f", interpolation_max_distance);
        this->setInterpolationMaxDistance(interpolation_max_distance);

        double footway_width;
        nPrivate.param<double>("footway_width", footway_width, 2);
        ROS_INFO("footway_width : %f", footway_width);
        this->parse();

        this->getCalculator()->setOrigin(origin_lat, origin_lon);
        this->publishRoadNetwork();
        // op.map = map;
        // op.initialized_ros = true;
        ROS_INFO("OsmParserNode init successfully!!!");

        map = std::make_shared<osm_planner::Parser>();
        map->setNewMap(file);
        map->setTypeOfWays(types_of_ways);
        map->setInterpolationMaxDistance(interpolation_max_distance);
        map->parse();
        map->getCalculator()->setOrigin(origin_lat, origin_lon);
        localization_source_ = std::make_shared<osm_planner::Localization>(map, "source");
        localization_target_ = std::make_shared<osm_planner::Localization>(map, "target");
        path_finder_ = std::make_shared<osm_planner::path_finder_algorithm::Dijkstra>();
    }

    void publishRoadNetwork()
    {
        this->publishRouteNetwork();
        for(auto point : this->path.poses)
        {
            // ROS_INFO("point : %f, %f ", point.pose.position.x, point.pose.position.y);
            point.header.frame_id = "map";
            roadPath.poses.push_back(point);
        }

        if (!this->path.poses.empty()) {
            roadPath.header.frame_id = "map";
            roadPath.header.stamp = ros::Time::now();
            usleep(90000);
            roadPathPub.publish(roadPath);
            ROS_INFO("Publish network successfully!!! Size : %lu", this->path.poses.size());
        } else {
            ROS_WARN("Path is empty, nothing to publish!");
        }
    }

    void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        nav_msgs::Path path;
        goalPoint.pose.position = msg->pose.position;
        goalPoint.pose.orientation = msg->pose.orientation;
        ROS_INFO("goalPoint : %f, %f", goalPoint.pose.position.x, goalPoint.pose.position.y);
        plan.clear();
        this->makePlan_(startPoint, goalPoint, plan);
    }

    void startPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        startPoint.pose.position = msg->point;
        ROS_INFO("startPoint : %f, %f", startPoint.pose.position.x, startPoint.pose.position.y);
    }

    void makePlan_(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        //Set the start pose to plan
        plan.push_back(start);

        ROS_INFO("localization of nearest point on the footway");
        //localization of nearest point on the footway
        localization_source_->setPositionFromPose(start.pose);
        ROS_INFO("111");
        this->publishPoint(start.pose.position, Parser::CURRENT_POSITION_MARKER, 1.0, start.pose.orientation);

        ROS_INFO("compute distance between start and goal");
        //compute distance between start and goal
        double dist_x = start.pose.position.x - goal.pose.position.x;
        double dist_y = start.pose.position.y - goal.pose.position.y;
        double startGoalDist = sqrt(pow(dist_x, 2.0) + pow(dist_y, 2.0));

        ROS_INFO("If distance between start and goal pose is lower as footway width then skip the planning on the osm map");
        //If distance between start and goal pose is lower as footway width then skip the planning on the osm map
        if (startGoalDist <  localization_source_->getFootwayWidth() + localization_source_->getDistanceFromWay()){
            plan.push_back(goal);
            shortest_path_.nav_path.poses.clear();
            shortest_path_.nav_path.poses.push_back(start);
            shortest_path_.nav_path.poses.push_back(goal);
            shortest_path_pub.publish(shortest_path_.nav_path);
            this->publishPoint(goal.pose.position, Parser::TARGET_POSITION_MARKER, 1.0, goal.pose.orientation);
            ROS_ERROR("The distance between start and goal pose is lower as footway width then skip the planning on the osm map.");
        }

        //set the nearest point as target and save new target point
        localization_target_->setPositionFromPose(goal.pose);

        //draw target point
        this->publishPoint(goal.pose.position, Parser::TARGET_POSITION_MARKER, 1.0, goal.pose.orientation);


       ///start planning, the Path is obtaining in global variable nav_msgs::Path path
        ROS_INFO("OSM planner: Planning trajectory...");
        ros::Time start_time = ros::Time::now();
        try {
            shortest_path_.node_path = path_finder_->findShortestPath(map->getGraphOfVertex(), localization_source_->getPositionNodeID(), localization_target_->getPositionNodeID());
            shortest_path_.nav_path = map->getPath(shortest_path_.node_path);

            ROS_INFO("OSM planner: Time of planning: %f ", (ros::Time::now() - start_time).toSec());

        } catch (osm_planner::path_finder_algorithm::PathFinderException &e) {
            if (e.getErrId() == osm_planner::path_finder_algorithm::PathFinderException::NO_PATH_FOUND) {
                ROS_ERROR("OSM planner: Make plan failed...");
                return;
            } else
                ROS_ERROR("OSM planner: Undefined error");
                return;
        }

          // Convert to geometry_msgs::PoseStamped
        for (int i=1; i< shortest_path_.nav_path.poses.size(); i++){

            geometry_msgs::PoseStamped new_goal = goal;
            new_goal.pose.position.x = shortest_path_.nav_path.poses[i].pose.position.x;
            new_goal.pose.position.y = shortest_path_.nav_path.poses[i].pose.position.y;
            new_goal.pose.orientation = shortest_path_.nav_path.poses[i].pose.orientation;
            plan.push_back(new_goal);
        }

        //add end (target) point
        shortest_path_.nav_path.poses.push_back(goal);
        shortest_path_pub.publish(shortest_path_.nav_path);
        plan.push_back(goal);

        ROS_INFO("Planning successfully!!!");
    }

private:
    struct OsmPath{
        nav_msgs::Path nav_path;
        std::vector<int> node_path;
    };
    ros::NodeHandle n, nPrivate;

    /* Subscribers */
    ros::Subscriber navGoalSub, startPointSub;

    /*Publishers*/
    ros::Publisher shortest_path_pub, roadPathPub;

    /* Services */
    ros::ServiceServer plan_service;
    std::string odom_topic_;

    //Class for localization on the map
    std::shared_ptr<osm_planner::Localization> localization_source_;
    std::shared_ptr<osm_planner::Localization> localization_target_;
    std::shared_ptr<osm_planner::path_finder_algorithm::PathFinderBase> path_finder_;

    std::shared_ptr<Parser> map;
    geometry_msgs::PoseStamped startPoint, goalPoint;

    // osm_planner::Planner op;
    std::vector<geometry_msgs::PoseStamped> plan;
    nav_msgs::Path roadPath;
    //msgs for shortest path
    OsmPath shortest_path_;

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "osm_parser_node");

    OsmParserNode osm_planner;

    ros::spin();


    return 0;
}
