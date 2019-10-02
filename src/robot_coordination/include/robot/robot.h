#include <list>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <robot/waypoint.h>
#include <robot_coordination/AddPath.h>
#include <robot_coordination/StartMovement.h>
#include <robot_coordination/StopMovement.h>

/** Control a robot with a follow-the-carrot strategy
 *
 * Plan is loaded directly with setPath() or through a service call of type AddPath.
 * Movement is started with a call to startExecutePath() or throught a service call of type StartMovement.
 * Movement is stopped with a call to stopExecutePath() or throught a service call of type SoptMovement.
 *
 * Not thread-safe.
 */
class Robot {
  public:

    Robot();

    /** Callback function to process odometry messages.
     *  This function is called every time the nav_msgs/Odometry message is received on odom topic.
     */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /** Service callback functions to process service request
     */
    bool addPathService(robot_coordination::AddPath::Request& req, robot_coordination::AddPath::Response& res);
    bool startMovementService(robot_coordination::StartMovement::Request& req, robot_coordination::StartMovement::Response& res);
    bool stopMovementService(robot_coordination::StopMovement::Request& req, robot_coordination::StopMovement::Response& res);

    /** Set the path to follow by the robot.
     *  This command rewrite the actual planned path of the robot with the Path class.
     */
    void setPath(const std::list<Waypoint>& path);

    /** The robot starts to move along the planned path.
     * The robot moves along the set of way-points using the "follow-the-carrot" control law.
     * As soon as the motion is finished, the state of the robot is changed to Waiting.
     */
    void startExecutePath();
    /** Stop the plan execution.
     * The robot stops at the spot and the state of the robot is changed to Waiting.
     */
    void stopExecutePath();

    /** Set the publisher created outside of the class*/
    void setCommandPublisher(ros::Publisher* pub);

    /** Create the publisher inside of the class*/
    void createCommandPublisher(ros::NodeHandle& nh);

  private:
    /** Implementation of the follow-the-carrot control law.
     * This function is to be called in the odometry callback function.
     *
     * @return true when robot finishes the plan, false otherwise.
     * */
    bool followTheCarrot();

    ros::Publisher* command_publisher_;

    ros::Time execute_start_time_;

    std::list<Waypoint> plan_;

    geometry_msgs::Pose robot_pose_;

    double threshold_;

    bool execute_;
};
