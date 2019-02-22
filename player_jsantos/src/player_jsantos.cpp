#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <vector>

using namespace std;

float randomizePosition()
{
  srand(6832 * time(NULL));  // set initial seed value to 5323
  return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
};

namespace jsantos_ns
{
class Team
{
public:
  string team_name;
  vector<string> player_names;
  ros::NodeHandle n;

  Team(string team_name_in)
  {
    team_name = team_name_in;

    n.getParam("/team_" + team_name, player_names);
  }

  void printInfo()
  {
    cout << "Team " << team_name << " has players: " << endl;

    for (size_t i = 0; i < player_names.size(); i++)
    {
      cout << player_names[i] << endl;
    }
  }

  bool playerBelongsToTeam(string player_name)
  {
    for (size_t i = 0; i < player_names.size(); i++)
    {
      if (player_names[i] == player_name)
        return true;
    }

    return false;
  }

private:
};

class Player
{
public:
  string player_name;

  Player(string player_name_in)
  {
    player_name = player_name_in;
  }

  void setTeamName(string team_name_in)
  {
    if (team_name_in == "red" || team_name_in == "green" || team_name_in == "blue")
    {
      team_name = team_name_in;
    }
    else
    {
      cout << "Cannot set team name " << team_name_in << endl;
    }
  }

  void setTeamName(int team_index)
  {
    if (team_index == 0)
      setTeamName("red");
    else if (team_index == 1)
      setTeamName("green");
    else if (team_index == 2)
      setTeamName("blue");
    else
      setTeamName("");
  }

  string getTeamName()
  {
    return team_name;
  };

private:
  string team_name;
};

class MyPlayer : public Player
{
public:
  boost::shared_ptr<Team> team_red;
  boost::shared_ptr<Team> team_green;
  boost::shared_ptr<Team> team_blue;
  boost::shared_ptr<Team> team_hunters;
  boost::shared_ptr<Team> team_mine;
  boost::shared_ptr<Team> team_preys;

  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  boost::shared_ptr<ros::Publisher> vis_pub;

  MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in)
  {
    setTeamName(team_name_in);
    team_red = (boost::shared_ptr<Team>)new Team("red");
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");

    ros::NodeHandle n;

    vis_pub = (boost::shared_ptr<ros::Publisher>)new ros::Publisher;
    (*vis_pub) = n.advertise<visualization_msgs::Marker>("player_names", 0);

    if (team_red->playerBelongsToTeam(player_name))
    {
      team_mine = team_red;
      team_preys = team_green;
      team_hunters = team_blue;
    }

    else if (team_green->playerBelongsToTeam(player_name))
    {
      team_mine = team_green;
      team_preys = team_blue;
      team_hunters = team_red;
    }

    else if (team_blue->playerBelongsToTeam(player_name))
    {
      team_mine = team_blue;
      team_preys = team_red;
      team_hunters = team_green;
    }
    else
    {
      cout << "Something wrong in team param" << endl;
    }

    setTeamName(team_mine->team_name);
    tf::Transform T1;

    float sx = randomizePosition();
    float sy = randomizePosition();

    T1.setOrigin(tf::Vector3(sx, sy, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, -M_PI);
    T1.setRotation(q);

    // Step 4
    tf::Transform Tglobal = T1;

    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));
    ros::Duration(0.1).sleep();
    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));

    printInfo();
  }

  void printInfo(void)
  {
    ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name);
    ROS_INFO_STREAM("I'm hunting " << team_preys->team_name << " and escaping " << team_hunters->team_name);
  }

  std::tuple<float, float> getDistanceAndAngleToPlayer(string alvo_name)
  {
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform(alvo_name, player_name, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.01).sleep();

      return { 1000, 0 };
    }

    float x = T0.getOrigin().x();
    float y = T0.getOrigin().y();
    float dist = sqrt(x * x + y * y);

    float ang = atan2(y, x);

    ROS_INFO_STREAM("Dist to " << alvo_name << " is " << dist);

    return { dist, ang };
  };

  float getDistanceToArenaCenter()
  {
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform("world", player_name, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.01).sleep();

      return 1000;
    }

    float x = T0.getOrigin().x();
    float y = T0.getOrigin().y();
    float dist = sqrt(x * x + y * y);

    ROS_INFO("Dist to center %f", dist);

    return dist;
  };

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
  {
    ROS_INFO("received a new msg");

    // Step 0
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform("world", player_name, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
    }

    // Step 2

    // for each pray fin closest and follow it
    vector<double> dist_to_preys;
    vector<double> ang_to_preys;

    for (size_t i = 0; i < team_preys->player_names.size(); i++)
    {
      std::tuple <float, float> t =getDistanceAndAngleToPlayer(team_preys->player_names[i]);
      dist_to_preys.push_back(std::get<0>(t));
      ang_to_preys.push_back(std::get<1>(t));
      
    }

    int index_closest_prey = 0;
    float dist_closest_prey = 1000;
    for (size_t i = 0; i < dist_to_preys.size(); i++)
    {
      if (dist_to_preys[i] < dist_closest_prey)
      {
        index_closest_prey = i;
        dist_closest_prey = dist_to_preys[i];
      }
    }

    float dx = 10;
    float angle = ang_to_preys[index_closest_prey];

    // Step 2.5

    float dx_max = msg->cheetah;
    dx > dx_max ? dx = dx_max : dx = dx;

    float angle_max = M_PI / 30;
    fabs(angle) > fabs(angle_max) ? angle = angle_max * angle / fabs(angle) : angle = angle;

    // Step 3
    tf::Transform T1;

    T1.setOrigin(tf::Vector3(dx, 0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, angle);
    T1.setRotation(q);

    // Step 4
    tf::Transform Tglobal = T0 * T1;

    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));

    visualization_msgs::Marker marker;
    marker.header.frame_id = player_name;
    marker.header.stamp = ros::Time();
    marker.ns = player_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    // marker.scale.x = 1;
    // marker.scale.y = 0.1;
    marker.scale.z = 0.6;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    // only if using a MESH_RESOURCE marker type:
    marker.text = player_name;
    vis_pub->publish(marker);
  }
};

}  // namespace jsantos_ns

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jsantos");
  ros::NodeHandle n;

  string player_name = "jsantos";
  string player_team = "red";

  jsantos_ns::MyPlayer player(player_name, player_team);

  ros::Subscriber sub = n.subscribe("/make_a_play", 100, &jsantos_ns::MyPlayer::makeAPlayCallback, &player);

  ros::Rate r(20);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}