//#include <pcl/point_types.h>
//#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <vector>
#include "rws2019_msgs/DoTheMath.h"

#include "sound_play/SoundRequest.h"

using namespace std;

float randomizePosition()
{
  srand(7559 * time(NULL));  // set initial seed value to 5323
  return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
};

float randomizePosition2()
{
  srand(1000 * time(NULL));  // set initial seed value to 5323
  return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
};

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

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

  string last_prey;
  string last_hunter;

  boost::shared_ptr<ros::Publisher> sound_play_pub;

  boost::shared_ptr<ros::Timer> timer;

  MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in)
  {
    setTeamName(team_name_in);
    team_red = (boost::shared_ptr<Team>)new Team("red");
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");

    ros::NodeHandle n;

    vis_pub = (boost::shared_ptr<ros::Publisher>)new ros::Publisher;
    (*vis_pub) = n.advertise<visualization_msgs::Marker>("/bocas", 0);

    sound_play_pub = (boost::shared_ptr<ros::Publisher>)new ros::Publisher;

    (*sound_play_pub) = n.advertise<sound_play::SoundRequest>("/robotsound", 10);

    timer = (boost::shared_ptr<ros::Timer>)new ros::Timer;
    (*timer) = n.createTimer(ros::Duration(5), &MyPlayer::timerCallback, this);

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
    float sy = randomizePosition2();

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

    last_prey = "";
    last_hunter = "";
  }

  void printInfo(void)
  {
    // ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name);
    // ROS_INFO_STREAM("I'm hunting " << team_preys->team_name << " and escaping " << team_hunters->team_name);
  }

  std::tuple<float, float> getDistanceAndAngleToPlayer(string alvo_name)
  {
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform(player_name, alvo_name, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      // ROS_ERROR("%s", ex.what());
      ros::Duration(0.01).sleep();

      return { 1000, 0 };
    }

    float x = T0.getOrigin().x();
    float y = T0.getOrigin().y();
    float dist = sqrt(x * x + y * y);

    float ang = atan2(y, x);

    // ROS_INFO_STREAM("Dist to " << alvo_name << " is " << dist);

    return { dist, ang };
  };

  std::tuple<float, float> getDistanceAndAngleToWorld()
  {
    return getDistanceAndAngleToPlayer("world");
  }

  // TESTE
  std::tuple<float, float> getDistanceAndAngleOfAlvoToWorld(string alvo_name)
  {
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform("world", alvo_name, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      // ROS_ERROR("%s", ex.what());
      ros::Duration(0.01).sleep();

      return { 1000, 0 };
    }

    float x = T0.getOrigin().x();
    float y = T0.getOrigin().y();
    float dist = sqrt(x * x + y * y);

    float ang = atan2(y, x);

    // ROS_INFO_STREAM("Dist to " << alvo_name << " is " << dist);

    return { dist, ang };
  };

  bool Math(rws2019_msgs::DoTheMath::Request &req, rws2019_msgs::DoTheMath::Response &res)
  {
    if (req.op == "+")
    {
      res.result = req.a + req.b;
    }
    else if (req.op == "-")
    {
      res.result = req.a - req.b;
    }
    else if (req.op == "*")
    {
      res.result = req.a * req.b;
    }
    else if (req.op == "/")
    {
      res.result = req.a / req.b;
    }
    else
    {
      return false;
    }

    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.result);

    return true;
  }

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
  {
    // ROS_INFO("received a new msg");

    bool someting_change = false;

    float dist_to_cntr = std::get<0>(getDistanceAndAngleToWorld());
    float ang_to_cntr = std::get<1>(getDistanceAndAngleToWorld());

    // Step 0
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform("world", player_name, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      // ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
    }

    // Step 2

    // for each pray fin closest and follow it
    vector<double> dist_to_preys;
    vector<double> ang_to_preys;

    std::tuple<float, float> t;
    for (size_t i = 0; i < msg->red_alive.size(); i++)
    {
      t = getDistanceAndAngleToPlayer(msg->red_alive[i]);
      dist_to_preys.push_back(std::get<0>(t));
      ang_to_preys.push_back(std::get<1>(t));
    }

    vector<double> dist_to_hunter;
    vector<double> ang_to_hunter;

    for (size_t i = 0; i < msg->green_alive.size(); i++)
    {
      t = getDistanceAndAngleToPlayer(msg->green_alive[i]);
      dist_to_hunter.push_back(std::get<0>(t));
      ang_to_hunter.push_back(std::get<1>(t));
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

    int index_closest_hunter = 0;
    float dist_closest_hunter = 1000;

    for (size_t i = 0; i < dist_to_hunter.size(); i++)
    {
      if (dist_to_hunter[i] < dist_closest_hunter)
      {
        index_closest_hunter = i;
        dist_closest_hunter = dist_to_hunter[i];
      }
    }
#if 0
    if (index_closest_prey != -1)
    {
      string prey = msg->red_alive[index_closest_prey];
      if (prey != last_prey)
      {
        !someting_change;
        last_prey = prey;
      }
    }
    else if (index_closest_hunter != -1)
    {
      string hunter = msg->red_alive[index_closest_hunter];
      if (hunter != last_hunter)
      {
        !someting_change;
        last_hunter = hunter;
      }
    }
#endif

    // Step 2.5

    float dx = 100;
    float angle;
    float dx_max;
    float angle_max;

    string boca;

    if (dist_closest_hunter <= 1.2)  // prioridade 1 : fugir
    {
      // angle = ang_to_hunter[index_closest_hunter];

      t = getDistanceAndAngleOfAlvoToWorld(team_hunters->player_names[index_closest_hunter]);

      angle = std::get<1>(t);

      boca = "Vou fugir do " + team_hunters->player_names[index_closest_hunter];
    }
    else  // prioridade 2 : caçar
    {
      angle = ang_to_preys[index_closest_prey];

      boca = "Vou-te apanhar " + team_preys->player_names[index_closest_prey];
    }

    if (dist_to_cntr > 7.2)  // prioridade : nao atingir a berma
    {
      angle = ang_to_cntr + M_PI / 2;

      boca = "AI A ARENA A ACABAR!!!";
    }

    // ROS_INFO_STREAM(angle);

    dx_max = msg->cheetah;
    dx > dx_max ? dx = dx_max : dx = dx;

    angle_max = M_PI / 30;
    if (angle != 0)
    {
      fabs(angle) > fabs(angle_max) ? angle = angle_max * angle / fabs(angle) : angle = angle;
    }
    // Step 3
    tf::Transform T1;

    T1.setOrigin(tf::Vector3(dx, 0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, angle);
    T1.setRotation(q);

    // Step 4
    tf::Transform Tglobal = T0 * T1;

    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));
#if 0
    if (someting_change)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = player_name;
      marker.header.stamp = ros::Time();
      marker.ns = player_name;
      marker.id = 0;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.y = 0.5;
      // marker.scale.x = 1;
      // marker.scale.y = 0.1;
      marker.scale.z = 0.4;
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.lifetime = ros::Duration(2);
      marker.frame_locked = 1;

      marker.text = boca;

      vis_pub->publish(marker);
    }
#endif
  }
  /*
    void PCL_callback(const PointCloud::ConstPtr& msg)
    {
      printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
      BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
        printf("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    }*/

  void timerCallback(const ros::TimerEvent &event)
  {
    sound_play::SoundRequest sound_request;

    sound_request.sound = -3;
    sound_request.command = 1;
    sound_request.volume = 1.0;

    sound_request.arg = "jsantos, ola";
    sound_request.arg2 = "voice_kal_diphone";

    //sound_play_pub->publish(sound_request);
  }
};

}  // namespace jsantos_ns

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jsantos");
  ros::NodeHandle n;

  string player_name = "jsantos";
  string player_team = "blue";

  jsantos_ns::MyPlayer player(player_name, player_team);

  ros::Subscriber sub = n.subscribe("/make_a_play", 100, &jsantos_ns::MyPlayer::makeAPlayCallback, &player);

  // ros::Subscriber pcl_sub = n.subscribe<PointCloud>("/object_point_cloud", 1, &jsantos_ns::MyPlayer::PCL_callback,
  // &player);

  ros::ServiceServer service = n.advertiseService("do_the_math", &jsantos_ns::MyPlayer::Math, &player);

  ros::Rate r(20);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}