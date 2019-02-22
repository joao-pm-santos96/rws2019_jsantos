#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <vector>

using namespace std;

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
  tf::Transform transform;

  MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in)
  {
    setTeamName(team_name_in);
    team_red = (boost::shared_ptr<Team>)new Team("red");
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");

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

    printInfo();
  }

  void printInfo(void)
  {
    ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name);
    ROS_INFO_STREAM("I'm hunting " << team_preys->team_name << " and escaping " << team_hunters->team_name);
  }

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
  {
    ROS_INFO("received a new msg");

    // publicar transformacao

    tf::Transform transform1;

    transform1.setOrigin(tf::Vector3(-5, 5, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 5);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", player_name));
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

  while (ros::ok())
  {
    ros::Duration(1).sleep();
    player.printInfo();

    ros::spinOnce();
  }
}