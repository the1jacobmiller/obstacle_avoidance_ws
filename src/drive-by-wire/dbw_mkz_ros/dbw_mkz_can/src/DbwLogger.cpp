#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <signal.h>
#include <sys/types.h>
#include <stdio.h>
#include <pwd.h>
#include <ros/ros.h>
#include <string>
#include <time.h>
#include <unistd.h>
#include <sys/wait.h>

#include <std_msgs/String.h>

using namespace std;

bool recording;

string get_bagfile_name() {
  time_t rawtime;
  time(&rawtime);
  struct tm * timeinfo;
  timeinfo = localtime(&rawtime);
  string day = to_string(timeinfo->tm_mday);
  string month = to_string(timeinfo->tm_mon + 1);
  string year = to_string(timeinfo->tm_year + 1900);
  string date = year + "-" + month + "-" + day;
  string sec = to_string(timeinfo->tm_sec);
  string min = to_string(timeinfo->tm_min);
  string hour = to_string(timeinfo->tm_hour);
  string current_time = hour + ":" + min + ":" + sec;

  string bagfile_name = date + "(" + current_time + ").bag";
  return bagfile_name;
}

void *record(void *arg) {
  string bagfile_name = get_bagfile_name();
  char file_name[bagfile_name.length()+1];
  strcpy(file_name, bagfile_name.c_str());

  ROS_WARN_STREAM("DbwLogger - starting new bag " << file_name);

  int status;
  int pid = fork();
  if (pid > 0) { // parent process
    ROS_WARN_STREAM("DbwLogger - created new child process " << pid);
    int wait_return = waitpid(pid, &status, 0);
    if(wait_return == -1) {
      ROS_ERROR_STREAM("DbwLogger - Parent failed to wait for child node - returning\n");
      return NULL;
    }
    if(WIFEXITED(status)) { // if there is an error in the child
      int child_status = (signed char)WEXITSTATUS(status);
      if (child_status != 0) {
        ROS_ERROR_STREAM("DbwLogger - Child exited with status " << child_status);
        return NULL;
      }
    }
  } else if (pid == 0) { // child process
    char *username;
    struct passwd *pass;
    pass = getpwuid(getuid());
    username = pass->pw_name;
    string directory = "/home/" + string(username) + "/Desktop/dbw_bags";
    char dir[directory.length()+1];
    strcpy(dir, directory.c_str());
    chdir(dir);

    char *args[] = {"rosbag", "record", "-q", "-a", "--split","--duration=20", "--max-splits", "3", "-O", file_name, "__name:=DbwLoggerRecord", NULL};
    exit(execvp(args[0], args));
    exit(-1); // should never reach here
  } else {
    ROS_ERROR_STREAM("DbwLogger - Error occurred while forking");
    return NULL;
  }
}

void *stop(void *arg) {
  ROS_ERROR_STREAM("DbwLogger - Killing all rosbags");

  int status;
  int pid = fork();
  if (pid > 0) { // parent process
    ROS_WARN_STREAM("DbwLogger - created new child process " << pid);
    int wait_return = waitpid(pid, &status, 0);
    if(wait_return == -1) {
      ROS_ERROR_STREAM("DbwLogger - Parent failed to wait for child node - returning\n");
      return NULL;
    }
    if(WIFEXITED(status)) { // if there is an error in the child
      int child_status = (signed char)WEXITSTATUS(status);
      if (child_status != 0) {
        ROS_ERROR_STREAM("DbwLogger - Child exited with status " << child_status);
        return NULL;
      }
    }
  } else if (pid == 0) { // child process
    // char *args[] = {"sudo", "killall", "rosbag", NULL};
    char *args[] = {"rosnode", "kill", "/DbwLoggerRecord", NULL};
    exit(execvp(args[0], args));
    exit(-1); // should never reach here
  } else {
    ROS_ERROR_STREAM("DbwLogger - Error occurred while forking");
    return NULL;
  }

  return NULL;
}

void recvHMI(const std_msgs::String::ConstPtr &msg) {
  pthread_t thread;
  if (msg->data == "ENTER" && !recording) {
    recording = true;
    pthread_create (&thread, NULL, record, NULL);
  }
  else if (msg->data == "EXIT" && recording) {
    recording = false;
    pthread_create (&thread, NULL, stop, NULL);
  }
  pthread_detach(thread);
}

int main(int argc, char **argv) {
  // initialize ros node
  ros::init(argc, argv, "vsi_event_logger");
  ros::NodeHandle n;
  ros::Rate r(30);

  // Subscribers
  ros::Subscriber sub_hmi = n.subscribe("/hmi_command", 10, recvHMI);

  ros::MultiThreadedSpinner spinner(0); // use one thread for each CPU core
  spinner.spin();

}
