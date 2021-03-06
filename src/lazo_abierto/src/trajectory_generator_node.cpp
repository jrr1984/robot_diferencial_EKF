#include <ros/ros.h>
#include <robmovil_msgs/Trajectory.h>
#include <robmovil_msgs/TrajectoryPoint.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <XmlRpcValue.h>

#include <vector>

// Forward declarations
void build_sin_trajectory(double, double, double, double, robmovil_msgs::Trajectory&, nav_msgs::Path&);
void build_spline_trajectory(double, std::vector<std::vector<double>>&, robmovil_msgs::Trajectory&, nav_msgs::Path&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  ros::Publisher trajectory_publisher = nh.advertise<robmovil_msgs::Trajectory>("/robot/trajectory", 1, true);
  
  // Path descripto en poses para visualizacion en RViz
  ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("/ground_truth/target_path", 1, true);

  robmovil_msgs::Trajectory trajectory_msg;
  nav_msgs::Path path_msg;

  trajectory_msg.header.seq = 0;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.header.frame_id = "odom";

  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "odom";
  
  // Parseo del tipo de trayectoria y parametros
  std::string trajectory_type;
  nhp.param<std::string>("trajectory_type", trajectory_type, "sin");
  
  if(trajectory_type == "sin")
  {
    double stepping;
    double total_time;
    double amplitude;
    double cycles;
    
    nhp.param<double>("stepping", stepping, 0.1);
    nhp.param<double>("total_time", total_time, 20); // 20: da masomenos, 50: lo sigue muy bien el pioneer
    nhp.param<double>("amplitude", amplitude, 1);
    nhp.param<double>("cycles", cycles, 1);
    
    build_sin_trajectory(stepping, total_time, amplitude, cycles, trajectory_msg, path_msg);
  
  }else if(trajectory_type == "spline")
  {
    
    XmlRpc::XmlRpcValue spline_waypoints_data;
    nhp.param("spline_waypoints", spline_waypoints_data, spline_waypoints_data);
    
    if(spline_waypoints_data.size() == 0 || spline_waypoints_data.size() % 4 != 0){
      ROS_ERROR("Trajectory waypoints data is not multiple of 4, it expects: time, position_x, position_y, orientation, velocity_x, velocity_y, velocity_orientation");
      return 1;
    }
    
    std::vector<std::vector<double>> spline_waypoints;
    for(int i = 0; i < spline_waypoints_data.size(); i+=4)
    {
      std::vector<double> wpoints;
      for(int j = i; j < i+4; j++)
      {
        double value = static_cast<double>(spline_waypoints_data[j]);
        wpoints.push_back(value);
      }
      
      spline_waypoints.push_back(wpoints);
    }
    
    double stepping;
    nhp.param<double>("stepping", stepping, 0.1);
    
    build_spline_trajectory(stepping, spline_waypoints, trajectory_msg, path_msg);
    
  }

  trajectory_publisher.publish( trajectory_msg );
  path_publisher.publish( path_msg );

  ros::spin();

  return 0;
}

void build_sin_trajectory(double stepping, double total_time, double amplitude, double cycles, robmovil_msgs::Trajectory& trajectory_msg, nav_msgs::Path& path_msg)
{
  // atan2(vy(0), vx(0)) = orientacion inicial
  double initial_orientation = atan2( amplitude * (cycles * 2*M_PI * 1/total_time), cycles * 2*M_PI * 1/total_time );
  
  for (double t = 0; t <= total_time; t = t + stepping)
  {
    // X se extiende lo suficiente para dar varias vueltas en el tiempo determinado
    double x = cycles * 2*M_PI * t * 1/total_time;
    // Y funcion seno con determinada amplitud
    double y = amplitude * sin( x );

    // derivadas primeras
    double vx = cycles * 2*M_PI * 1/total_time;
    double vy = amplitude * cos(x) * vx;

    // derivadas segundas
    double vvx = 0;
    double vvy = amplitude * (-sin(x) * vx * vx + cos(x) * vvx);
    
    /* dado que la funcion esta construida pensada con Y "hacia arriba", X "hacia derecha" 
     * y la orientacion inicial puede no ser 0 -> entonces aplicamos una rotacion de manera de 
     * alinear la primera orientacion al eje X del robot y todo vector direccion de manera acorde */
    double x_rot = cos(-initial_orientation) * x + -sin(-initial_orientation) * y;
    double y_rot = sin(-initial_orientation) * x + cos(-initial_orientation) * y;
    double vx_rot = cos(-initial_orientation) * vx + -sin(-initial_orientation) * vy;
    double vy_rot = sin(-initial_orientation) * vx + cos(-initial_orientation) * vy;
    double vvx_rot = cos(-initial_orientation) * vvx + -sin(-initial_orientation) * vvy;
    double vvy_rot = sin(-initial_orientation) * vvx + cos(-initial_orientation) * vvy;
    
    x = x_rot; y = y_rot; vx = vx_rot; vy = vy_rot; vvx = vvx_rot; vvy = vvy_rot;
    
    // calculo del angulo en cada momento y la derivada del angulo
    double a = atan2( vy, vx );
    double va = (vvy*vx-vvx*vy)/(vx*vx+vy*vy);

    // se crean los waypoints de la trajectoria
    robmovil_msgs::TrajectoryPoint point_msg;

    point_msg.time_from_start = ros::Duration( t );

    point_msg.transform.translation.x = x;
    point_msg.transform.translation.y = y;
    point_msg.transform.translation.z = 0;

    point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( a );

    point_msg.velocity.linear.x = vx;
    point_msg.velocity.linear.y = vy;
    point_msg.velocity.linear.z = 0;

    point_msg.velocity.angular.x = 0;
    point_msg.velocity.angular.y = 0;
    point_msg.velocity.angular.z = va;

    trajectory_msg.points.push_back( point_msg );
    
    geometry_msgs::PoseStamped stamped_pose_msg;
    
    stamped_pose_msg.header.stamp = path_msg.header.stamp;
    stamped_pose_msg.header.frame_id = path_msg.header.frame_id;
    
    stamped_pose_msg.pose.position.x = x;
    stamped_pose_msg.pose.position.y = y;
    stamped_pose_msg.pose.position.z = 0;
    
    stamped_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(a);
    
    path_msg.poses.push_back(stamped_pose_msg);
  }
}

void build_spline_trajectory(double stepping, std::vector<std::vector<double>>& wpoints, robmovil_msgs::Trajectory& trajectory_msg, nav_msgs::Path& path_msg)
{
	
	// number of points in the trajectory
	int n_total_points = wpoints.size();
	std::cout << n_total_points << std::endl; 
  
  for(int n_point= 0; n_point < n_total_points-1; n_point++ ){
  
    double initial_time = wpoints [n_point][0];
    double final_time = wpoints [n_point+1][0];
    double delta_time = final_time - initial_time;
    
    double xa = wpoints [n_point][1];
    double ya = wpoints [n_point][2];
    double thetaa = wpoints [n_point][3];
    double xb = wpoints [n_point + 1][1];
    double yb = wpoints [n_point + 1][2];
    double thetab = wpoints [n_point + 1][3];
    
    /*std::cout << "xa " << xa << " ya " << ya << " thetaa " << thetaa  << std::endl;
    std::cout << "xb " << xb << " yb " << yb << " thetab " << thetab  << std::endl;
    std::cout << "tin " << initial_time << " tfin " << final_time << std::endl;
*/
    // curvature parameters
    double n1 = 5;
    double n2 = 5;
    
    // polynomial parameters
    double a0 = xa;
    double a1 = n1 * cos(thetaa);
    double a2 = 3 * (xb - xa) - 2 * n1 * cos(thetaa) - n2 * cos(thetab);
    double a3 = -2 * (xb - xa) + n1 * cos(thetaa) + n2 * cos(thetab);
    double b0 = ya;
    double b1 = n1 * sin(thetaa);
    double b2 = 3 * (yb - ya) - 2 * n1 * sin(thetaa) - n2 * sin(thetab);
    double b3 = -2 * (yb - ya) + n1 * sin(thetaa) + n2 * sin(thetab);
    
    //std::cout << "a0 " << a0 << " a1 " << a1 << " a2 " << a2 << " a3 " << a3 << std::endl;
    //std::cout << "b0 " << b0 << " b1 " << b1 << " b2 " << b2 << " b3 " << b3 << std::endl;

    for (double t = initial_time; t <= final_time; t = t + stepping)
    {
      //
      double t_offset = t-initial_time;
      std::cout << "total_time " << delta_time << std::endl;
      std::cout << "t " << t << std::endl;
      // X se extiende lo suficiente para dar varias vueltas en el tiempo determinado
      double x = a0 + a1 * t_offset / delta_time + a2 * t_offset * t_offset / (delta_time * delta_time) + a3 * t_offset * t_offset * t_offset / (delta_time * delta_time * delta_time);
      double y = b0 + b1 * t_offset / delta_time + b2 * t_offset * t_offset / (delta_time * delta_time) + b3 * t_offset * t_offset * t_offset / (delta_time * delta_time * delta_time);

      std::cout << "x " << x << std::endl; 
      //std::cout << "y " << y << std::endl; 
      
      // derivadas primeras
      double vx = a1 / delta_time + 2. * a2 * t_offset / (delta_time * delta_time) + 3. * a3 * t_offset * t_offset / (delta_time * delta_time * delta_time);
      double vy = b1 / delta_time + 2. * b2 * t_offset / (delta_time * delta_time) + 3. * b3 * t_offset * t_offset / (delta_time * delta_time * delta_time);

      std::cout << "vx " << vx << std::endl; 
      //std::cout << "vy " << vy << std::endl; 
      
      // derivadas segundas
      double vvx = 2. * a2 / (delta_time * delta_time) + 6. * a3 * t_offset / (delta_time * delta_time * delta_time);
      double vvy = 2. * b2 / (delta_time * delta_time) + 6. * b3 * t_offset / (delta_time * delta_time * delta_time);
      
      // calculo del angulo en cada momento y la derivada del angulo
      double a = atan2( vy, vx );
      double va = (vvy*vx-vvx*vy)/(vx*vx+vy*vy);
      
      // se crean los waypoints de la trajectoria
      robmovil_msgs::TrajectoryPoint point_msg;

      point_msg.time_from_start = ros::Duration( t );

      point_msg.transform.translation.x = x;
      point_msg.transform.translation.y = y;
      point_msg.transform.translation.z = 0;

      point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( a );

      point_msg.velocity.linear.x = vx;
      point_msg.velocity.linear.y = vy;
      point_msg.velocity.linear.z = 0;
      
      //std::cout << "vx" << vx << std::endl; 
      //std::cout << "vy" << vy << std::endl; 
      //std::cout << "step" << stepping << std::endl; 
      //std::cout << "total_time" << total_time << std::endl; 
        
      point_msg.velocity.angular.x = 0;
      point_msg.velocity.angular.y = 0;
      point_msg.velocity.angular.z = va;

      trajectory_msg.points.push_back( point_msg );
      
      // Construccion de la trayectoria como un nav_msgs::Path para su visualizacion en RViz
      geometry_msgs::PoseStamped stamped_pose_msg;
      
      stamped_pose_msg.header.stamp = path_msg.header.stamp;
      stamped_pose_msg.header.frame_id = path_msg.header.frame_id;
      
      stamped_pose_msg.pose.position.x = x;
      stamped_pose_msg.pose.position.y = y;
      stamped_pose_msg.pose.position.z = 0;
      
      stamped_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw( a );
      
      path_msg.poses.push_back(stamped_pose_msg);  
    }
  }

}
