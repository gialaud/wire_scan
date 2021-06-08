#include "wire_scan.h"

using namespace std;
using namespace TooN;
using namespace sun;
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

/* Global Variables */
Vector<6> jointPosition_Robot;
Vector<6> jointPosition_Robot_cmd;
Vector<6> jointPosition_DH;
Vector<6> qDH_k;
Vector<6> error; //<-- l'errore deve essere inizializzato diverso da 0
UnitQuaternion quat;
UnitQuaternion oldQ;
Vector<3> xdd, xd, x, x0, w; // Desired Values
Vector<3> xe; // Current ee position
Vector<3> xe_first; // Initial ee position
double hz = 100; /* Hz */

UR5e UR5e_robot; /* Robot obj */

/* Sensor Data */
uint8_t proximity_data = 255;
bool proximity_new_data_available = false;

/* Callback for Proximity Data */
void proximityCallback(const read_sensor::proximity_sensor_data::ConstPtr& msg)
{
  proximity_data = msg->proximity_sensor_data[0];
  proximity_new_data_available = true;
}

/* Callback for Robot Joint States Data */
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  jointPosition_Robot[0] = msg->position[2];
  jointPosition_Robot[1] = msg->position[1];
  jointPosition_Robot[2] = msg->position[0];
  jointPosition_Robot[3] = msg->position[3];
  jointPosition_Robot[4] = msg->position[4];
  jointPosition_Robot[5] = msg->position[5];
  
  jointPosition_DH = UR5e_robot.joints_Robot2DH(jointPosition_Robot);
  
  xe = transl( UR5e_robot.fkine(jointPosition_DH) );
}

int main(int argc, char* argv[])
{  
  /* Local variables */
  float deltaX; /* Delta X w.r.t. the current ee position (base frame) */
  float deltaY; /* Delta Y w.r.t. the current ee position (base frame) */
  float deltaZ; /* Delta Z w.r.t. the current ee position on first map point */
  float v;      /* Scanning speed */
  float deltaScan;     /* DeltaScan */
  bool scan_x = true;  /* Scanning direction: if 1 scan along x, if 0 scan along y w.r.t. the base frame */
  char dataConfirmation = 0;
  
  /* Get time now for file names */
  time_t t = time(0);   
  struct tm * now = localtime( & t );

  /* File names */
  string name_map_file = "map_";
  string name_mapVertex_file = "mapVertex_";
  string name_pointCloud_file = "pointCloudData_";
  char buffer_name_time[80];
  strftime (buffer_name_time,80,"%Y-%m-%d_%H-%M-%S",now);
  string name_time(buffer_name_time);
  
  /* Open Text files for Data Storage */
  ofstream map_file (name_map_file + name_time +".txt");
  ofstream mapVertex_file (name_mapVertex_file + name_time +".txt");
  ofstream pointCloud_file (name_pointCloud_file + name_time +".txt");
  
  /* App Initialization */
  if (argc < 6U)
  {
    cout << "ERROR: Too few arguments. Exiting..." << endl;
    exit(-1);
  }
  else 
  {
    /* Get deltas */
    deltaX = atof(argv[1]);
    deltaY = atof(argv[2]);
    deltaZ = atof(argv[3]);
    
    /* Get deltaScan */
    deltaScan = atof(argv[4]);
    
    /* Get speed */
    v = atof(argv[5]);
        
    /* Get Scanning direction */
    if (argc == 7U)
    {
      if ( strcmp("x", argv[6]) == 0 )
      {
        scan_x = true;
      }
      else if ( strcmp("y", argv[6]) == 0 )
      {
        scan_x = false;
      }
      else
      {
        cout << "ERROR: Scanning direction wrong. Exiting..." << endl;
        exit(-1);
      }
    }
    else
    {
      ;
    }
  }
  
  /* Data Confirmation */
  cout << endl << BOLDRED << "Data Confirmation:" << RESET << endl;
  cout << BOLDGREEN << "Delta X: " << RESET << deltaX << " m" << endl;
  cout << BOLDGREEN << "Delta Y: " << RESET << deltaY << " m" << endl;
  cout << BOLDGREEN << "Delta Z: " << RESET << deltaZ << " m" << endl;
  cout << BOLDGREEN << "DeltaScan: " << RESET << deltaScan << " m" << endl << endl;
  cout << BOLDGREEN << "v: " << RESET << v << " m/s" << endl << endl;
  cout << BOLDGREEN << "Scanning Direction: " << RESET;
  if(scan_x == true) cout << "x" << endl << endl;
  else cout << "y" << endl << endl;
  cout << "Do you want to proceed? [y/n]: ";
  scanf("%c", &dataConfirmation);
  if(dataConfirmation != 'y' && dataConfirmation != 'Y') exit(-1);
  
  /* Map generation */
  vector<float> map_x;
  vector<float> map_y;
  vector<float> map_time;
  uint32_t map_numRow = 0;
  float next_point_x = 0, next_point_y = 0, point_distance = 0;
  bool next_vertex = false;
  
  /* Add Current point - 1st point */
  map_x.push_back(0.0);
  map_y.push_back(0.0);
  map_time.push_back(deltaZ/v);
  
  /* Save first point into the file */
  map_file << map_x.at(0) << " " << map_y.at(0) << " " << map_time.at(0) << endl;
  mapVertex_file << map_x.at(0) << " " << map_y.at(0) << endl;
  
  /* Compute numRow and numColumn */
  if(scan_x == true)
  {
    map_numRow = (uint32_t)(deltaY/deltaScan);
  }
  else
  {
    map_numRow = (uint32_t)(deltaX/deltaScan);
  }
    
  for(uint32_t k = 1; k < map_numRow*2; k++)
  {
    if(scan_x == true)
    {       
      if(k%4 == 1)
      {
        /* Compute next vertex */
        next_point_x = map_x.back()+deltaX;
        next_point_y = map_y.back();
        
        next_vertex = false;
        
        /* Until next point is reached */
        while(!next_vertex)
        {
          point_distance = sqrt( (next_point_x-map_x.back())*(next_point_x-map_x.back()) + (next_point_y-map_y.back())*(next_point_y-map_y.back()) );
          if( point_distance > (TRAJ_MAX_POINT_DISTANCE + numeric_limits<float>::epsilon()) )
          {
            map_x.push_back(map_x.back()+TRAJ_MAX_POINT_DISTANCE);
            map_y.push_back(next_point_y);
          }
          else
          {
            map_x.push_back(next_point_x);
            map_y.push_back(next_point_y);
            
            next_vertex = true;
            
            mapVertex_file << next_point_x << " " << next_point_y << endl;
          }
          
          /* Compute point final time */
          map_time.push_back( sqrt( (map_x.back()-map_x.at(map_x.size()-2))*(map_x.back()-map_x.at(map_x.size()-2)) + 
                                    (map_y.back()-map_y.at(map_y.size()-2))*(map_y.back()-map_y.at(map_y.size()-2)) ) / v + map_time.at(map_time.size()-1) );
          
          /* Save point into the file */
          map_file << map_x.back() << " " << map_y.back() << " " << map_time.back() << endl;
        }
      }
      else if(k%4 == 2)
      {
        /* Compute next vertex */
        next_point_x = map_x.back();
        next_point_y = map_y.back()+deltaScan;
        
        next_vertex = false;
        
        /* Until next point is reached */
        while(!next_vertex)
        {
          point_distance = sqrt( (next_point_x-map_x.back())*(next_point_x-map_x.back()) + (next_point_y-map_y.back())*(next_point_y-map_y.back()) );
          if( point_distance > (TRAJ_MAX_POINT_DISTANCE + numeric_limits<float>::epsilon()) )
          {
            map_x.push_back(next_point_x);
            map_y.push_back(map_y.back()+TRAJ_MAX_POINT_DISTANCE);
          }
          else
          {
            map_x.push_back(next_point_x);
            map_y.push_back(next_point_y);
            
            next_vertex = true;
            
            mapVertex_file << next_point_x << " " << next_point_y << endl;
          }
          
          /* Compute point final time */
          map_time.push_back( sqrt( (map_x.back()-map_x.at(map_x.size()-2))*(map_x.back()-map_x.at(map_x.size()-2)) + 
                                    (map_y.back()-map_y.at(map_y.size()-2))*(map_y.back()-map_y.at(map_y.size()-2)) ) / v + map_time.at(map_time.size()-1) );
          
          /* Save point into the file */
          map_file << map_x.back() << " " << map_y.back() << " " << map_time.back() << endl;
        }
      }
      else if(k%4 == 3)
      {
        /* Compute next vertex */
        next_point_x = map_x.back()-deltaX;
        next_point_y = map_y.back();
        
        next_vertex = false;
        
        /* Until next point is reached */
        while(!next_vertex)
        {
          point_distance = sqrt( (next_point_x-map_x.back())*(next_point_x-map_x.back()) + (next_point_y-map_y.back())*(next_point_y-map_y.back()) );
          if( point_distance > (TRAJ_MAX_POINT_DISTANCE + numeric_limits<float>::epsilon()) )
          {
            map_x.push_back(map_x.back()-TRAJ_MAX_POINT_DISTANCE);
            map_y.push_back(next_point_y);
          }
          else
          {
            map_x.push_back(next_point_x);
            map_y.push_back(next_point_y);
            
            next_vertex = true;
            
            mapVertex_file << next_point_x << " " << next_point_y << endl;
          }
          
          /* Compute point final time */
          map_time.push_back( sqrt( (map_x.back()-map_x.at(map_x.size()-2))*(map_x.back()-map_x.at(map_x.size()-2)) + 
                                    (map_y.back()-map_y.at(map_y.size()-2))*(map_y.back()-map_y.at(map_y.size()-2)) ) / v + map_time.at(map_time.size()-1) );
          
          /* Save point into the file */
          map_file << map_x.back() << " " << map_y.back() << " " << map_time.back() << endl;
        }
      }
      else if(k%4 == 0)
      {
        /* Compute next vertex */
        next_point_x = map_x.back();
        next_point_y = map_y.back()+deltaScan;
        
        next_vertex = false;
        
        /* Until next point is reached */
        while(!next_vertex)
        {
          point_distance = sqrt( (next_point_x-map_x.back())*(next_point_x-map_x.back()) + (next_point_y-map_y.back())*(next_point_y-map_y.back()) );
          if( point_distance > (TRAJ_MAX_POINT_DISTANCE + numeric_limits<float>::epsilon()) )
          {
            map_x.push_back(next_point_x);
            map_y.push_back(map_y.back()+TRAJ_MAX_POINT_DISTANCE);
          }
          else
          {
            map_x.push_back(next_point_x);
            map_y.push_back(next_point_y);
            
            next_vertex = true;
            
            mapVertex_file << next_point_x << " " << next_point_y << endl;
          }
          
          /* Compute point final time */
          map_time.push_back( sqrt( (map_x.back()-map_x.at(map_x.size()-2))*(map_x.back()-map_x.at(map_x.size()-2)) + 
                                    (map_y.back()-map_y.at(map_y.size()-2))*(map_y.back()-map_y.at(map_y.size()-2)) ) / v + map_time.at(map_time.size()-1) );
          
          /* Save point into the file */
          map_file << map_x.back() << " " << map_y.back() << " " << map_time.back() << endl;
        }
      }
      else
      {
        ;
      }
    }
    else
    {
      if(k%4 == 1)
      {
        /* Compute next vertex */
        next_point_x = map_x.back();
        next_point_y = map_y.back()+deltaY;
        
        next_vertex = false;
        
        /* Until next point is reached */
        while(!next_vertex)
        {
          point_distance = sqrt( (next_point_x-map_x.back())*(next_point_x-map_x.back()) + (next_point_y-map_y.back())*(next_point_y-map_y.back()) );
          if( point_distance > (TRAJ_MAX_POINT_DISTANCE + numeric_limits<float>::epsilon()) )
          {
            map_x.push_back(next_point_x);
            map_y.push_back(map_y.back()+TRAJ_MAX_POINT_DISTANCE);
          }
          else
          {
            map_x.push_back(next_point_x);
            map_y.push_back(next_point_y);
            
            next_vertex = true;
            
            mapVertex_file << next_point_x << " " << next_point_y << endl;
          }
          
          /* Compute point final time */
          map_time.push_back( sqrt( (map_x.back()-map_x.at(map_x.size()-2))*(map_x.back()-map_x.at(map_x.size()-2)) + 
                                    (map_y.back()-map_y.at(map_y.size()-2))*(map_y.back()-map_y.at(map_y.size()-2)) ) / v + map_time.at(map_time.size()-1) );
          
          /* Save point into the file */
          map_file << map_x.back() << " " << map_y.back() << " " << map_time.back() << endl;
        }
      }
      else if(k%4 == 2)
      {
        /* Compute next vertex */
        next_point_x = map_x.back()+deltaScan;
        next_point_y = map_y.back();
        
        next_vertex = false;
        
        /* Until next point is reached */
        while(!next_vertex)
        {
          point_distance = sqrt( (next_point_x-map_x.back())*(next_point_x-map_x.back()) + (next_point_y-map_y.back())*(next_point_y-map_y.back()) );
          if( point_distance > (TRAJ_MAX_POINT_DISTANCE + numeric_limits<float>::epsilon()) )
          {
            map_x.push_back(map_x.back()+TRAJ_MAX_POINT_DISTANCE);
            map_y.push_back(next_point_y);
          }
          else
          {
            map_x.push_back(next_point_x);
            map_y.push_back(next_point_y);
            
            next_vertex = true;
            
            mapVertex_file << next_point_x << " " << next_point_y << endl;
          }
          
          /* Compute point final time */
          map_time.push_back( sqrt( (map_x.back()-map_x.at(map_x.size()-2))*(map_x.back()-map_x.at(map_x.size()-2)) + 
                                    (map_y.back()-map_y.at(map_y.size()-2))*(map_y.back()-map_y.at(map_y.size()-2)) ) / v + map_time.at(map_time.size()-1) );
          
          /* Save point into the file */
          map_file << map_x.back() << " " << map_y.back() << " " << map_time.back() << endl;
        }
      }
      else if(k%4 == 3)
      {
        /* Compute next vertex */
        next_point_x = map_x.back();
        next_point_y = map_y.back()-deltaY;
        
        next_vertex = false;
        
        /* Until next point is reached */
        while(!next_vertex)
        {
          point_distance = sqrt( (next_point_x-map_x.back())*(next_point_x-map_x.back()) + (next_point_y-map_y.back())*(next_point_y-map_y.back()) );
          if( point_distance > (TRAJ_MAX_POINT_DISTANCE + numeric_limits<float>::epsilon()) )
          {
            map_x.push_back(next_point_x);
            map_y.push_back(map_y.back()-TRAJ_MAX_POINT_DISTANCE);
          }
          else
          {
            map_x.push_back(next_point_x);
            map_y.push_back(next_point_y);
            
            next_vertex = true;
            
            mapVertex_file << next_point_x << " " << next_point_y << endl;
          }
          
          /* Compute point final time */
          map_time.push_back( sqrt( (map_x.back()-map_x.at(map_x.size()-2))*(map_x.back()-map_x.at(map_x.size()-2)) + 
                                    (map_y.back()-map_y.at(map_y.size()-2))*(map_y.back()-map_y.at(map_y.size()-2)) ) / v + map_time.at(map_time.size()-1) );
          
          /* Save point into the file */
          map_file << map_x.back() << " " << map_y.back() << " " << map_time.back() << endl;
        }
      }
      else if(k%4 == 0)
      {
        /* Compute next vertex */
        next_point_x = map_x.back()+deltaScan;
        next_point_y = map_y.back();
        
        next_vertex = false;
        
        /* Until next point is reached */
        while(!next_vertex)
        {
          point_distance = sqrt( (next_point_x-map_x.back())*(next_point_x-map_x.back()) + (next_point_y-map_y.back())*(next_point_y-map_y.back()) );
          if( point_distance > (TRAJ_MAX_POINT_DISTANCE + numeric_limits<float>::epsilon()) )
          {
            map_x.push_back(map_x.back()+TRAJ_MAX_POINT_DISTANCE);
            map_y.push_back(next_point_y);
          }
          else
          {
            map_x.push_back(next_point_x);
            map_y.push_back(next_point_y);
            
            next_vertex = true;
            
            mapVertex_file << next_point_x << " " << next_point_y << endl;
          }
          
          /* Compute point final time */
          map_time.push_back( sqrt( (map_x.back()-map_x.at(map_x.size()-2))*(map_x.back()-map_x.at(map_x.size()-2)) + 
                                    (map_y.back()-map_y.at(map_y.size()-2))*(map_y.back()-map_y.at(map_y.size()-2)) ) / v + map_time.at(map_time.size()-1) );
          
          /* Save point into the file */
          map_file << map_x.back() << " " << map_y.back() << " " << map_time.back() << endl;
        }
      }
      else
      {
        ; 
      }
    }
  } 
  
  /* Close the map file */
  map_file.close();
  mapVertex_file.close();
  
  /* ROS Initialization */
  string nodeName = "WireScan";
  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;
  
  /* ROS publisher declaration */
  ros::Publisher point_cloud_pub = n.advertise<geometry_msgs::Point>("/pointCloud", 1);
  ros::Publisher robot_traj_pub = n.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command", 1);
  geometry_msgs::Point point_msg;
  trajectory_msgs::JointTrajectory scanTrajectory_msg;
  trajectory_msgs::JointTrajectoryPoint scanTrajectoryPoint;
  scanTrajectoryPoint.positions.resize(6);
  
  /* Fill joints name */
  scanTrajectory_msg.joint_names.push_back("shoulder_pan_joint");
  scanTrajectory_msg.joint_names.push_back("shoulder_lift_joint");
  scanTrajectory_msg.joint_names.push_back("elbow_joint");
  scanTrajectory_msg.joint_names.push_back("wrist_1_joint");
  scanTrajectory_msg.joint_names.push_back("wrist_2_joint");
  scanTrajectory_msg.joint_names.push_back("wrist_3_joint");
  
  /* ROS subscriber declaration */
  ros::Subscriber proximity_sub = n.subscribe(PROXIMITY_SENSOR_TOPIC, 1, proximityCallback);
  ros::Subscriber jointStates_sub = n.subscribe("/joint_states", 1, jointStatesCallback);
  
  /* Sleep for a while */
  sleep(1);
  
  /* Get initial robot joint position */
  for(uint8_t k = 0; k < 50; k++)
  {
    ros::spinOnce();
    usleep(10000);
  }
  
  /* Compute initial ee position */
  Matrix<4,4> T_init = UR5e_robot.fkine(jointPosition_DH);
  
  /* Robot Variables initialization */
  Vector<> qpDH = Zeros(UR5e_robot.getNumJoints());
  qDH_k = jointPosition_DH;
  error = Ones; //<-- l'errore deve essere inizializzato diverso da 0
  UnitQuaternion oldQp(T_init);
  oldQ = oldQp;
  xdd = Zeros;
  xd = Zeros;
  x0 = transl(T_init);
  w = Zeros;
  
  /* Fill trajectory message */
  cout << "Processing the robot trajectory. Number of points to process: " << map_x.size() << endl;
  for(uint32_t k = 0; k < map_x.size(); k++)
  {
    error = Ones;
    x = x0 + makeVector( map_x.at(k), map_y.at(k), 0 ) + makeVector(0, 0, deltaZ);
    
    //CLIK
    while( norm(error) > 1E-5 )
    {
      quat = oldQ;
      
      qDH_k = UR5e_robot.clik(   
                                qDH_k, //<- qDH attuale
                                x, // <- posizione desiderata
                                quat, // <- quaternione desiderato
                                oldQ,// <- quaternione al passo precedente (per garantire la continuità)
                                xd, // <- velocità in translazione desiderata
                                w, //<- velocità angolare desiderata
                                Ones,// <- maschera, se l'i-esimo elemento è zero allora l'i-esima componente cartesiana non verrà usata per il calcolo dell'errore
                                0.2*hz,// <- guadagno del clik (qui è scelto in maniera conservativa)
                                1.0/hz,// <- Ts, tempo di campionamento
                                0.0, // <- quadagno obj secondario
                                Zeros(UR5e_robot.getNumJoints()), // velocità di giunto dell'obj secondario (qui sono zero)              
                                //Return Vars
                                qpDH, // <- variabile di ritorno velocità di giunto
                                error, //<- variabile di ritorno errore
                                oldQ // <- variabile di ritorno: Quaternione attuale (N.B. qui uso oldQ in modo da aggiornare direttamente la variabile oldQ e averla già pronta per la prossima iterazione)
                              );
    }
    
    /* Fill traj vector */
    jointPosition_Robot_cmd = UR5e_robot.joints_DH2Robot(qDH_k);
    
    scanTrajectoryPoint.positions[0] = jointPosition_Robot_cmd[0]; /* shoulder_pan_joint */
    scanTrajectoryPoint.positions[1] = jointPosition_Robot_cmd[1]; /* shoulder_lift_joint */
    scanTrajectoryPoint.positions[2] = jointPosition_Robot_cmd[2]; /* elbow_joint */
    scanTrajectoryPoint.positions[3] = jointPosition_Robot_cmd[3]; /* wrist_1_joint */
    scanTrajectoryPoint.positions[4] = jointPosition_Robot_cmd[4]; /* wrist_2_joint */
    scanTrajectoryPoint.positions[5] = jointPosition_Robot_cmd[5]; /* wrist_3_joint */
    
    scanTrajectoryPoint.time_from_start = ros::Duration(map_time.at(k));
    
    scanTrajectory_msg.points.push_back( scanTrajectoryPoint );
    
    /* Plot status */
    cout << "Points processed: " << k+1 << " of " <<  map_x.size() << "\r";
  }
  cout << endl;
  
  /* Print the Start */
  cout << BOLDGREEN << endl << "Wire scanning in progress..." << RESET << endl;
  
  /* First iteration flag  */
  bool first_iteration_flag = true;
  
  /* Set Node Rate */
  ros::Rate loop_rate(hz); /* [Hz] */


  
  /* Main loop */
  while (ros::ok())
  {    

    /* Get callback data */
    ros::spinOnce();

    /* Send the Trajectory only at the first loop iteration */
    if ( first_iteration_flag )
    {
      xe_first = xe;

      scanTrajectory_msg.header.stamp = ros::Time::now() + ros::Duration(0.5);
      
      robot_traj_pub.publish(scanTrajectory_msg);
         
      first_iteration_flag = false;
    }
    
    /* Store and send new PointCloud sample */
    if( proximity_new_data_available )
    {
      cout << xe[2] << endl;
      point_msg.x = xe[0];
      point_msg.y = xe[1];
      point_msg.z = xe[2] - proximity_data/1000.0;
     
      pointCloud_file << point_msg.x << " " << point_msg.y << " " << point_msg.z << endl;
      point_cloud_pub.publish( point_msg );
      
      proximity_new_data_available = false;
    }
    
    /* Check if trajectory is ended and close the application */
    if( sqrt( ((xe[0]-xe_first[0])-map_x.back())*((xe[0]-xe_first[0])-map_x.back()) + ((xe[1]-xe_first[1])-map_y.back())*((xe[1]-xe_first[1])-map_y.back()) ) < 1E-4 )
    {
      break;
    }
    else
    {
      ;
    }
    
    /* Sleep until new iteration */
    loop_rate.sleep();
  }
  
  /* Close PointCloud file */
  pointCloud_file.close();
  
  return 0;
}
