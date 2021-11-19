#include <ros/ros.h>
#include "bash_color_list.h"
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "sun_robot_lib/Robot.h"
#include "sun_robot_lib/Robots/UR5e.h"
#include <sun_math_toolbox/UnitQuaternion.h>

using namespace std;
using namespace TooN;
using namespace sun;

Vector<6> jointPosition_Robot;
Vector<6> jointPosition_Robot_cmd;
Vector<6> jointPosition_DH;
Vector<6> qDH_k;
// Vector<> qpDH;
Vector<6> error; //<-- l'errore deve essere inizializzato diverso da 0
UnitQuaternion oldQ;
Vector<3> xd, w; // Desired Values
Vector<3> xe; // Actual ee position
double hz = 100; /* Hz */

vector <Vector<3>> p_des;
vector <UnitQuaternion> Q_des;
vector <double> point_distance;
double actual_distance;

bool new_traj = false;

UR5e UR5e_robot;

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

void desiredPoseCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    p_des.clear();
    Q_des.clear();
    point_distance.clear();

    for(int i=0; i<msg->poses.size(); i++)
    {
        p_des.push_back(makeVector(msg->poses[i].position.x, \
                                   msg->poses[i].position.y, \
                                   msg->poses[i].position.z));
        Q_des.push_back(UnitQuaternion(makeVector(msg->poses[i].orientation.w, \
                                                  msg->poses[i].orientation.x, \
                                                  msg->poses[i].orientation.y, \
                                                  msg->poses[i].orientation.z)));

        if(i!=0) // for the first point the distance is related to the actual pose (filled after)
        {
            point_distance.push_back( sqrt( (p_des.at(i)-p_des.at(i-1)) * (p_des.at(i)-p_des.at(i-1)) ) );
        }
    }

    new_traj = true;
}

int main(int argc, char* argv[])
{
    double velocity; // cruise speed in m/s
    char msgConfirmation, velConfirmation = 0;

    if (argc < 2U)
    {
        cout << BOLDRED <<"ERROR: " << RESET << "Insert the desired velocity [m/s]." << endl << "Exiting..." << endl;
        exit(-1);
    }

    velocity = atof(argv[1]);
    cout << endl << BOLDBLUE << "Velocity Confirmation: " << RESET << velocity << " m/s" << endl;
    cout << "Is it correct? [y/n]: ";
    cin >> velConfirmation;
    if(velConfirmation != 'y' && velConfirmation != 'Y') exit(-1);
    
    ros::init(argc, argv, "follow_traj_node");
    ros::NodeHandle n;

    ros::Publisher robot_traj_pub = n.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command", 1);
    trajectory_msgs::JointTrajectory trajectory_msg;
    trajectory_msgs::JointTrajectoryPoint trajectoryPoint;
    trajectoryPoint.positions.resize(6);
    
    /* Fill joints name */
    trajectory_msg.joint_names.push_back("shoulder_pan_joint");
    trajectory_msg.joint_names.push_back("shoulder_lift_joint");
    trajectory_msg.joint_names.push_back("elbow_joint");
    trajectory_msg.joint_names.push_back("wrist_1_joint");
    trajectory_msg.joint_names.push_back("wrist_2_joint");
    trajectory_msg.joint_names.push_back("wrist_3_joint");

    ros::Subscriber desiredPose_sub = n.subscribe("/goToPose", 1, desiredPoseCallback);
    ros::Subscriber jointStates_sub = n.subscribe("/joint_states", 1, jointStatesCallback);

    Matrix<4,4> T_init = UR5e_robot.fkine(jointPosition_DH);
  
    /* Sleep for a while */
    sleep(1);

    while(ros::ok())
    {
        ros::spinOnce();

        if(new_traj)
        {   
            new_traj = false;

            cout << endl << "New message received. Do you want to proceed? [y/n]: ";
            cin >> msgConfirmation;
            
            if(msgConfirmation != 'y' && msgConfirmation != 'Y')
            {
                cout << RED << "Trajectory discarded!" << RESET << endl;
            }
            else
            {
                actual_distance = 0.0;
                trajectory_msg.points.clear();

                point_distance.insert(point_distance.begin(), sqrt((p_des.at(0)-xe) * (p_des.at(0)-xe) ));
                if( point_distance.at(0) < 1E-3 ) // in case the motion is only in orientation, this distance is zero
                {
                    point_distance.at(0) = 0.05;
                }

                /* Clik Variables initialization */
                qDH_k = jointPosition_DH;
                error = Ones; //<-- l'errore deve essere inizializzato diverso da 0
                Matrix<4,4> T_init = UR5e_robot.fkine(jointPosition_DH);
                UnitQuaternion oldQp(T_init);
                oldQ = oldQp;
                Vector<> qpDH = Zeros(UR5e_robot.getNumJoints());
                xd = Zeros;
                w = Zeros;

                /* Fill trajectory message */
                for(int k=0; k<p_des.size(); k++)
                {
                    error = Ones;
                    Q_des[k] = UnitQuaternion(Q_des[k],oldQ); // correct desired quaternion for continuity
                    
                    //CLIK
                    while( norm(error) > 1E-5 )
                    {
                        qDH_k = UR5e_robot.clik(   
                                                    qDH_k, //<- qDH attuale
                                                    p_des[k], // <- posizione desiderata
                                                    Q_des[k], // <- quaternione desiderato
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
                    
                    trajectoryPoint.positions[0] = jointPosition_Robot_cmd[0]; /* shoulder_pan_joint */
                    trajectoryPoint.positions[1] = jointPosition_Robot_cmd[1]; /* shoulder_lift_joint */
                    trajectoryPoint.positions[2] = jointPosition_Robot_cmd[2]; /* elbow_joint */
                    trajectoryPoint.positions[3] = jointPosition_Robot_cmd[3]; /* wrist_1_joint */
                    trajectoryPoint.positions[4] = jointPosition_Robot_cmd[4]; /* wrist_2_joint */
                    trajectoryPoint.positions[5] = jointPosition_Robot_cmd[5]; /* wrist_3_joint */
                    
                    actual_distance +=  point_distance.at(k);
                    trajectoryPoint.time_from_start = ros::Duration(actual_distance/velocity);
                    
                    trajectory_msg.points.push_back( trajectoryPoint );

                    /* Plot status */
                    cout << "Processing the robot trajectory. Points processed: " << k+1 << " of " <<  p_des.size() << "\r";
                }
                cout << endl;

                trajectory_msg.header.stamp = ros::Time::now() + ros::Duration(0.5);

                cout << GREEN << "Executing trajectory..." << RESET << endl;
                robot_traj_pub.publish(trajectory_msg);
            }
        }
        ros::Duration(0.1).sleep();
    }

    return 0;
}