#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int16.h>
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
double hz = 100; /* Hz */
int duration = 5;

Vector<3> p_des;
UnitQuaternion Q_des;

bool new_pose = false;

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
}

void desiredPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    p_des[0] = msg->position.x;
    p_des[1] = msg->position.y;
    p_des[2] = msg->position.z;

    Q_des = UnitQuaternion(makeVector(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z));

    new_pose = true;
}

void durationCallback(const std_msgs::Int16::ConstPtr &msg)
{
    duration = msg->data;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "go_to_point_node");
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
    ros::Subscriber duration_sub = n.subscribe("/duration", 1, durationCallback);
    ros::Subscriber jointStates_sub = n.subscribe("/joint_states", 1, jointStatesCallback);

    Matrix<4,4> n_T_e = Identity;
    n_T_e[2][3]=0.224;
    UR5e_robot.setnTe(n_T_e);

    Matrix<4,4> T_init = UR5e_robot.fkine(jointPosition_DH);
  
    /* Sleep for a while */
    sleep(1);

    while(ros::ok())
    {
        ros::spinOnce();

        if(new_pose)
        {   
            new_pose = false;
            trajectory_msg.points.clear();


            /* Clik Variables initialization */
            qDH_k = jointPosition_DH;
            error = Ones; //<-- l'errore deve essere inizializzato diverso da 0
            Matrix<4,4> T_init = UR5e_robot.fkine(jointPosition_DH);
            UnitQuaternion oldQp(T_init);
            oldQ = oldQp;
            // cout << oldQ << endl;
            Q_des = UnitQuaternion(Q_des,oldQ);
            // cout << Q_des << endl;
            // getchar();
            Vector<> qpDH = Zeros(UR5e_robot.getNumJoints());
            xd = Zeros;
            w = Zeros;

            //CLIK
            while( norm(error) > 1E-5 )
            {
                qDH_k = UR5e_robot.clik(   
                                            qDH_k, //<- qDH attuale
                                            p_des, // <- posizione desiderata
                                            Q_des, // <- quaternione desiderato
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
            
            trajectoryPoint.time_from_start = ros::Duration(duration);
            
            trajectory_msg.points.push_back( trajectoryPoint );

            trajectory_msg.header.stamp = ros::Time::now() + ros::Duration(0.5);

            cout << "Sending message..." << endl;
            robot_traj_pub.publish(trajectory_msg);
        }
    
    }

    return 0;
}