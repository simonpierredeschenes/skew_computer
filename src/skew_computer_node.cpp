#include <ros/ros.h>
#include <mutex>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <std_msgs/Float32.h>

typedef PointMatcher<float> PM;

const double GRAVITATIONAL_ACCELERATION = -9.807;
const double LINEAR_ACCELERATION_THRESHOLD = 0.2;
const double ANGULAR_SPEED_THRESHOLD = 0.02;
const ros::Duration MIN_STILL_SEQUENCE_DURATION(3.0);

std::shared_ptr<PM::Transformation> transformation;
std::mutex imuMeasurementMutex;
double latestLinearAcceleration;
double latestAngularSpeed;
bool stillSequenceBegun = false;
ros::Time startOfStillSequence;
bool stillCloudSaved = false;
PM::DataPoints stillCloud;
bool stillSequenceOver = false;
PM::DataPointsFilters filters;
PM::ICP icp;
std::string sensorFrame;
std::string odomFrame;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::ofstream outputFile;
ros::Publisher residualPublisher;

PM::TransformationParameters parseCloudPose(const std::string& cloudPoseString)
{
	std::string poseStr = cloudPoseString;
	PM::TransformationParameters cloudPose = PM::TransformationParameters::Identity(4, 4);
	
	poseStr.erase(std::remove(poseStr.begin(), poseStr.end(), '['), poseStr.end());
	poseStr.erase(std::remove(poseStr.begin(), poseStr.end(), ']'), poseStr.end());
	std::replace(poseStr.begin(), poseStr.end(), ',', ' ');
	std::replace(poseStr.begin(), poseStr.end(), ';', ' ');
	
	float poseMatrix[4 * 4];
	std::stringstream poseStringStream(poseStr);
	for(int i = 0; i < 4 * 4; i++)
	{
		if(!(poseStringStream >> poseMatrix[i]))
		{
			throw std::runtime_error("An error occurred while trying to parse the initial map pose.");
		}
	}
	
	float extraOutput = 0;
	if((poseStringStream >> extraOutput))
	{
		throw std::runtime_error("Invalid initial map pose dimension.");
	}
	
	for(int i = 0; i < 4 * 4; i++)
	{
		cloudPose(i / 4, i % 4) = poseMatrix[i];
	}
	
	return cloudPose;
}

void loadStillCloud(const std::string& cloudFile, PM::TransformationParameters cloudPose)
{
	stillSequenceOver = true;
	stillCloud = PM::DataPoints::load(cloudFile);
	stillCloud = transformation->compute(stillCloud, cloudPose);
}

void loadYamlConfig(const std::string& filtersConfigFile, const std::string& icpConfigFile)
{
	if(!filtersConfigFile.empty())
	{
		std::ifstream ifs(filtersConfigFile.c_str());
		filters = PM::DataPointsFilters(ifs);
		ifs.close();
	}
	
	if(!icpConfigFile.empty())
	{
		std::ifstream ifs(icpConfigFile.c_str());
		icp.loadFromYaml(ifs);
		ifs.close();
	}
	else
	{
		icp.setDefault();
	}
}

PM::TransformationParameters findTransform(std::string sourceFrame, std::string targetFrame, ros::Time time)
{
	geometry_msgs::TransformStamped tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time, ros::Duration(0.1));
	return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, 4);
}

void lidarCallback(const sensor_msgs::PointCloud2& msg)
{
	try
	{
		double linearAcceleration, angularSpeed;
		imuMeasurementMutex.lock();
		linearAcceleration = latestLinearAcceleration;
		angularSpeed = latestAngularSpeed;
		imuMeasurementMutex.unlock();
		
		if(stillSequenceOver)
		{
			PM::DataPoints cloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(msg);
			filters.apply(cloud);
			PM::TransformationParameters sensorToOdom = findTransform(sensorFrame, odomFrame, msg.header.stamp);
			cloud = transformation->compute(cloud, sensorToOdom);
			
			PM::TransformationParameters correction = icp(cloud, stillCloud);
			cloud = transformation->compute(cloud, correction);
			
			icp.matcher->init(stillCloud);
			PM::Matches matches(icp.matcher->findClosests(cloud));
			PM::OutlierWeights outlierWeights(icp.outlierFilters.compute(cloud, stillCloud, matches));
			float residual = icp.errorMinimizer->getResidualError(cloud, stillCloud, outlierWeights, matches);
			
			std_msgs::Float32 residualMsg;
			residualMsg.data = residual;
			residualPublisher.publish(residualMsg);
			
			if(outputFile.is_open())
			{
				outputFile << msg.header.stamp << "," << linearAcceleration << "," << angularSpeed << "," << residual << std::endl;
			}
		}
		else
		{
			if(stillSequenceBegun)
			{
				if(linearAcceleration > LINEAR_ACCELERATION_THRESHOLD || angularSpeed > ANGULAR_SPEED_THRESHOLD)
				{
					stillSequenceBegun = false;
					stillCloudSaved = false;
					if(linearAcceleration > LINEAR_ACCELERATION_THRESHOLD)
					{
						ROS_WARN_STREAM("Linear acceleration is " + std::to_string(linearAcceleration / LINEAR_ACCELERATION_THRESHOLD) + " times too high!");
					}
					if(angularSpeed > ANGULAR_SPEED_THRESHOLD)
					{
						ROS_WARN_STREAM("Angular speed is " + std::to_string(angularSpeed / ANGULAR_SPEED_THRESHOLD) + " times too high!");
					}
					ROS_WARN("Cancelling still sequence...");
					return;
				}
				
				if(!stillCloudSaved && msg.header.stamp - startOfStillSequence >= MIN_STILL_SEQUENCE_DURATION * 0.5)
				{
					// Saving cloud in the middle of the still sequence
					stillCloudSaved = true;
					ROS_INFO("Saving still cloud!");
					
					stillCloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(msg);
					filters.apply(stillCloud);
					PM::TransformationParameters sensorToOdom = findTransform(sensorFrame, odomFrame, msg.header.stamp);
					stillCloud = transformation->compute(stillCloud, sensorToOdom);
				}
				
				if(msg.header.stamp - startOfStillSequence >= MIN_STILL_SEQUENCE_DURATION)
				{
					stillSequenceOver = true;
					ROS_INFO("Ending still sequence!");
				}
			}
			else if(linearAcceleration <= LINEAR_ACCELERATION_THRESHOLD && angularSpeed <= ANGULAR_SPEED_THRESHOLD)
			{
				stillSequenceBegun = true;
				ROS_INFO("Beginning still sequence!");
				startOfStillSequence = msg.header.stamp;
			}
		}
	}
	catch(tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		return;
	}
}

void imuCallback(const sensor_msgs::Imu& msg)
{
	double linearAcceleration = std::sqrt(std::pow(msg.linear_acceleration.x, 2) +
										  std::pow(msg.linear_acceleration.y, 2) +
										  std::pow(msg.linear_acceleration.z - GRAVITATIONAL_ACCELERATION, 2));
	double angularSpeed = std::sqrt(std::pow(msg.angular_velocity.x, 2) +
									std::pow(msg.angular_velocity.y, 2) +
									std::pow(msg.angular_velocity.z, 2));
	
	imuMeasurementMutex.lock();
	latestLinearAcceleration = linearAcceleration;
	latestAngularSpeed = angularSpeed;
	imuMeasurementMutex.unlock();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "skew_computer_node");
	
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	
	transformation = PM::get().TransformationRegistrar.create("RigidTransformation");
	
	std::string stillCloudFile;
	if(pnh.getParam("still_cloud_file", stillCloudFile))
	{
		std::string stillCloudPoseString;
		pnh.param<std::string>("still_cloud_pose", stillCloudPoseString, "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]");
		PM::TransformationParameters stillCloudPose = parseCloudPose(stillCloudPoseString);
		loadStillCloud(stillCloudFile, stillCloudPose);
	}
	
	std::string filtersConfigFile, icpConfigFile;
	pnh.param<std::string>("filters_config_file", filtersConfigFile, "");
	pnh.param<std::string>("icp_config_file", icpConfigFile, "");
	loadYamlConfig(filtersConfigFile, icpConfigFile);
	
	pnh.param<std::string>("sensor_frame", sensorFrame, "rslidar32");
	pnh.param<std::string>("odom_frame", odomFrame, "odom");
	
	std::string outputFileName;
	if(pnh.getParam("output_file_name", outputFileName))
	{
		outputFile.open(outputFileName);
		outputFile.close();
		outputFile.open(outputFileName, std::ios::app);
		outputFile << "stamp,linear_acceleration,angular_speed,residual" << std::endl;
	}
	
	tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
	tf2_ros::TransformListener tfListener(*tfBuffer);
	
	ros::Subscriber sub1 = nh.subscribe("cloud_topic", 1, lidarCallback);
	ros::Subscriber sub2 = nh.subscribe("acceleration_topic", 1, imuCallback);
	
	residualPublisher = nh.advertise<std_msgs::Float32>("residual", 1);
	
	ros::spin();
	
	outputFile.close();
	
	return 0;
}
