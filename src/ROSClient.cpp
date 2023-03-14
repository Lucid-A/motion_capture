// Linux 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/unistd.h>
#include <dirent.h>
#include <iostream>
#include <netdb.h>   
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <net/if.h>
#include <sys/ioctl.h>
#include <vector>
#define MAX_PATH 256

#include "SeekerSDKTypes.h"
#include "SeekerSDKClient.h"
#include "Utility.h"

#include "ros/ros.h"
#include "motion_capture/nokovStamped.h"


void  __attribute__((__cdecl__)) ForcePlateHandler(sForcePlates* data, void* pUserData);	// receives force plate data from the server
void  __attribute__((__cdecl__)) DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server
void  __attribute__((__cdecl__)) MessageHandler(int msgType, char* msg);		            // receives SeekerSDK error messages

int CreateClient(char* serverIp);

unsigned int MyServersDataPort = 3130;
unsigned int MyServersCommandPort = 3131;

SeekerSDKClient* theClient;
char szMyIPAddress[128] = "";
char szServerIPAddress[128] = "";

typedef std::vector<std::vector<SlideFrameArray>> TrackerArray;
// Used to calculate velocity and acceleration
TrackerArray MarkerVelocityTrackerArray;
TrackerArray MarkerAccelerationTrackerArray;
std::vector<SlideFrameArray> BoneVelocityTrackerArray;
std::vector<SlideFrameArray> BoneAccelerationTrackerArray;

ros::Publisher pub; 

int get_localip(const char * eth_name, char *local_ip_addr)
{	
	int ret = -1;    
	register int fd;    
	struct ifreq ifr; 	
	if (local_ip_addr == NULL || eth_name == NULL)
	{
		return ret;
	}
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) > 0)
	{
		strcpy (ifr.ifr_name, eth_name);
		if (!(ioctl(fd, SIOCGIFADDR, &ifr)))
		{
			ret = 0;
			strcpy(local_ip_addr, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
			printf("Found %s:%s\n", eth_name, local_ip_addr);
		}
	}
	if (fd > 0)
	{
		close(fd);
	}
	return ret;
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "nokov_forward");
    ros::NodeHandle nh;
    pub = nh.advertise<motion_capture::nokovStamped>("nokov_forward", 1000);
    ros::Rate loop_rate(120); // nokov frame rate: 60Hz

	int iResult = -1;

#ifdef LOST_FRAME_TEST
	memset(&s_Record, 0x00, sizeof(s_Record));
#endif
	
	//printf("Client gethostbyname szMyIPAddress[%s] .\n", szMyIPAddress);
	char ipBuf[100] = "192.168.0.123";

	//4.Create SeekerSDK Client
	iResult = CreateClient(ipBuf);
	if(iResult != ErrorCode_OK)
	{
		printf("Error initializing client.  Exiting");

		getchar();
		return 1;
	}
	else
	{
		printf("Client initialized and ready.\n");
	}

	getchar();
	getchar();

	// Done - clean up.
	theClient->Uninitialize();

	return ErrorCode_OK;
}

// Establish a SeekerSDK Client connection
int CreateClient(char* szServerIP)
{
	// release previous server
	if(theClient)
	{
		theClient->Uninitialize();
		delete theClient;
	}

	// create SeekerSDK client
	theClient = new SeekerSDKClient();

	// print version info
	unsigned char ver[4];
	theClient->SeekerSDKVersion(ver);
	printf("SeekerSDK Sample Client 2.4.0.2957(SeekerSDK ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	// Set callback handlers
	//theClient->SetMessageCallback(MessageHandler);
	//theClient->SetVerbosityLevel(Verbosity_Error);
	// Init Client and connect to SeekerSDK server
	int retCode = -1;
	retCode = theClient->Initialize(szServerIP);//szMyIPAddressClient IP (local IP)szServerIPServer IP (computer IP running motion capture software)
	//Sleep(10);
	memcpy(szServerIPAddress, szServerIP, sizeof(szServerIP));

	if (retCode != ErrorCode_OK)
	{
		printf("Unable to connect to server.Error code: %d. Exiting\n", retCode);
		return ErrorCode_Internal;
	}
	else
	{
		// print server info
		sServerDescription ServerDescription;
		memset(&ServerDescription, 0, sizeof(ServerDescription));
		theClient->GetServerDescription(&ServerDescription);
		if(!ServerDescription.HostPresent)
		{
			printf("Unable to connect to server. Host not present. Exiting.\n");
			return 1;
		}
		printf("Successfully connected to server\n");
	}

	{
		sDataDescriptions* ps = nullptr;
		theClient->GetDataDescriptions(&ps);
		if (ps)
		{
			for (int dsIndex = 0; dsIndex < ps->nDataDescriptions; ++dsIndex)
			{
				const auto& dataDescription = ps->arrDataDescriptions[dsIndex];

				switch (dataDescription.type)
				{
				case Descriptor_MarkerSet:
					printf("MarkerSetName : %s\n", dataDescription.Data.MarkerSetDescription->szName);
#ifdef LOST_FRAME_TEST
					s_Record.m_MarkerNums += dataDescription.Data.MarkerSetDescription->nMarkers;
#endif
					for (int markerIndex = 0; markerIndex < dataDescription.Data.MarkerSetDescription->nMarkers; ++markerIndex)
					{
						printf("Marker[%d] : %s\n", markerIndex, dataDescription.Data.MarkerSetDescription->szMarkerNames[markerIndex]);
					}
					break;
				case Descriptor_RigidBody:
					printf("RigidBody:%s\n", dataDescription.Data.RigidBodyDescription->szName);
#ifdef LOST_FRAME_TEST
					++s_Record.m_RigNums;
#endif
					break;
				case Descriptor_Skeleton:
					printf("Skeleton:%s\n", dataDescription.Data.SkeletonDescription->szName);
					for (int boneIndex = 0; boneIndex < dataDescription.Data.SkeletonDescription->nRigidBodies; ++boneIndex)
					{
						auto boneDescription = dataDescription.Data.SkeletonDescription->RigidBodies[boneIndex];
						printf("[%d] %s\n", boneDescription.ID, boneDescription.szName);
					}
					break;
				case Descriptor_ForcePlate:
					for (int channelIdx = 0; channelIdx < dataDescription.Data.ForcePlateDescription->nChannels; ++channelIdx)
					{
						printf("Channel:%d %s\n", channelIdx,
							dataDescription.Data.ForcePlateDescription->szChannelNames[channelIdx]);
					}
				default:
					break;
				}
			}

			theClient->FreeDataDescriptions(ps);
			ps = nullptr;
		}

	}

	theClient->WaitForForcePlateInit();
	theClient->SetDataCallback(DataHandler, theClient);	// this function will receive data from the server
	theClient->SetForcePlateCallback(ForcePlateHandler, theClient);
	return ErrorCode_OK;
}


// ForcePlateHandler receives data from the server
void  __attribute__((__cdecl__)) ForcePlateHandler(sForcePlates* pForcePlate, void* pUserData)
{
	if (nullptr != pForcePlate)
	{
		for (int plateIdx = 0; plateIdx < pForcePlate->nForcePlates; ++plateIdx)
		{
			printf("Frame[%d]:ForcePlate[%d]\nFxyz:(%lf,%lf,%lf)\nxyz:(%lf,%lf,%lf)\nMFree:%lf\n", pForcePlate->iFrame,
				plateIdx, pForcePlate->ForcePlates[plateIdx].Fxyz[0],
				pForcePlate->ForcePlates[plateIdx].Fxyz[1],
				pForcePlate->ForcePlates[plateIdx].Fxyz[2],
				pForcePlate->ForcePlates[plateIdx].xyz[0],
				pForcePlate->ForcePlates[plateIdx].xyz[1],
				pForcePlate->ForcePlates[plateIdx].xyz[2],
				pForcePlate->ForcePlates[plateIdx].Mfree);
		}
	}
}

// DataHandler receives data from the server
void  __attribute__((__cdecl__)) DataHandler(sFrameOfMocapData* data, void* pUserData)
{

	SeekerSDKClient* pClient = (SeekerSDKClient*) pUserData;

		int i = 0;

		// FrameOfMocapData params
		bool bIsRecording = data->params & 0x01;
		bool bTrackedModelsChanged = data->params & 0x02;
		if (bIsRecording)
			printf("RECORDING\n");
		if (bTrackedModelsChanged)
			printf("Models Changed.\n");

		// timecode - for systems with an eSync and SMPTE timecode generator - decode to values
		int hour, minute, second, frame, subframe;

		bool bValid = pClient->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
		// decode to friendly string
		char szTimecode[128] = "";
		pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
		printf("Timecode : %s\n", szTimecode);//print timecode stardard
		printf("FrameNO:%d\tTimeStamp:%lld\n", data->iFrame, data->iTimeStamp);

		printf("MarkerSet [Count=%d]\n", data->nMarkerSets);	//Number of Markerset

		MarkerVelocityTrackerArray.resize(data->nMarkerSets);
		MarkerAccelerationTrackerArray.resize(data->nMarkerSets);
        static Vel MarkerVelocity[30][6][3];
		static angle heading[30][6][3]={0};
		for(i=0; i < data->nMarkerSets; i++)
		{
			sMarkerSetData *markerset = &data->MocapData[i];
			MarkerVelocityTrackerArray[i].resize(markerset->nMarkers+1);
			MarkerAccelerationTrackerArray[i].resize(markerset->nMarkers+1);

            static CalculateVelocity method(60, 5); //FPS:60 FrameFactor:3
			static CalculateAcceleration method2(60, 5); //FPS:60 FrameFactor:5
            
            motion_capture::nokovStamped msg;
            msg.header.stamp = ros::Time::Time(data->iTimeStamp / 1000, (data->iTimeStamp % 1000) * 1E6);
            msg.header.frame_id = std::string("nokov");

            double w[6][3]={0};
			Accel MarkerAccleration;
			double center_x=0;
			double center_y=0;
			double center_z=0;
			double a[3][4]={{1110.02,1,0,0},
			                { 991.62,0,1,0},
				            {      0,0,0,1}
			               };
			double transform_map[6][3]={0};
			printf( "Markerset%d: %s [nMarkers Count=%d]\n", i+1, markerset->szName, markerset->nMarkers);  //Output the id and name of the i-th Markerset and the total number of named markers in the Markerset
			printf( "{\n");
			for (int i_Marker = 0;i_Marker < markerset->nMarkers; i_Marker++)	//Output the id and information (x, y, z) of the Marker point contained in the i-th Markerset
			{
				transform_map[i_Marker][0]=-a[0][0]*a[0][1]-a[1][0]*a[1][1]-a[2][0]*a[2][1]
				                           +a[0][1]*markerset->Markers[i_Marker][0]
										   +a[1][1]*markerset->Markers[i_Marker][1]
										   +a[2][1]*markerset->Markers[i_Marker][2];
				transform_map[i_Marker][1]=-a[0][0]*a[0][2]-a[1][0]*a[1][2]-a[2][0]*a[2][2]
				                           +a[0][2]*markerset->Markers[i_Marker][0]
										   +a[1][2]*markerset->Markers[i_Marker][1]
										   +a[2][2]*markerset->Markers[i_Marker][2];
				transform_map[i_Marker][2]=-a[0][0]*a[0][3]-a[1][0]*a[1][3]-a[2][0]*a[2][3]
				                           +a[0][3]*markerset->Markers[i_Marker][0]
										   +a[1][3]*markerset->Markers[i_Marker][1]
										   +a[2][3]*markerset->Markers[i_Marker][2];
				printf("\tMarker%d: %3.2f,%3.2f,%3.2f\n",
					i_Marker,
					transform_map[i_Marker][0],
					transform_map[i_Marker][1],
					transform_map[i_Marker][2]);

				// calculate the velocity and acceleration
				MarkerVelocityTrackerArray[i][i_Marker].Cache(transform_map[i_Marker][0]
					, transform_map[i_Marker][1], transform_map[i_Marker][2],data->iTimeStamp);
				MarkerAccelerationTrackerArray[i][i_Marker].Cache(transform_map[i_Marker][0]
					, transform_map[i_Marker][1], transform_map[i_Marker][2],data->iTimeStamp);


                MarkerVelocity[i][i_Marker][0]=MarkerVelocity[i][i_Marker][1];
				MarkerVelocity[i][i_Marker][1]=MarkerVelocity[i][i_Marker][2];
				MarkerVelocityTrackerArray[i][i_Marker].tryToCalculate(MarkerVelocity[i][i_Marker][2], method);
                
				heading[i][i_Marker][0]=heading[i][i_Marker][1];
				heading[i][i_Marker][1]=heading[i][i_Marker][2];
				heading[i][i_Marker][2].heading=atan2(MarkerVelocity[i][i_Marker][2].Vy+MarkerVelocity[i][i_Marker][1].Vy+MarkerVelocity[i][i_Marker][0].Vy
				                                     ,MarkerVelocity[i][i_Marker][2].Vx+MarkerVelocity[i][i_Marker][1].Vx+MarkerVelocity[i][i_Marker][0].Vx);
				heading[i][i_Marker][2].TimeStamp=data->iTimeStamp;
				if(heading[i][i_Marker][2].heading-heading[i][i_Marker][0].heading>5)
				{
                    w[i_Marker][0]=w[i_Marker][1];
					w[i_Marker][1]=w[i_Marker][2];
                    w[i_Marker][2]=1000*(heading[i][i_Marker][2].heading-heading[i][i_Marker][0].heading-2*M_PI)
					            /(heading[i][i_Marker][2].TimeStamp-heading[i][i_Marker][0].TimeStamp);
					w[i_Marker][2]=(w[i_Marker][0]+w[i_Marker][1]+w[i_Marker][2])/3;
				}
				else if(heading[i][i_Marker][2].heading-heading[i][i_Marker][0].heading<-5)
				{
					w[i_Marker][0]=w[i_Marker][1];
					w[i_Marker][1]=w[i_Marker][2];
                    w[i_Marker][2]=1000*(heading[i][i_Marker][2].heading-heading[i][i_Marker][0].heading+2*M_PI)
					            /(heading[i][i_Marker][2].TimeStamp-heading[i][i_Marker][0].TimeStamp);
					w[i_Marker][2]=(w[i_Marker][0]+w[i_Marker][1]+w[i_Marker][2])/3;
				}
				else
				{
					w[i_Marker][0]=w[i_Marker][1];
					w[i_Marker][1]=w[i_Marker][2];
					w[i_Marker][2]=1000*(heading[i][i_Marker][2].heading-heading[i][i_Marker][0].heading)
					            /(heading[i][i_Marker][2].TimeStamp-heading[i][i_Marker][0].TimeStamp);
					w[i_Marker][2]=(w[i_Marker][0]+w[i_Marker][1]+w[i_Marker][2])/3;
				}
                
				
				MarkerAccelerationTrackerArray[i][i_Marker].tryToCalculate(MarkerAccleration, method2);
				std::cout<< "\t\t"<<"w:"<<w[i_Marker][2]<< "\t"<<"heading:"<<heading[i][i_Marker][2].heading<<std::endl;;
				std::cout << "\t\t" <<MarkerVelocity[i][i_Marker][2]<< "\t\t" << MarkerAccleration;
			}
			for(int i_Marker = 0;i_Marker < markerset->nMarkers; i_Marker++)
			{
              center_x+=transform_map[i_Marker][0]/markerset->nMarkers;
			  center_y+=transform_map[i_Marker][1]/markerset->nMarkers;
			  center_z+=transform_map[i_Marker][2]/markerset->nMarkers;
			}
            

			double x1=transform_map[2][0]-transform_map[0][0];
			double y1=transform_map[2][1]-transform_map[0][1];
			double x2=transform_map[1][0]-transform_map[3][0];
			double y2=transform_map[1][1]-transform_map[3][1];
			double car_heading=atan2(y1+y2,x1+x2);
            
			MarkerVelocityTrackerArray[i][markerset->nMarkers].Cache(center_x, center_y, center_z,data->iTimeStamp);
			MarkerAccelerationTrackerArray[i][markerset->nMarkers].Cache(center_x, center_y, center_z,data->iTimeStamp);
            
            MarkerVelocity[i][markerset->nMarkers][0]=MarkerVelocity[i][markerset->nMarkers][1];
			MarkerVelocity[i][markerset->nMarkers][1]=MarkerVelocity[i][markerset->nMarkers][2];
			MarkerVelocityTrackerArray[i][markerset->nMarkers].tryToCalculate(MarkerVelocity[i][markerset->nMarkers][2], method);
                
			heading[i][markerset->nMarkers][0]=heading[i][markerset->nMarkers][1];
			heading[i][markerset->nMarkers][1]=heading[i][markerset->nMarkers][2];
			heading[i][markerset->nMarkers][2].heading=atan2(MarkerVelocity[i][markerset->nMarkers][2].Vy+MarkerVelocity[i][markerset->nMarkers][1].Vy+MarkerVelocity[i][markerset->nMarkers][0].Vy
			                                                ,MarkerVelocity[i][markerset->nMarkers][2].Vx+MarkerVelocity[i][markerset->nMarkers][1].Vx+MarkerVelocity[i][markerset->nMarkers][0].Vx);
			heading[i][markerset->nMarkers][2].TimeStamp=data->iTimeStamp;
			if(heading[i][markerset->nMarkers][2].heading-heading[i][markerset->nMarkers][0].heading>5)
			{
				w[markerset->nMarkers][0]=w[markerset->nMarkers][1];
				w[markerset->nMarkers][1]=w[markerset->nMarkers][2];
                w[markerset->nMarkers][2]=1000*(heading[i][markerset->nMarkers][2].heading-heading[i][markerset->nMarkers][0].heading-2*M_PI)
				                       /(heading[i][markerset->nMarkers][2].TimeStamp-heading[i][markerset->nMarkers][0].TimeStamp);
				w[markerset->nMarkers][2]=(w[markerset->nMarkers][0]+w[markerset->nMarkers][1]+w[markerset->nMarkers][2])/3;
			}
			else if(heading[i][markerset->nMarkers][2].heading-heading[i][markerset->nMarkers][0].heading<-5)
			{
				w[markerset->nMarkers][0]=w[markerset->nMarkers][1];
				w[markerset->nMarkers][1]=w[markerset->nMarkers][2];
                w[markerset->nMarkers][2]=1000*(heading[i][markerset->nMarkers][2].heading-heading[i][markerset->nMarkers][0].heading+2*M_PI)
				                       /(heading[i][markerset->nMarkers][2].TimeStamp-heading[i][markerset->nMarkers][0].TimeStamp);
			    w[markerset->nMarkers][2]=(w[markerset->nMarkers][0]+w[markerset->nMarkers][1]+w[markerset->nMarkers][2])/3;
			}
			else
			{
				w[markerset->nMarkers][0]=w[markerset->nMarkers][1];
				w[markerset->nMarkers][1]=w[markerset->nMarkers][2];
				w[markerset->nMarkers][2]=1000*(heading[i][markerset->nMarkers][2].heading-heading[i][markerset->nMarkers][0].heading)
				                       /(heading[i][markerset->nMarkers][2].TimeStamp-heading[i][markerset->nMarkers][0].TimeStamp);
			    w[markerset->nMarkers][2]=(w[markerset->nMarkers][0]+w[markerset->nMarkers][1]+w[markerset->nMarkers][2])/3;
			}
            MarkerAccelerationTrackerArray[i][markerset->nMarkers].tryToCalculate(MarkerAccleration, method2);
			
            msg.pose2d.x = center_x;
            msg.pose2d.y = center_y;
            msg.pose2d.theta = car_heading;

            msg.vel_linear.x = MarkerVelocity[i][markerset->nMarkers][2].Vx;
            msg.vel_linear.y = MarkerVelocity[i][markerset->nMarkers][2].Vy;
            msg.vel_linear.z = MarkerVelocity[i][markerset->nMarkers][2].Vz;

            msg.vel_theta = heading[i][markerset->nMarkers][2].heading;
            msg.yaw_omega = w[markerset->nMarkers][2];

            msg.acc_linear.x = MarkerAccleration.Ax;
            msg.acc_linear.y = MarkerAccleration.Ay;
            msg.acc_linear.z = MarkerAccleration.Az;

            pub.publish(msg);

            printf("\tMarkercenter: %3.2f,%3.2f,%3.2f\n",
					center_x,
					center_y,
					center_z);
			std::cout<< "\t\t"<<"w_cen:"<<w[markerset->nMarkers][2]<< "\t"<<"heading_cen:"<<heading[i][markerset->nMarkers][2].heading << "\t"<<"car_heading:"<<car_heading<<std::endl;
			std::cout << "\t\t" <<MarkerVelocity[i][markerset->nMarkers][2]<< "\t\t" << MarkerAccleration;
			printf( "}\n");//The data output of the i-th Markerset is completed
		}

		BoneVelocityTrackerArray.resize(data->nRigidBodies);
		BoneAccelerationTrackerArray.resize(data->nRigidBodies);

		//Print RigidBodies
		printf("Markerset.RigidBodies [Count=%d]\n", data->nRigidBodies);//Number of Markerset.Skeleton(skeleton)
		printf("{\n");
		for (i = 0; i < data->nRigidBodies; i++)
		{
			printf("\tid\t\tx\ty\tz\tqx\tqy\tqz\tqw\n");
			printf("\tRigidBody%d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
				data->RigidBodies[i].ID,
				data->RigidBodies[i].x,
				data->RigidBodies[i].y,
				data->RigidBodies[i].z,
				data->RigidBodies[i].qx,
				data->RigidBodies[i].qy,
				data->RigidBodies[i].qz,
				data->RigidBodies[i].qw);

			// calculate the velocity and acceleration
			BoneVelocityTrackerArray[i].Cache(data->RigidBodies[i].x
				, data->RigidBodies[i].y, data->RigidBodies[i].z,data->iTimeStamp);
			BoneAccelerationTrackerArray[i].Cache(data->RigidBodies[i].x
				, data->RigidBodies[i].y, data->RigidBodies[i].z,data->iTimeStamp);

			//Caution: Actually, you cat get velocity of frame 2 after you get frame 3's position, But it just has a little difference
			static CalculateVelocity method1(60, 3); //FPS:60 FrameFactor:3
			static CalculateAcceleration method2(60, 3); //FPS:60 FrameFactor:3

			Vel boneVelocity;
			BoneVelocityTrackerArray[i].tryToCalculate(boneVelocity, method1);

			Accel boneAccleration;
			BoneAccelerationTrackerArray[i].tryToCalculate(boneAccleration, method2);

			std::cout << "\t\t" << boneVelocity << "\t\t" << boneAccleration;

			printf("\tRigidBody markers [Count=%d]\n", data->RigidBodies[i].nMarkers);
			for (int iMarker = 0; iMarker < data->RigidBodies[i].nMarkers; iMarker++)//Output the id and information (x, y, z) of the marker associated with the j-th RigidBody (RigidBody)
			{
				printf("\t\t");
				if (data->RigidBodies[i].MarkerIDs)
					printf("Marker%d: ", data->RigidBodies[i].MarkerIDs[iMarker]);
				if (data->RigidBodies[i].Markers)
					printf("%3.2f,%3.2f,%3.2f\n",
						data->RigidBodies[i].Markers[iMarker][0],
						data->RigidBodies[i].Markers[iMarker][1],
						data->RigidBodies[i].Markers[iMarker][2]);
			}

			printf("}\n");//The data output of the j-th RigidBody (RigidBody) is completed
		}

		//Print Skeletons
		printf("Markerset.Skeletons [Count=%d]\n", data->nSkeletons);//Number of Markerset.Skeleton(skeleton)
		for(i=0; i < data->nSkeletons; i++)
		{
			printf("Markerset%d.Skeleton [nRigidBodies Count=%d]\n",  data->Skeletons[i].skeletonID,  data->Skeletons[i].nRigidBodies);//Skeleton (Skeleton) of the i-th Markerset of the motion capture data, including the number of RigidBody (RigidBody)
			printf( "{\n");
			for(int j = 0; j < data->Skeletons[i].nRigidBodies; j++)//Output id and information(x, y, z, qx, qy, qz, qw) of the j-th RigidBody (RigidBody) 
			{
				printf("\tid\t\tx\ty\tz\tqx\tqy\tqz\tqw\n");
				printf("\tRigidBody%d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
					data->Skeletons[i].RigidBodyData[j].ID, 
					data->Skeletons[i].RigidBodyData[j].x, 
					data->Skeletons[i].RigidBodyData[j].y, 
					data->Skeletons[i].RigidBodyData[j].z, 
					data->Skeletons[i].RigidBodyData[j].qx, 
					data->Skeletons[i].RigidBodyData[j].qy, 
					data->Skeletons[i].RigidBodyData[j].qz, 
					data->Skeletons[i].RigidBodyData[j].qw );

				printf("\tRigidBody markers [Count=%d]\n", data->Skeletons[i].RigidBodyData[j].nMarkers);
				for(int iMarker=0; iMarker < data->Skeletons[i].RigidBodyData[j].nMarkers; iMarker++)//Output the id and information (x, y, z) of the marker associated with the j-th RigidBody (RigidBody)
				{
					printf("\t\t");
					if(data->Skeletons[i].RigidBodyData[j].MarkerIDs)
						printf("Marker%d: ", data->Skeletons[i].RigidBodyData[j].MarkerIDs[iMarker]);
					if(data->Skeletons[i].RigidBodyData[j].Markers)
						printf("%3.2f,%3.2f,%3.2f\n" ,
						data->Skeletons[i].RigidBodyData[j].Markers[iMarker][0],
						data->Skeletons[i].RigidBodyData[j].Markers[iMarker][1],
						data->Skeletons[i].RigidBodyData[j].Markers[iMarker][2]);
				}
			}
			printf( "}\n");//The data output of the j-th RigidBody (RigidBody) is completed
		}
		printf("Other Markers [Count=%d]\n", data->nOtherMarkers);//Output the total number of unnamed markers contained
		printf( "{\n");
		for(i = 0; i < data->nOtherMarkers; i++)//Output the id and information (x, y, z) of the unnamed marker included
		{
			printf("\tOther Marker%d: %3.2f,%3.2f,%3.2f\n",
				i,
				data->OtherMarkers[i][0],
				data->OtherMarkers[i][1],
				data->OtherMarkers[i][2]);
		}
		printf("}\n\n");//The data output of unnamed marker is completed, and the data output of one frame is completed
		printf("Analog [Count=%d]\n", data->nAnalogdatas);//Output the total number of analog data contained
		printf("{\n");
		for (int i = 0; i < data->nAnalogdatas; ++i)
		{
			printf("\tAnalogData %d: %3.2f\n", i, data->Analogdata[i]);
		}
		printf("}\n\n");//The data output of analog data is completed, and the data output of one frame is completed
	}


// MessageHandler receives SeekerSDK error/debug messages
void  __attribute__((__cdecl__)) MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}
