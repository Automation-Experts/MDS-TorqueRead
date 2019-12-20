/*
 ============================================================================
 Name : SBOT2_GROUPCONTROL.cpp
 Author :	Elmo Motion Control.
 Version :	1.00
 Description : The following example supports the following functionalities:

 - Power On.
 - Group Enable.
 - Move Linear Absolute to 0.
 - Move Linear Absolute to 10000.
 - Move Linear Absolute to 20000.
 - Move Linear Absolute to 0.
 - Power Off.
 - Group Disable.

 The program works with group v01 of 2 axes - a01 and a02.
 The program works with CAN and EtherCAT.

 ============================================================================
 */
#include "mmc_definitions.h"
#include "mmcpplib.h"
#include "main.h"		// Application header file.
#include <iostream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <sys/time.h>			// For time structure
#include <signal.h>				// For Timer mechanism
#include <string.h>
#include <ctime>

#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#define PORT 8080

/*
 ============================================================================
 Function:				main()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				04/10/2015
 Modifications:			N/A

 Description:

 The main function of this sample project.
 ============================================================================
 */

enum state {
	MotorInit, ServerInit, GetClientParams, RunMotors, EndOrRestart
};

struct BPosition {
	double pos;
	float vel;
	int mode;
};

int main(int argc, char *argv[]) {

	short int load_read = 0, lift_read = 0;
	short int load_pos = 0;
	int torque_mA = 500;
	int run_count = 0;
	int sampling_ms = 200;
	int run_limit = 20 * (1000 / sampling_ms);

	int current_position = 0;
	vector<BPosition> positions;
	BPosition p1 = { 4000, 2000, MC_BUFFERED_MODE };
	BPosition p2 = { 0, 4000, MC_BUFFERED_MODE };
	BPosition p3 = { 8000, 8000, MC_BUFFERED_MODE };
	BPosition p4 = { 200000, 100000, MC_BUFFERED_MODE };
	BPosition p5 = { 0, 100000, MC_BUFFERED_MODE };
	BPosition p6 = { 0, 8000, MC_BUFFERED_MODE };
	positions.push_back(p1);
	positions.push_back(p2);
	positions.push_back(p3);
	positions.push_back(p4);
	positions.push_back(p5);
	//positions.push_back(p6);

	char *usage = "Usage: (torque_mA) (sampling_ms)";

	string tc_str = "TC=";
	float torque_A;
	stringstream stream;

	string restart_msg;

	int state = MotorInit;

	while (1) {
		switch (state) {
		case MotorInit:
			MainInit();
			ChangeOpMode();

			while (NC_AXIS_DONE_MASK & lift_status)
				lift_status = load_axis.ReadStatus();
			while (NC_AXIS_DONE_MASK & load_status)
				load_status = lift_axis.ReadStatus();

			try {
//				executeInput(lift_axis, "HM[1]=0");
//				executeInput(lift_axis, "HM[3]=0");
//				executeInput(lift_axis, "HM[1]=1");

				string um_str = "UM=1";
				executeInput(load_axis, um_str);
				um_str = "UM=5";
				executeInput(lift_axis, um_str);

				cout << "Lift position: " << lift_axis.GetActualPosition()
						<< endl;
				//lift_axis.SetPosition(0, OPM402_PROFILE_POSITION_MODE);
				cout << "Position reset: " << lift_axis.GetActualPosition()
						<< endl;

				load_axis.PowerOn(MC_BUFFERED_MODE);
				lift_axis.PowerOn(MC_BUFFERED_MODE);
			}
			catch(CMMCException& exception) {
				lift_axis.SetPosition(0, OPM402_PROFILE_POSITION_MODE);
				printf(
						"Exception in function %s, axis ref=%s, err=%d, status=%d, %d, bye\n",
						exception.what(), exception.axisName(),
						exception.error(), exception.status(),
						exception.axisRef());
			}
			catch (...) {
				std::cerr << "Unknown exception caught\n";
				MainClose();
				exit(0);
			}

			// A is load, B is lift

			state = ServerInit;
			break;
		case ServerInit:
			cout << "Waiting for client to connect..." << endl;
			startServer();
			state = GetClientParams;
			break;
		case GetClientParams:
			if (getClientParams(new_socket, torque_mA, sampling_ms)) {
				// invalid params
				cout << "Invalid client params, trying again" << endl;
				state = GetClientParams;
			} else {
				cout << "Read torque: " << torque_mA << ", sampling_ms: " << sampling_ms << endl;

				//todo put the time in one spot
				time_t t = time(0);
				tm* now = localtime(&t);

				//todo make this send nicer
				stringstream date_stream;
				date_stream << (now->tm_year + 1900) << '-'<< (now->tm_mon + 1) << '-'<<  now->tm_mday << endl;
				cout << date_stream.str();
				send(new_socket, date_stream.str().c_str(), date_stream.str().length(), 0);
				state = RunMotors;
			}
			break;
		case RunMotors:
			torque_A = torque_mA / 1000.0;
			stream.str("");
			stream << fixed << setprecision(2) << torque_A;
			cout << "Sending: " << tc_str + stream.str() << endl;
			executeInput(load_axis, tc_str + stream.str());
			try {
				while (!(lift_status & NC_AXIS_ERROR_STOP_MASK)) {
					lift_status = load_axis.ReadStatus();
					load_status = lift_axis.ReadStatus();

					//a1.SendSdoUploadAsync(0,4,0x6077,0);
					load_read = load_axis.SendSdoUpload(0, 4, 0x6077, 0); //rc is current in mA
					lift_read = lift_axis.SendSdoUpload(0, 4, 0x6077, 0);
					load_read *= 10;
					lift_read *= 10;

					time_t t = time(0);
					tm* now = localtime(&t);
					stringstream time_stream;
					time_stream << (now->tm_hour + 1) << ':' << (now->tm_min + 1) << ':' << (now->tm_sec) ;
					stringstream read_out;
					read_out << "TIME," << time_stream.str() << ", LOAD," << setprecision(2) << load_read
							<< ", LIFT, " << lift_read << endl;
					string str = read_out.str();
					//					cout << "---- Load Current: " << load_read
					//							<< "  ---- Lift Current: " << lift_read << endl;
					send(new_socket, str.c_str(), str.length(), 0);

					load_pos = load_axis.SendSdoUpload(0, 4, 0x6064, 0);
					//cout << "---- A pos: " << a_pos << endl;

					usleep(sampling_ms * 1000);
					if (load_status & NC_AXIS_STAND_STILL_MASK) {
						if (current_position >= positions.size()) {
							break;
						}
						BPosition p = positions[current_position];
						lift_axis.MoveAbsolute(p.pos, p.vel, MC_BUFFERED_MODE);
						current_position++;
						//cout << endl << endl;
						send(new_socket, "\n\n", 2, 0);
						cout << "New motion: " << time_stream.str() << endl;
					}
				}
			}
			catch(CMMCException& exception) {
				MainClose();
				printf(
						"Exception in function %s, axis ref=%s, err=%d, status=%d, %d, bye\n",
						exception.what(), exception.axisName(),
						exception.error(), exception.status(),
						exception.axisRef());
				exit(0);
			}
			catch (...) {
				std::cerr << "Unknown exception caught\n";
				MainClose();
				exit(0);
			}

			cout << "Final position: " << lift_axis.GetActualPosition() << endl;
			state = EndOrRestart;
			break;

		case EndOrRestart:
			restart_msg = "Send 'r' to restart\n";
			send(new_socket, restart_msg.c_str(), restart_msg.length(), 0);
			read(new_socket, buffer, 1024);

			if (strstr(buffer,"r") != NULL) {
				current_position = 0;
				memset(buffer, 0, sizeof buffer);
				state = GetClientParams;
			} else {
				cout << "End of program" << endl;
				MainClose();
				return 0;
				break;
			}
		default:
			break;
		} // end of switch statement
	} // end of while loop

}

int getClientParams(int ff, int & torque_mA, int& sampling_ms) {
	char *usage = "Usage: (torque_mA) (sampling_ms)\n";

	send(new_socket, usage, strlen(usage), 0);

	read(new_socket, buffer, 1024);
	cout << buffer << endl;
	char * pch = strtok(buffer, " ,.-");
	//todo handle params better...
	int param_num = 0;
	while (pch != NULL) {
		int num = atoi(pch);
		if (num != 0) {
			if (param_num == 0) {
				if (num > 3000)
					num = 3000;
				torque_mA = num;
			} else if (param_num == 1) {
				if (num > 1000)
					num = 1000;
				sampling_ms = num;
			}
		} else {
			return 1;
		}
		param_num++;
		pch = strtok(NULL, " ,.-");
	}
	return 0;
}

int startServer() {
	int server_fd, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char *hello = "Hello from server\n";

	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	// Forcefully attaching socket to the port 8080
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);

	// Forcefully attaching socket to the port 8080
	if (bind(server_fd, (struct sockaddr *) &address, sizeof(address)) < 0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	if (listen(server_fd, 3) < 0) {
		perror("listen");
		exit(EXIT_FAILURE);
	}
	if ((new_socket = accept(server_fd, (struct sockaddr *) &address,
			(socklen_t*) &addrlen)) < 0) {
		perror("accept");
		exit(EXIT_FAILURE);
	}
	//valread = read(new_socket, buffer, 1024);
	send(new_socket, hello, strlen(hello), 0);
	printf("Hello message sent\n");
}

void executeInput(CMMCSingleAxis& axis, string input) {
	char input_array[input.length() + 1];
	strcpy(input_array, input.c_str());
	unsigned char * uarray = (unsigned char *) input_array;
	axis.ElmoExecute(uarray, input.length());
}

void usage() {
	cout << "Usage: MDS-TorqueRead logging_time_ms" << endl;
	exit(0);
}
/*
 ============================================================================
 Function:				MainInit()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Initilaize the system, including axes, communication, etc.
 ============================================================================
 */
void MainInit() {
	// Here will come initialization code for all system, axes, communication etc.
	struct sigaction stSigAction;
	float fRes;
	//
	// Init the sigAction structure.
	memset(&stSigAction, 0, sizeof(stSigAction));
	stSigAction.sa_handler = &TerminateApplication;
	// Initialized case of CTRL+C.
	sigaction(SIGINT, &stSigAction, NULL);
	//
	// InitializeCommunication to the GMAS:
	gConnHndl = cConn.ConnectIPCEx(0x7fffffff, (MMC_MB_CLBK) CallbackFunc);

	//	a_axis.ConfigPDO(PDO_NUM_3,PDO_PARAM_REG,NC_COMM_EVENT_GROUP1,1,1,1,1,1) ;
	//	CMMCPPGlobal::Instance()->SetSyncTime(gConnHndl, SYNC_MULTIPLIER) ;
	//

	// Set UM (control loop type) to
	//	int um_type = UM_TORQUE_CONTROL_LOOP;
	//	a_axis.ElmoSetAsyncParam("UM", um_type) ;
	//	um_type = UM_POSITION_CONTROL_LOOP;
	//	b_axis.ElmoSetAsyncParam("UM", um_type) ;
	//
	// Download UF which is only used to hold user parameters
	//	fRes = 12.3 ;
	//	a_axis.ElmoSetAsyncArray("UF",1,fRes) ;
	//	fRes = 0.0 ;
	//	a_axis.ElmoGetSyncArray("UF",1,fRes);
	//	cout << "UF result: " << fRes << endl;

	//
	// Register Run Time Error Callback function
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);
	//
	// 	Enable throw feature.
	CMMCPPGlobal::Instance()->SetThrowFlag(true, false);
	//
	// Register the callback function for Emergency:
	cConn.RegisterEventCallback(MMCPP_EMCY, (void*) Emergency_Received);
	//
	// Initialize default parameters of single axis. This is not a must. Each parameter may be initialized individually.
	stSingleDefault.fEndVelocity = 0;
	stSingleDefault.dbDistance = 100000;
	stSingleDefault.dbPosition = 0;
	stSingleDefault.fVelocity = 100000;
	stSingleDefault.fAcceleration = 1000000;
	stSingleDefault.fDeceleration = 1000000;
	stSingleDefault.fJerk = 20000000;
	stSingleDefault.eDirection = MC_POSITIVE_DIRECTION;
	stSingleDefault.eBufferMode = MC_BUFFERED_MODE;
	stSingleDefault.ucExecute = 1;
	//
	// Initialize default parameters of group axis.
	memset(stGroupDefault.dAuxPoint, 0, sizeof(double)
			* NC_MAX_NUM_AXES_IN_NODE);
	memset(stGroupDefault.dEndPoint, 0, sizeof(double)
			* NC_MAX_NUM_AXES_IN_NODE);
	memset(stGroupDefault.fTransitionParameter, 0, sizeof(float)
			* NC_MAX_NUM_AXES_IN_NODE);
	stGroupDefault.fVelocity = 100000;
	stGroupDefault.fAcceleration = 1000000;
	stGroupDefault.fDeceleration = 1000000;
	stGroupDefault.fJerk = 20000000;
	stGroupDefault.eCoordSystem = MC_ACS_COORD;
	stGroupDefault.eTransitionMode = MC_TM_NONE_MODE;
	stGroupDefault.eArcShortLong = MC_NONE_ARC_CHOICE;
	stGroupDefault.ePathChoice = MC_NONE_PATH_CHOICE;
	stGroupDefault.eCircleMode = MC_NONE_CIRC_MODE;
	stGroupDefault.ucSuperimposed = 0;
	stGroupDefault.m_uiExecDelayMs = 0;
	stGroupDefault.ucExecute = 1;
	//
	// TODO: Update number of necessary axes:
	//
	cout << "Initializing a_axis and a2..." << endl;
	load_axis.InitAxisData("a_axis", gConnHndl);
	lift_axis.InitAxisData("b_axis", gConnHndl);
	//v1.InitAxisData("v01",gConnHndl);
	//
	// Set default motion parameters. TODO: Update for all axes.
	load_axis.SetDefaultParams(stSingleDefault);
	lift_axis.SetDefaultParams(stSingleDefault);
	//v1.SetDefaultParams(stGroupDefault);
	//

	// You may of course change internal parameters manually:
	load_axis.m_fAcceleration = 960000.0;
	load_axis.m_fDeceleration = 960000.0;
	load_axis.m_fVelocity = 640000.0;
	//a2.m_fAcceleration = 1621333.0;
	//a2.m_fDeceleration = 1621333.0;
	//a2.m_fVelocity = 1013333.0;
	//

	lift_status = load_axis.ReadStatus();
	if (lift_status & NC_AXIS_ERROR_STOP_MASK) {
		load_axis.Reset();
		lift_status = load_axis.ReadStatus();
		if (lift_status & NC_AXIS_ERROR_STOP_MASK) {
			cout << "Axis a_axis in Error Stop. Aborting.";
			exit(0);
		}
	}
	//
	load_status = lift_axis.ReadStatus();
	if (load_status & NC_AXIS_ERROR_STOP_MASK) {
		lift_axis.Reset();
		load_status = lift_axis.ReadStatus();
		if (load_status & NC_AXIS_ERROR_STOP_MASK) {
			cout << "Axis a2 in Error Stop. Aborting.";
			exit(0);
		}
	}
	//
	//	giGroupStatus = v1.ReadStatus();
	//	if(giGroupStatus & NC_GROUP_ERROR_STOP_MASK)
	//	{
	//		v1.Reset();
	//		giGroupStatus = v1.ReadStatus();
	//
	//		if(giGroupStatus & NC_AXIS_ERROR_STOP_MASK)
	//		{
	//			cout << "Group v1 in Error Stop. Aborting." ;
	//			exit(0) ;
	//		}
	//	}
	//
	return;
}
void disableMotors()
{
	load_axis.Stop(MC_BUFFERED_MODE);
	lift_axis.Stop(MC_BUFFERED_MODE);
	load_axis.PowerOff(MC_BUFFERED_MODE);
	lift_axis.PowerOff(MC_BUFFERED_MODE);

	while (NC_AXIS_DISABLED_MASK & lift_status)
		lift_status = load_axis.ReadStatus();
	while (NC_AXIS_DISABLED_MASK & load_status)
		load_status = lift_axis.ReadStatus();

	load_axis.SetOpMode(OPM402_PROFILE_POSITION_MODE);
	loadOpMode = load_axis.GetOpMode();
	while (loadOpMode != OPM402_PROFILE_POSITION_MODE) {
		//a_axis.SetOpMode(OPM402_PROFILE_VELOCITY_MODE);
		loadOpMode = load_axis.GetOpMode();
	}
}
/*
 ============================================================================
 Function:				MainClose()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Close all that needs to be closed before the application progra, is
 terminated.
 ============================================================================
 */
void MainClose() {
	//
	//	Here will come code for all closing processes
	//

	disableMotors();
	MMC_CloseConnection(gConnHndl);
	return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Function name	:	void callback function																		//
//	Created			:	Version 1.00																				//
//	Updated			:	3/12/2010																					//
//	Modifications	:	N/A																							//
//	Purpose			:	interupt function 																			//
//																													//
//	Input			:	N/A																							//
//	Output			:	N/A																							//
//	Return Value	:	int																							//
//	Modifications:	:	N/A																							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize, void* lpsock) {
	// Which function ID was received ...
	switch (recvBuffer[1]) {
	case EMCY_EVT:
		//
		// Please note - The emergency event was registered.
		// printf("Emergency Event received\r\n") ;
		break;
	case ASYNC_REPLY_EVT:
		//		printf("buffer size: %d  ", recvBufferSize);
		//		for (int i = 0; i < recvBufferSize; i++)
		//			cout << recvBuffer[i];
		//		cout << endl;
		break;
	case MOTIONENDED_EVT:
		printf("Motion Ended Event received\r\n");
		break;
	case HBEAT_EVT:
		printf("H Beat Fail Event received\r\n");
		break;
	case PDORCV_EVT:
		printf("PDO Received Event received - Updating Inputs\r\n");
		break;
	case DRVERROR_EVT:
		printf("Drive Error Received Event received\r\n");
		break;
	case HOME_ENDED_EVT:
		printf("Home Ended Event received\r\n");
		break;
	case SYSTEMERROR_EVT:
		printf("System Error Event received\r\n");
		break;
		/* This is commented as a specific event was written for this function. Once it occurs
		 * the ModbusWrite_Received will be called
		 case MODBUS_WRITE_EVT:
		 // TODO Update additional data to be read such as function parameters.
		 // TODO Remove return 0 if you want to handle as part of callback.
		 return 0;
		 printf("Modbus Write Event received - Updating Outputs\r\n") ;

		 break ;
		 */
	}
	//
	return 1;
}
//
//
///////////////////////////////////////////////////////////////////////
//	Function name	:	int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Callback function in case a Run Time Error was received.
//	Input			:	const char - *msg, unsigned int - uiConnHndl, unsigned short - usAxisRef, short - sErrorID, unsigned short - usStatus.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
int OnRunTimeError(const char *msg, unsigned int uiConnHndl,
		unsigned short usAxisRef, short sErrorID, unsigned short usStatus) {
	MMC_CloseConnection(uiConnHndl);
	printf(
			"MMCPPExitClbk: Run time Error in function %s, axis ref=%d, err=%d, status=%d, bye\n",
			msg, usAxisRef, sErrorID, usStatus);
	exit(0);
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	void terminate_application(int iSigNum)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Called in case application is terminated, stop modbus, engines, and power off engines
//	Input			:	int iSigNum - Signal Num.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
void TerminateApplication(int iSigNum) {
	//
	printf("In Terminate Application ...\n");
	giTerminate = 1;
	sigignore(SIGALRM);
	//
	switch (iSigNum) {
	// Handle ctrl+c.
	case SIGINT:
		// TODO Close what needs to be closed before program termination.
		exit(0);
		break;
	default:
		break;
	}
	return;
}
//
//
///////////////////////////////////////////////////////////////////////
//	Function name	:	void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Callback function in case an Emergency event was received.
//	Input			:	unsigned short - usAxisRef, short - sEmcyCode.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode) {
	printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef,
			sEmcyCode);
}
//
///////////////////////////////////////////////////////////////////////
//	Function name	:	ChangeToRelevantMode
//	Created			:	Version 1.00
//	Updated			:	12/10/2015
//	Purpose			:	Called in case of working with group.
//						Changes the NC motion mode, according to communication type (ETHERCAT / CAN).
//	Input			:	N/A
//	Output			:	N/A
//	Return Value	:	void
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
void ChangeOpMode() {
	double dConnectionType;
	//
	dConnectionType
			= cConn.GetGlobalBoolParameter(MMC_CONNECTION_TYPE_PARAM, 0);

	//
	// ETHERCAT
	if (dConnectionType == eCOMM_TYPE_ETHERCAT) {
		//		a_axis.SetOpMode (OPM402_CYCLIC_SYNC_POSITION_MODE);
		//		//
		//		// Waiting for Set Operation Mode
		//		giXOpMode = (OPM402)a_axis.GetOpMode();
		//		while ( giXOpMode != OPM402_CYCLIC_SYNC_POSITION_MODE )
		//			giXOpMode =  a_axis.GetOpMode();
		//		//
		//		a2.SetOpMode (OPM402_CYCLIC_SYNC_POSITION_MODE);
		//		//
		//		// Waiting for Set Operation Mode
		//		giYOpMode =  a2.GetOpMode();
		//		while ( giYOpMode != OPM402_CYCLIC_SYNC_POSITION_MODE )
		//			giYOpMode =  a2.GetOpMode();
	}
	// CAN
	else {
		load_axis.SetOpMode(OPM402_PROFILE_POSITION_MODE);
		// Waiting for Set Operation Mode
		//
		loadOpMode = load_axis.GetOpMode();
		while (loadOpMode != OPM402_PROFILE_POSITION_MODE) {
			//a_axis.SetOpMode(OPM402_PROFILE_VELOCITY_MODE);
			loadOpMode = load_axis.GetOpMode();
		}

		lift_axis.SetOpMode(OPM402_PROFILE_POSITION_MODE);
		//
		// Waiting for Set Operation Mode
		liftOpMode = lift_axis.GetOpMode();
		while (liftOpMode != OPM402_PROFILE_POSITION_MODE)
			liftOpMode = lift_axis.GetOpMode();
	}
}

//
//
