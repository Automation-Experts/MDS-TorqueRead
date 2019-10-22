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
#include <sys/time.h>			// For time structure
#include <signal.h>				// For Timer mechanism
#include <string.h>
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

int main(int argc, char *argv[])
{
	try
	{
		int sleep_ms = 0;
		// Note that the first argument is always opening a udp socket
		if (argc > 1)
		{
			sleep_ms = atoi(argv[1]);
			if(!sleep_ms)
			{
				cout << "Sleep timer was invalid or 0" << endl;
				exit(0);
			}
		}
		else
		{
			cout << "Error: Missing Arguments" << endl;
			usage();
		}


		//	Initialize system, axes and all needed initializations
		MainInit();
		// Changes op mode type based on (ETHERCAT / CAN).
		ChangeToRelevantMode();

		short int a_read = 0, b_read = 0;
		short int a_pos = 0;
		int run_count = 0;
		int run_limit = 10*(1000/sleep_ms);

		a_axis.PowerOn(MC_BUFFERED_MODE);
		b_axis.PowerOn(MC_BUFFERED_MODE);

		while( NC_AXIS_DONE_MASK & giXStatus)
			giXStatus = a_axis.ReadStatus();
		while( NC_AXIS_DONE_MASK & giYStatus)
			giYStatus = b_axis.ReadStatus();


		cout << "Axes initialized..." << endl;

		//string pos_str = "PA[1]=8000";
		//executeInput(b_axis,pos_str);
		//executeInput(b_axis,"BG");

		b_axis.SetPosition(0,OPM402_PROFILE_POSITION_MODE);
		b_axis.MoveAbsolute(12000,2000, MC_BUFFERED_MODE);

		//string xq_str = "XQ##P2P_Abs(2000,1000)";
		//string pa_str = "PA[1]=4000";
		string tc_str = "TC=0.4";
		executeInput(a_axis,tc_str);


		while (! (giXStatus & NC_AXIS_ERROR_STOP_MASK) && ++run_count < run_limit)
		{
			giXStatus = a_axis.ReadStatus();
			giYStatus = b_axis.ReadStatus();

			if (run_count == 2)
			{
				executeInput(a_axis,"TC=0.14");
			}

			//a1.SendSdoUploadAsync(0,4,0x6077,0);
			a_read = a_axis.SendSdoUpload(0,4,0x6077,0);  //rc is current in mA
			b_read = b_axis.SendSdoUpload(0,4,0x6077,0);
			cout << "---- A current: " << a_read << "  ---- B current: " << b_read << endl;

			a_pos = a_axis.SendSdoUpload(0,4,0x6064,0);
			//cout << "---- A pos: " << a_pos << endl;

			usleep(sleep_ms * 1000);
		}

		// Terminate the application program back to the Operating System
		MainClose();
		cout << "End of program" << endl;
		return 0;
	}
	catch(CMMCException& exception)
	{
		MainClose();
		printf("Exception in function %s, axis ref=%s, err=%d, status=%d, %d, bye\n", exception.what(), exception.axisName(), exception.error(), exception.status(), exception.axisRef());
		exit(0);
	}
	catch (...)
	{
		std::cerr << "Unknown exception caught\n";
		MainClose();
		exit(0);
	}
}

void executeInput(CMMCSingleAxis& axis, string input)
{
	char input_array[input.length() + 1];
	strcpy(input_array, input.c_str());
	unsigned char * uarray = (unsigned char *) input_array;
	axis.ElmoExecute(uarray, input.length());
}

void usage()
{
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
void MainInit()
{
	// Here will come initialization code for all system, axes, communication etc.
	//
	struct sigaction stSigAction;
	//
	// Init the sigAction structure.
	memset(&stSigAction, 0, sizeof(stSigAction));
	stSigAction.sa_handler = &TerminateApplication;
	// Initialized case of CTRL+C.
	sigaction(SIGINT, &stSigAction, NULL);
	//
	// InitializeCommunication to the GMAS:
	gConnHndl = cConn.ConnectIPCEx(0x7fffffff,(MMC_MB_CLBK)CallbackFunc) ;
	//
	// Register Run Time Error Callback function
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);
	//
	// 	Enable throw feature.
	CMMCPPGlobal::Instance()->SetThrowFlag(true,false);
	//
	// Register the callback function for Emergency:
	cConn.RegisterEventCallback(MMCPP_EMCY,(void*)Emergency_Received) ;
	//
	// Initialize default parameters of single axis. This is not a must. Each parameter may be initialized individually.
	stSingleDefault.fEndVelocity	= 0 ;
	stSingleDefault.dbDistance 		= 100000 ;
	stSingleDefault.dbPosition 		= 0 ;
	stSingleDefault.fVelocity 		= 100000 ;
	stSingleDefault.fAcceleration 	= 1000000 ;
	stSingleDefault.fDeceleration 	= 1000000 ;
	stSingleDefault.fJerk 			= 20000000 ;
	stSingleDefault.eDirection 		= MC_POSITIVE_DIRECTION ;
	stSingleDefault.eBufferMode 	= MC_BUFFERED_MODE ;
	stSingleDefault.ucExecute 		= 1 ;
	//
	// Initialize default parameters of group axis.
	memset(stGroupDefault.dAuxPoint, 0, sizeof (double)*NC_MAX_NUM_AXES_IN_NODE);
	memset(stGroupDefault.dEndPoint, 0, sizeof (double)*NC_MAX_NUM_AXES_IN_NODE);
	memset(stGroupDefault.fTransitionParameter, 0, sizeof (float)*NC_MAX_NUM_AXES_IN_NODE);
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
	a_axis.InitAxisData("a_axis",gConnHndl) ;
	b_axis.InitAxisData("b_axis",gConnHndl) ;
	//v1.InitAxisData("v01",gConnHndl);
	//
	// Set default motion parameters. TODO: Update for all axes.
	a_axis.SetDefaultParams(stSingleDefault) ;
	b_axis.SetDefaultParams(stSingleDefault) ;
	//v1.SetDefaultParams(stGroupDefault);
	//

	// You may of course change internal parameters manually:
	a_axis.m_fAcceleration = 960000.0;
	a_axis.m_fDeceleration = 960000.0;
	a_axis.m_fVelocity = 640000.0;
	//a2.m_fAcceleration = 1621333.0;
	//a2.m_fDeceleration = 1621333.0;
	//a2.m_fVelocity = 1013333.0;
	//

	giXStatus 	= a_axis.ReadStatus() ;
	if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a_axis.Reset() ;
		giXStatus 	= a_axis.ReadStatus() ;
		if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a_axis in Error Stop. Aborting." ;
			exit(0) ;
		}
	}
	//
	giYStatus 	= b_axis.ReadStatus() ;
	if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		b_axis.Reset() ;
		giYStatus 	= b_axis.ReadStatus() ;
		if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a2 in Error Stop. Aborting." ;
			exit(0) ;
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
void MainClose()
{
//
//	Here will come code for all closing processes
//
	a_axis.Stop(MC_BUFFERED_MODE);
	b_axis.Stop(MC_BUFFERED_MODE);
	a_axis.PowerOff(MC_BUFFERED_MODE);
	b_axis.PowerOff(MC_BUFFERED_MODE);

	while( NC_AXIS_DISABLED_MASK & giXStatus)
		giXStatus = a_axis.ReadStatus();
	while( NC_AXIS_DISABLED_MASK & giYStatus)
		giYStatus = b_axis.ReadStatus();


	a_axis.SetOpMode(OPM402_PROFILE_POSITION_MODE);
	giXOpMode =  a_axis.GetOpMode();
	while ( giXOpMode != OPM402_PROFILE_POSITION_MODE)
	{
		//a_axis.SetOpMode(OPM402_PROFILE_VELOCITY_MODE);
		giXOpMode =  a_axis.GetOpMode();
	}

	MMC_CloseConnection(gConnHndl) ;
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
int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock)
{
	// Which function ID was received ...
	switch(recvBuffer[1])
	{
	case EMCY_EVT:
		//
		// Please note - The emergency event was registered.
		// printf("Emergency Event received\r\n") ;
		break ;
	case ASYNC_REPLY_EVT:
//		printf("buffer size: %d  ", recvBufferSize);
//		for (int i = 0; i < recvBufferSize; i++)
//			cout << recvBuffer[i];
//		cout << endl;
		break;
	case MOTIONENDED_EVT:
		printf("Motion Ended Event received\r\n") ;
		break ;
	case HBEAT_EVT:
		printf("H Beat Fail Event received\r\n") ;
		break ;
	case PDORCV_EVT:
		printf("PDO Received Event received - Updating Inputs\r\n") ;
		break ;
	case DRVERROR_EVT:
		printf("Drive Error Received Event received\r\n") ;
		break ;
	case HOME_ENDED_EVT:
		printf("Home Ended Event received\r\n") ;
		break ;
	case SYSTEMERROR_EVT:
		printf("System Error Event received\r\n") ;
		break ;
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
	return 1 ;
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
int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
{
	MMC_CloseConnection(uiConnHndl);
	printf("MMCPPExitClbk: Run time Error in function %s, axis ref=%d, err=%d, status=%d, bye\n", msg, usAxisRef, sErrorID, usStatus);
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
void TerminateApplication(int iSigNum)
{
	//
	printf("In Terminate Application ...\n");
	giTerminate = 1 ;
	sigignore(SIGALRM);
	//
	switch(iSigNum)
	{
		// Handle ctrl+c.
		case SIGINT:
			// TODO Close what needs to be closed before program termination.
			exit(0);
			break;
		default:
			break;
	}
	return ;
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
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
	printf("Emergency Message Received on Axis %d. Code: %x\n",usAxisRef,sEmcyCode) ;
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
void ChangeToRelevantMode()
{
	double dConnectionType;
	//
	dConnectionType = cConn.GetGlobalBoolParameter(MMC_CONNECTION_TYPE_PARAM , 0);

	//
	// ETHERCAT
	if( dConnectionType == eCOMM_TYPE_ETHERCAT )
	{
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
	else
	{
		a_axis.SetOpMode(OPM402_PROFILE_POSITION_MODE);
		// Waiting for Set Operation Mode
		//
		giXOpMode =  a_axis.GetOpMode();
		while ( giXOpMode != OPM402_PROFILE_POSITION_MODE)
		{
			//a_axis.SetOpMode(OPM402_PROFILE_VELOCITY_MODE);
			giXOpMode =  a_axis.GetOpMode();
		}

		b_axis.SetOpMode(OPM402_PROFILE_POSITION_MODE);
		//
		// Waiting for Set Operation Mode
		giYOpMode =  b_axis.GetOpMode();
		while ( giYOpMode != OPM402_PROFILE_POSITION_MODE )
			giYOpMode =  b_axis.GetOpMode();
	}
}


//
//
