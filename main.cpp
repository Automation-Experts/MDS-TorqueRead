/*
============================================================================
 Name : 	CPP_DemoCAN.cpp
 Author :	Benjamin Spitzer
 Version :	1.00
 Description : The following example supports the following functionalities:


The following features are demonstrated for a CAN network (Due to SYNC and PDO initializations):

- Two separate state machines for handling parallel motions / sequences.
- Modbus callback registration.
- Emergency callback registration.
- PDO3 and SYNC initializations.
- Modbus reading and updates of axis status and positions.
- Point to Point motion state machine

 The program works with 2 axes - a01 and a02.
 For the above functions, the following modbus 'codes' are to be sent to address 40001:

 	 - Point to Point 	- 1. Simple Point to Point motion on axis a01.


============================================================================
*/
#include "mmc_definitions.h"
#include "mmcpplib.h"
#include "main.h"			// Application header file.
#include <iostream>
#include <sys/time.h>			// For time structure
#include <signal.h>				// For Timer mechanism
/*
============================================================================
 Function:				main()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 The main function of this sample project.
============================================================================
*/

int main()
{
	//
	//	Initialize system, axes and all needed initializations
	//
	MainInit();
	//
	//	Execute the state machine to handle the system sequences and control
	//
	MachineSequences();
	//
	//	Close what needs to be closed before program termination
	//
	MainClose();
	//
	return 1;		// Terminate the application program back to the Operating System
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
//
//	Here will come initialization code for all system, axes, communication etc.
//
// 	InitializeCommunication to the GMAS:
//
	int iRes ;
	float fRes ;
	appTimeout = 0;
	//
	gConnHndl = cConn.ConnectIPCEx(0x7fffffff,(MMC_MB_CLBK)CallbackFunc) ;
	//
	// Start the Modbus Server:
	//cHost.MbusStartServer(gConnHndl,1) ;
	//
	// Register Run Time Error Callback function
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);
	//
	// Register the callback function for Modbus and Emergency:
	//cConn.RegisterEventCallback(MMCPP_MODBUS_WRITE,(void*)ModbusWrite_Received) ;
	//cConn.RegisterEventCallback(MMCPP_EMCY,(void*)Emergency_Received) ;
	//
	//
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
	// 	TODO: Update number of necessary axes:
	//
	a1.InitAxisData("a01",gConnHndl) ;
	//a2.InitAxisData("a02",gConnHndl) ;
	//
	// Set default motion parameters. TODO: Update for all axes.
	a1.SetDefaultParams(stSingleDefault) ;
	//a2.SetDefaultParams(stSingleDefault) ;
	//
	giXStatus 	= a1.ReadStatus() ;
	if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a1.Reset() ;
		sleep(1) ;
		giXStatus 	= a1.ReadStatus() ;
		if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a1 in Error Stop. Aborting." ;
			exit(0) ;
		}
	}
	cout << "debug 1" << endl;
//	giYStatus 	= a2.ReadStatus() ;
//	if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
//	{
//		a2.Reset() ;
//		sleep(1) ;
//		giYStatus 	= a2.ReadStatus() ;
//		if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
//		{
//			cout << "Axis a2 in Error Stop. Aborting." ;
//			exit(0) ;
//		}
//	}
	//
	// Initialize PDOs and SYNC's in the system:
	// PDO 3, Group 1, OnSync:

	a1.ConfigPDO(PDO_NUM_3,PDO_PARAM_REG,NC_COMM_EVENT_GROUP1,1,1,1,1,1) ;
	CMMCPPGlobal::Instance()->SetSyncTime(gConnHndl, SYNC_MULTIPLIER) ;
	//
//	iRes = 5 ;
//	// Set UM to 5:
//	a1.ElmoSetAsyncParam("UM",iRes) ;
//	a2.ElmoSetAsyncParam("UM",iRes) ;
//	//
//	// Download UF to drive example:
//	fRes = 12.3 ;
//	a1.ElmoSetAsyncArray("UF",1,fRes) ;
//	fRes = 0.0 ;
//	a1.ElmoGetSyncArray("UF",1,fRes);
	//
	// Clear the modbus memory array:
	//memset(mbus_write_in.regArr,0x0,250) ;
	cout << "debug 2" << endl;
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
	//cHost.MbusStopServer() ;
	MMC_CloseConnection(gConnHndl) ;
	return;
}
/*
============================================================================
 Function:				MachineSequences()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Starts the Main Timer function that will execute the states machines
 to control the system. Also performs a slow background loop for
 less time-critical background processes and monitoring of requests
 to terminate the application.
============================================================================
*/
void MachineSequences()
{
//
//	Init all variables of the states machines
//
	cout << "debug 3" << endl;
	MachineSequencesInit();
//
//	Enable MachineSequencesTimer() every TIMER_CYCLE ms
//
	EnableMachineSequencesTimer(TIMER_CYCLE);
//
//	Background loop. Handles termination request and other less time-critical background proceses
//
	while (!giTerminate)
	{
		//MachineSequencesTimer(0);
//
//		Execute background process if required
//
		BackgroundProcesses();
//
//		Sleep for ~SLEEP_TIME micro-seconds to reduce CPU load
//
		usleep(SLEEP_TIME);
	}
//
//	Termination requested. Close what needs to be cloased at the states machines
//
	MachineSequencesClose();

	return;		// Back to the main() for program termination
}
/*
============================================================================
 Function:				MachineSequencesInit()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Initilaize the states machines variables
============================================================================
*/
void MachineSequencesInit()
{
//
//	Initializing all variables for the states machines
//
	giTerminate 	= FALSE;

	giState1 		= eSM1;
	giPrevState1 	= eIDLE;
	giSubState1 	= eIDLE;
	//
	giState2 		= eIDLE;
	giPrevState2 	= eIDLE;
	giSubState2 	= eIDLE;

	giReentrance = FALSE;

	return;
}
/*
============================================================================
 Function:				MachineSequencesClose()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Close all that needs to be closed at the states machines before the
 application program is terminated.
============================================================================
*/
void MachineSequencesClose()
{
//
//	Here will come code for all closing processes
//
	return;
}
/*
============================================================================
 Function:				BackgroundProcesses()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Performs background processes that are not managed by the States Machines.
 This can be saving logs to the FLASH, performing background calculations etc.
 Generally speaking, this function, although colored in red in the Close all that needs to be closed at the states machines before the
 application program is terminated.
============================================================================
*/
void BackgroundProcesses()
{
//
//	Here will come code for all closing processes
//
	if (appTimeout++ > 10 * 100)
	{
		cout << "App has timed out." << endl;
		giTerminate = true;
	}
	return;
}
/*
============================================================================
 Function:				EnableMachineSequencesTimer()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Enables the main machine sequences timer function, to be executed each
 TIMER_CYCLE ms.
============================================================================
*/
void EnableMachineSequencesTimer(int TimerCycle)
{
	struct itimerval timer;
	struct sigaction stSigAction;

	// Whenever a signal is caught, call TerminateApplication function
	stSigAction.sa_handler = TerminateApplication;

	sigaction(SIGINT, &stSigAction, NULL);
	sigaction(SIGTERM, &stSigAction, NULL);
	sigaction(SIGABRT, &stSigAction, NULL);
	sigaction(SIGQUIT, &stSigAction, NULL);
//
//	Enable the main machine sequences timer function
//
	timer.it_interval.tv_sec 	= 0;
	timer.it_interval.tv_usec 	= TimerCycle * 1000;// From ms to micro seconds
	timer.it_value.tv_sec 		= 0;
	timer.it_value.tv_usec 		= TimerCycle * 1000;// From ms to micro seconds

	setitimer(ITIMER_REAL, &timer, NULL);

	signal(SIGALRM, MachineSequencesTimer); 		// every TIMER_CYCLE ms SIGALRM is received which calls MachineSequencesTimer()

	return;
}
/*
============================================================================
 Function:				MachineSequencesTimer()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 A timer function that is called by the OS every TIMER_CYCLE ms.
 It executes the machine sequences states machines and actully controls
 the sequences and behavior of the machine.
============================================================================
*/
void MachineSequencesTimer(int iSig)
{
//
//	In case the application is waiting for termination, do nothing.
//	This can happen if giTerminate has been set, but the background loop
//	didn't handle it yet (it has a long sleep every loop)
//
	if (giTerminate == TRUE) return;
//
//	Avoid reentrance of this time function
//
//	Reentrance can theoretically happen if the execution of this timer function
//	is wrongly longer than TIMER_CYCLE. In such case, reentrance should be avoided
//	to prevent unexpected behavior (the code is not designed for reentrance).
//
//	In addition, some error handling should be taken by the user.
//
	if (giReentrance)
	{
//
//		Print an error message and return. Actual code should take application related error handling
//
		printf("Reentrancy!\n");

		return;
	}

	giReentrance = TRUE;		// to enable detection of reentrancy. The flag is cleared at teh end of this function
//
//	Read all input data.
//
//	Here, every TIMER_CYCLE ms, the user should read all input data that may be
//	required for the states machine code and copy them into "mirror" variables.
//
//	The states machines code, below, should use only the mirror variables, to ensure
//	that all input data is synchronized to the timer event.
//
//	Input data can be from the Host (MODBUS) or from the drives or I/Os units
//	(readingfrom the GMAS core firmware using one of the Function Blocks library
//	functions) or from any other source.
//
	ReadAllInputData();
/*
============================================================================

	States Machines code starts here!

============================================================================
*/

//
//	In case it is a new state value, clear also the value of the sub-state
//	to ensure it will start from its beginning (from the first ssub-state)
//
	if(giTempState1 != eIDLE)
	{
		giState1  = giTempState1 ;
	}
	if(giTempState2 != eIDLE)
	{
		giState2  = giTempState2 ;
	}

	if (giState1 != giPrevState1)
	{
		giSubState1 	= FIRST_SUB_STATE;
		giPrevState1 	= giState1;
//		mbus_write_in.startRef 		= 0	;       // Reset Current state we are running on in Modbus so we do not return to it.
//		mbus_write_in.refCnt 		= 1	;
//		mbus_write_in.regArr[0] 	= 0;
		//cHost.MbusWriteHoldingRegisterTable(mbus_write_in) ;
	}
//
	if (giState2 != giPrevState2)
	{
		giSubState2 	= FIRST_SUB_STATE;
		giPrevState2 	= giState2;
	}
//	Handle the main state machine.
//
//	The value of the State variable is used to make decisions of the main states machine and to call,
//	as necessary, the relevant function that handles to process itslef in a sub-state machine.
//
	switch (giState1)
	{
//
//		Do nothing, waiting for commands
//
		case eIDLE:
		{
			break;
		}
//
//		Do State Machine1
//
		case eSM1:
		{
			StateFunction_1();			// calls a sub-state machine function to handle this proocess
			break;
		}
//
//		Do State Machine2
//
		case eSM2:
		{
			StateFunction_2();			// calls a sub-state machine function to handle this proocess
			break;
		}
//
//		The default case. Should not happen, the user can implement error handling.
//
		default:
		{
			break;
		}
	}

	// 2nd state machine.
		switch (giState2)
		{
	//
	//		Do nothing, waiting for commands
	//
			case eIDLE:
			{
				break;
			}
	//
	//		Do State Machine1
	//
			case eSM1:
			{
				StateFunction_1();			// calls a sub-state machine function to handle this proocess
				break;
			}
	//
	//		Do State Machine2
	//
			case eSM2:
			{
				StateFunction_2();			// calls a sub-state machine function to handle this proocess
				break;
			}
	//
	//		The default case. Should not happen, the user can implement error handling.
	//
			default:
			{
				break;
			}
		}

//
//	Write all output data
//
//	Here, every TIMER_CYCLE ms, after the execution of all states machines
//	(based on all the Input Data as read from all sources at teh top of this function)
//	the user should write all output data (written into mirror variables within the
//	states machines code) to the "external world" (MODBUS, GMAS FW core, ...).
//
//	After alll states machines were executed and generated their outputs, the outputs
//	are writen to the "external world" to actually execute the states machines "decisions"
//
	WriteAllOutputData();
//
//	Clear the reentrancy flag. Now next execution of this function is allowed
//
	giReentrance = FALSE;
//
	return;		// End of the sequences timer function. The function will be triggered again upon the next timer event.
}
/*
============================================================================
 Function:				ReadAllInputData()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 The data used during the processing of the states machines must be synchronized with the
 timer event and must be "frozen" during the processing of all states machines code, so that
 all decisions and calculations will use the same data.

 This function is called at the beginning of the timer function, each TIMER_CYCLE ms,
 to collect all required input data (as relevant to each application) and to copy
 it into "mirror" variables, that will be used by the state machines code.
 ============================================================================
*/
void ReadAllInputData()
{
	MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT 	mbus_read_out;
//
//	Here should come the code to read all required input data, for instance:
//
//	giXInMotion = ...
//	giYInMotion = ...
//
// The data read here may arrive from different sources:
// 	- Host Communication (Modbus, Ethernet-IP. This can be read on a cyclic basis, or from a callback.
//	- GMAS Firmware. Such as actual positions, torque, velocities.

	// TODO: Change the number of registers to read.
	//cHost.MbusReadHoldingRegisterTable(MODBUS_READ_OUTPUTS_INDEX,MODBUS_READ_CNT,mbus_read_out) ;

	//giTempState1= (mbus_read_out.regArr[1] << 16 & 0xFFFF0000) | (mbus_read_out.regArr[0] & 0xFFFF);
	//giTempState2= (mbus_read_out.regArr[3] << 16 & 0xFFFF0000) | (mbus_read_out.regArr[2] & 0xFFFF);
	//
	giXStatus 	= a1.ReadStatus() ;
	//giYStatus 	= a2.ReadStatus() ;
	giXPos 		= (int)a1.GetActualPosition() ;
	//giYPos 		= (int)a2.GetActualPosition() ;
	return;
}
/*
============================================================================
 Function:				WriteAllOutputData()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Write all the data, generated by the states machines code, to the "external
 world".

 This is done here (and not inside the states machines code) to ensure that
 all data is updated simultaneously and in a synchronous way.

 The states machines code write the data into "mirror" variables, that are here
 copied or sent to the "external world" (Host via MODBUS, GMAS core firmware, etc.)
  ============================================================================
*/
void WriteAllOutputData()
{
//
//	Here should come the code to write/send all ouput data
//
//	mbus_write_in.startRef 		= MODBUS_UPDATE_START_INDEX	;       // index of start write modbus register.
//	mbus_write_in.refCnt 		= MODBUS_UPDATE_CNT			;		// number of indexes to write
	//
//	InsertLongVarToModbusShortArr(&mbus_write_in.regArr[0],  (long) giXPos) ;
//	InsertLongVarToModbusShortArr(&mbus_write_in.regArr[2],  (long) giYPos) ;
	//
//	cHost.MbusWriteHoldingRegisterTable(mbus_write_in) ;
	return;
}
/*
============================================================================
 Function:				StateFunction_1()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 A sub-states machine function. This function executes the sub-states machine
 of the Function1 process.

 For instance, a homing state machine will consist of:

	 Change Operation Mode.
	 Power Enable
	 Start homing - method number n
	 Wait for end of homing

 Each step is handled by a dedicated function. However, calling a function
 is not a must and the relevant code for each sub-state can be directly
 written within the switch-case structure.
============================================================================
*/
void StateFunction_1()
{
//
//	Handle the sub-state machine.
//
//	The value of the Sub-State variable is used to make decisions of the sub-states machine and to call,
//	as necessary, the relevant function that handles to process itself.
//
	switch (giSubState1)
	{
//
//		Perform sub SM 1
//
		case eSubState_SM1_PowerOn:
		{
			SubState1_1PowerOn();
			break;
		}
//
//		Perform sub SM 2
//
		case eSubState_SM1_WPowerOn:
		{
			SubState1_2WPowerOn();
			break;
		}
//
//		Perform sub SM 3
//
		case eSubState_SM1_Move1:
		{
			SubState1_3Function();
			break;
		}
//
//		Perform sub SM 4
//
		case eSubState_SM1_WMove1:
		{
			SubState1_4Function();
			break;
		}
		case eSubState_SM1_Move2:
			//
			// Change acceleration:
			a1.m_fAcceleration 	= 50000.0 ;
			a1.MoveAbsolute(0.0,2000,MC_ABORTING_MODE) ;
			giSubState1 		= eSubState_SM1_WMove2 ;
			break ;
		case eSubState_SM1_WMove2:
			if (giXStatus & NC_AXIS_STAND_STILL_MASK)
			{
				a1.PowerOff() ;
				//a2.PowerOff() ;
				giState1 = eIDLE;
			}
			break ;
//
//		The default case. Should not happen, the user can implement error handling.
//
		default:
		{
			break;
		}
	}
//
	return;
}
/*
============================================================================
 Function:				StateFunction_2()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 A sub-states machine function. This function executes the sub-states machine
 of the XY move process.

 The move prcess, in this simplified example consists of the following steps:

 Begin move
 Wait for end of motion

 Each step is handled by a dedicated function. However, calling a function
 is not a must and the relevant code for each sub-state can be directly
 written within the switch-case structure.
============================================================================
*/
void StateFunction_2()
{
//
//	Handle the sub-state machine.
//
//	The value of the Sub-State variable is used to make decisions of the sub-states machine and to call,
//	as necessary, the relevant function that handles to process itslef.
//
	switch (giSubState1)
	{
//
//		Begin move
//
		case eSubState_SM2_1:
		{
			SubState2_1Function();
			break;
		}
//
//		Wait for end of motion
//
		case eSubState_SM2_2:
		{
			SubState2_2Function();
			break;
		}
//
//		The default case. Should not happen, the user can implement error handling.
//
		default:
		{
			break;
		}
	}
//
	return;
}
/*
============================================================================
 Function:				SubState1_1Function()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Starts a motion to the reverse limits of X and Y, as part of the
 states machine code for X,Y Homing, and chage the sub state to wait for the
 limits.
 ============================================================================
*/
void SubState1_1PowerOn()
{
//
//	Here will come the code to start the relevant motions
//
	a1.PowerOn() ;
	//a2.PowerOn() ;
//
//	Changing to the next sub-state
//
	giSubState1 = eSubState_SM1_WPowerOn;

	return;
}
/*
============================================================================
 Function:				SubState1_2Function()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Waits for X and Y limits to be in motion - only then move to the next sub-state.
  ============================================================================
*/
void SubState1_2WPowerOn()
{
//
//	Changing to the next sub-state only if axis changed to standstill.
//
//	Note that a faster implementation could be to put here the code of the next sub-state as well.

	if (giXStatus & NC_AXIS_STAND_STILL_MASK)
	{
		giSubState1 = eSubState_SM1_Move1;
	}

	return;
}
/*
============================================================================
 Function:				SubState1_3Function()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Starts a motion to the index of X and Y, as part of the
 states machine code for X,Y Homing, and chage the sub state to wait for the
 indexes.
 ============================================================================
*/
void SubState1_3Function()
{
//
//	Here will come the code to start the relevant motions
//
	a1.MoveAbsolute(10000.0,2000,MC_ABORTING_MODE) ;
//
//	Changing to the next sub-state
//
	giSubState1 = eSubState_SM1_WMove1;

	return;
}
/*
============================================================================
 Function:				SubState1_4Function()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Waits for X and Y motions to end.
  ============================================================================
*/
void SubState1_4Function()
{
//
//	Ending state machine only if both axes are not in motion.
//
	if (giXStatus & NC_AXIS_STAND_STILL_MASK)
	{
		giSubState1 = eSubState_SM1_Move2;
	}

	return;
}
/*
============================================================================
 Function:				SubState2_1Function()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Starts an XY motion as part of the states machine code for X,Y move,
 and chage the sub state to wait for the end of motions.
 ============================================================================
*/
void SubState2_1Function()
{
//
//	Here will come the code to start the relevant motions
//
//
//	Changing to the next sub-state
//
	giSubState1 = eSubState_SM1_WMove2;

	return;
}
/*
============================================================================
 Function:				SubStateXYMoveWaitEndMotionFunction()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Waits for X and Y to stop motions and only then finishes the XY move process.
  ============================================================================
*/
void SubState2_2Function()
{
//
//	Ending X,Y move only if both indexes are activated.
//
	if ( (~giXInMotion) && (~giYInMotion) )
	{
		giState1 = eIDLE;
	}

	return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Function name	:	void callback function																		//
//	Created			:	Version 1.00																				//
//	Updated			:	3/12/2010																					//
//	Modifications	:	N/A																							//
//	Purpose			:	interuprt function 																			//
//																													//
//	Input			:	N/A																							//
//	Output			:	N/A																							//
//	Return Value	:	int																							//
//	Modifications:	:	N/A																							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock)
{
	// Whcih function ID was received ...
	switch(recvBuffer[1])
	{
	case ASYNC_REPLY_EVT:
		printf("ASYNC event Reply\r\n") ;
		break ;
	case EMCY_EVT:
		// Please note - The emergency event was registered.
		// printf("Emergency Event received\r\n") ;
		break ;
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
	case MODBUS_WRITE_EVT:
		// TODO Update additional data tobe read such as function parameters.
		// TODO Remove return 0 if you want to handle as part of callback.
		return 0;
		printf("Modbus Write Event received - Updating Outputs\r\n") ;

		break ;
	}
	//
	return 1 ;
}

void InsertLongVarToModbusShortArr(short* spArr, long lVal)
{
	*spArr 		= (short) (lVal	& 0xFFFF);
	*(spArr + 1)= (short)((lVal >> 16) 	& 0xFFFF);
}

int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
{
	MMC_CloseConnection(uiConnHndl);
	printf("OnRunTimeError: %s,axis ref=%d, err=%d, status=%d, bye\n", msg, usAxisRef, sErrorID, usStatus);
	exit(0);
}


///////////////////////////////////////////////////////////////////////
//	Function name	:	void TerminateApplication(int iSigNum)
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
	// Wait till other threads exis properly.
	sleep(1) ;
	return ;
}

//
// Callback Function once a Modbus message is received.
void ModbusWrite_Received()
{
	printf("Modbus Write Received\n") ;
}
//
// Callback Function once an Emergency is received.
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
	printf("Emergency Message Received on Axis %d. Code: %x\n",usAxisRef,sEmcyCode) ;
}
