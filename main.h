/*
 ============================================================================
 Name : SBOT2_GROUPCONTROL.cpp
 Author :	Elmo Motion Control.
 Version :
 Description : 	GMAS C/C++ project header file for Template
 ============================================================================
 */

/*
 ============================================================================
 Project general functions prototypes
 ============================================================================
 */
void MainInit();
void MainClose();
int OnRunTimeError(const char *msg, unsigned int uiConnHndl,
		unsigned short usAxisRef, short sErrorID, unsigned short usStatus);
void TerminateApplication(int iSigNum);
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode);
int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize, void* lpsock);
void ChangeOpMode();
void executeInput(CMMCSingleAxis& axis, string input);
void usage();
int startServer();
int getClientParams(int new_socket, int& torque_mA, int& sampling_ms );
/*
 ============================================================================
 General constants
 ============================================================================
 */
#define 	MAX_AXES				2		// number of Physical axes in the system. TODO Update MAX_AXES accordingly
#define 	UM_TORQUE_CONTROL_LOOP 		1
#define 	UM_SPEED_CONTROL_LOOP		2
#define		UM_POSITION_CONTROL_LOOP	5

#define 	SYNC_MULTIPLIER				1
/*
 ============================================================================
 Application global variables
 ============================================================================
 */
int giTerminate; // Flag to request program termination
// 	Examples for data read from the GMAS core about the X, Y drives
int lift_status;
int load_status;
int giGroupStatus;
int loadOpMode;
int liftOpMode;


int new_socket;
char buffer[1024] = { 0 };
//
/*
 ============================================================================
 Global structures for Elmo's Function Blocks
 ============================================================================
 */
MMC_CONNECT_HNDL gConnHndl; // Connection Handle
CMMCConnection cConn;
CMMCSingleAxis load_axis, lift_axis; // TODO : Update the names and number of the axes in the system
CMMCGroupAxis v1;
MMC_MOTIONPARAMS_SINGLE stSingleDefault; // Single axis default data
MMC_MOTIONPARAMS_GROUP stGroupDefault; // Group axis default data
