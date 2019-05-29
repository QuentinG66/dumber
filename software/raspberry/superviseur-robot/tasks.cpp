/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 15

#define PRIORITY_TREFRESHWD 99
#define TIME_REFRESH_WD 1000000000
#define TIME_PERIOD_CAMERA 100000000


#define RED "\033[91m"
#define GREEN "\033[92m"
#define BLUE "\033[34m"
#define YELLOW "\033[93m"
#define LPURPLE "\033[94m"
#define PURPLE "\033[95m"
#define RESET "\033[0m"

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */

/** 
 * Function which verify is robot output is Ok !
 * @return True if ok, else False
 */
bool isMsgFromRobotNOk(Message * msg){
	return (msg->CompareID(MESSAGE_ANSWER_COM_ERROR) || msg->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT) || msg->CompareID(MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND) || msg->CompareID(MESSAGE_ANSWER_COM_ERROR));
}

void Tasks::Init() {
//	cameraActive = false;
	int status;
	int err;
	//:		cameraActive = false;

	/**************************************************************************************/
	/* 	Mutex creation                                                                    */
	/**************************************************************************************/
	if (err = rt_mutex_create(&mutex_monitor, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_robot, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_move, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	/**
	 * Our code
	 **/

	if (err = rt_mutex_create(&mutex_watchdog, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	if (err = rt_mutex_create(&mutex_robotMsgLost, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	/** CAMERA **/
	if (err = rt_mutex_create(&mutex_cameraActive, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_dessinArene, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_calculPosition, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_msgCamera, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	if (err = rt_mutex_create(&mutex_camera, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_mutex_create(&mutex_areneOk, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	if (err = rt_mutex_create(&mutex_arena, NULL)) {
		cerr << "Error mutex create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	cout << "Mutexes created successfully" << endl << flush;

	/**************************************************************************************/
	/* 	Semaphors creation       							  */
	/**************************************************************************************/
	if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
		cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
		cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
		cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
		cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	if (err = rt_sem_create(&sem_camera, NULL, 1, S_FIFO)) {
		cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	/**
	 * Our code
	 **/

	cout << "Semaphores created successfully" << endl << flush;

	/**************************************************************************************/
	/* Tasks creation                                                                     */
	/**************************************************************************************/
	if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
		cerr << "Error task create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
		cerr << "Error task create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
		cerr << "Error task create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
		cerr << "Error task create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
		cerr << "Error task create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
		cerr << "Error task create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	/**
	 * Our code
	 **/
	if (err = rt_task_create(&th_refreshWD, "th_refreshWD", 0, PRIORITY_TREFRESHWD, 0)) {
		cerr << "Error task create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	/**** Camera task ***/
	if (err = rt_task_create(&th_requestCam, "th_requestCam", 0, PRIORITY_TCAMERA, 0)) {
		cerr << "Error task create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_create(&th_periodicCam, "th_periodicCam", 0, PRIORITY_TCAMERA, 0)) {
		cerr << "Error task create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}


	cout << "Tasks created successfully" << endl << flush;

	/**************************************************************************************/
	/* Message queues creation                                                            */
	/**************************************************************************************/

	if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*5000, Q_UNLIMITED, Q_FIFO)) < 0) {
		cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if ((err = rt_queue_create(&q_msgCamera, "q_msgCamera", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
		cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
	rt_task_set_priority(NULL, T_LOPRIO);
	int err;

	if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
		cerr << "Error task start: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
		cerr << "Error task start: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
		cerr << "Error task start: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
		cerr << "Error task start: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
		cerr << "Error task start: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
		cerr << "Error task start: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	/** Our code **/
	if (err = rt_task_start(&th_requestCam, (void(*)(void*)) & Tasks::CameraRequest, this)) {
		cerr << "Error task start: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}
	if (err = rt_task_start(&th_periodicCam, (void(*)(void*)) & Tasks::CameraPeriodic, this)) {
		cerr << "Error task start: " << strerror(-err) << endl << flush;
		exit(EXIT_FAILURE);
	}

	cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
	monitor.Close();
	robot.Close();
}

/**
 */
void Tasks::Join() {
	cout << "Tasks synchronized" << endl << flush;
	rt_sem_broadcast(&sem_barrier);
	pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
	int status;

	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
	// Synchronization barrier (waiting that all tasks are started)
	rt_sem_p(&sem_barrier, TM_INFINITE);

	/**************************************************************************************/
	/* The task server starts here                                                        */
	/**************************************************************************************/
	rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
	status = monitor.Open(SERVER_PORT);
	rt_mutex_release(&mutex_monitor);

	cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

	if (status < 0) throw std::runtime_error {
		"Unable to start server on port " + std::to_string(SERVER_PORT)
			// Server can't be launched
	};
	monitor.AcceptClient(); // Wait the monitor client
	cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
	rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
	Message *msg;

	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
	// Synchronization barrier (waiting that all tasks are starting)
	rt_sem_p(&sem_barrier, TM_INFINITE);

	/**************************************************************************************/
	/* The task sendToMon starts here                                                     */
	/**************************************************************************************/
	rt_sem_p(&sem_serverOk, TM_INFINITE);

	while (1) {
		msg = ReadInQueue(&q_messageToMon);
		cout << YELLOW << "[#Monitor] -> " << RESET << "Send msg to mon: " << msg->ToString() << endl << flush;
		rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
		monitor.Write(msg); // The message is deleted with the Write
		rt_mutex_release(&mutex_monitor);
	}
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
	Message *msgRcv;

	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
	// Synchronization barrier (waiting that all tasks are starting)
	rt_sem_p(&sem_barrier, TM_INFINITE);

	/**************************************************************************************/
	/* The task receiveFromMon starts here                                                */
	/**************************************************************************************/
	rt_sem_p(&sem_serverOk, TM_INFINITE);
	cout << YELLOW << "[#Monitor] : " << RESET << " Received message from monitor activated" << endl << flush;

	while (1) {
		msgRcv = monitor.Read();
		cout << YELLOW << "[#Monitor] <-" << RESET <<" Rcv <= " << msgRcv->ToString() << endl << flush;

		if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
			delete(msgRcv);
			exit(-1);
		} else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
			rt_sem_v(&sem_openComRobot);
		} else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
			rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
			watchdog = false;
			rt_mutex_release(&mutex_watchdog);
			rt_sem_v(&sem_startRobot);
		}else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)){
			rt_sem_v(&sem_startRobot);	
			rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
			watchdog = true;
			rt_mutex_release(&mutex_watchdog);
		} else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) 	||
				msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) 				||
				msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) 						||
				msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) 					||
				msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

			rt_mutex_acquire(&mutex_move, TM_INFINITE);
			move = msgRcv->GetID();
			rt_mutex_release(&mutex_move);
		}else if (msgRcv->CompareID(MESSAGE_CAM_OPEN) 									||
							msgRcv->CompareID(MESSAGE_CAM_CLOSE)									||
							msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)							||
							msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)					||
							msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM) 					||
							msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START) ||
							msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){
			rt_mutex_acquire(&mutex_msgCamera, TM_INFINITE);
			WriteInQueue(&q_msgCamera, msgRcv);
			rt_mutex_release(&mutex_msgCamera);
		}else{
			cout << msgRcv->ToString() << endl << flush;
		}
		delete(msgRcv); // mus be deleted manually, no consumer
	}
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
	int status;
	int err;

	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
	// Synchronization barrier (waiting that all tasks are starting)
	rt_sem_p(&sem_barrier, TM_INFINITE);

	/**************************************************************************************/
	/* The task openComRobot starts here                                                  */
	/**************************************************************************************/
	while (1) {
		rt_sem_p(&sem_openComRobot, TM_INFINITE);
		rt_mutex_acquire(&mutex_robot, TM_INFINITE);
		status = robot.Open();
		rt_mutex_release(&mutex_robot);
		cout << "Open serial com (" << status << ")" << endl << flush;

		Message * msgSend;
		if (status < 0) {
			msgSend = new Message(MESSAGE_ANSWER_NACK);
		} else {
			msgSend = new Message(MESSAGE_ANSWER_ACK);
		}
		WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
	}
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
	int err;
	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
	// Synchronization barrier (waiting that all tasks are starting)
	rt_sem_p(&sem_barrier, TM_INFINITE);

	/**************************************************************************************/
	/* The task startRobot starts here                                                    */
	/**************************************************************************************/
	while (1) {

		Message * msgSend;
		rt_sem_p(&sem_startRobot, TM_INFINITE);

		rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
		rt_mutex_acquire(&mutex_robot, TM_INFINITE);
		if (watchdog){
			cout << "Start robot with watchdog (";
			msgSend = robot.Write(robot.StartWithWD());
			/**Start refresh wd **/
			if (err = rt_task_start(&th_refreshWD, (void(*)(void*)) & Tasks::RefreshWDTask, this)) {
				cerr << "Error task start: " << strerror(-err) << endl << flush;
				exit(EXIT_FAILURE);
			}
		}else{
			cout << "Start robot without watchdog (";
			msgSend = robot.Write(robot.StartWithoutWD());
		}

/*		if (isMsgFromRobotNOk(msgSend)){
			rt_mutex_acquire(&mutex_robotMsgLost,TM_INFINITE);
			robotMsgLost += 1;
			if (not(robotMsgLost < robotMaxMsgLost)){
				cout << RED << "[#Robot] " << RESET << "Error msg robot" << RESET << endl << flush;
				robot.Close();
			}
		}else{
			rt_mutex_acquire(&mutex_robotMsgLost,TM_INFINITE);
			robotMsgLost = 0;
			rt_mutex_release(&mutex_robotMsgLost);
		}*/

		rt_mutex_release(&mutex_watchdog);
		rt_mutex_release(&mutex_robot);
		cout << msgSend->ToString();
		cout << ")" << endl << flush;

//		cout << "Movement answer: " << msgSend->ToString() << endl << flush;
		WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

		if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
			rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
			robotStarted = 1;
			rt_mutex_release(&mutex_robotStarted);
		}
	}
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
	int rs;
	int cpMove;
	int batteryAsk = 0; 
	MessageBattery * msg;
	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
	// Synchronization barrier (waiting that all tasks are starting)
	rt_sem_p(&sem_barrier, TM_INFINITE);

	/**************************************************************************************/
	/* The task starts here                                                               */
	/**************************************************************************************/
	rt_task_set_periodic(NULL, TM_NOW, 100000000);

	while (1) {
		rt_task_wait_period(NULL);
		rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
		rs = robotStarted;
		rt_mutex_release(&mutex_robotStarted);
		if (rs == 1) {
			Message * fromRobot;
			rt_mutex_acquire(&mutex_move, TM_INFINITE);
			cpMove = move;
			rt_mutex_release(&mutex_move);

			rt_mutex_acquire(&mutex_robot, TM_INFINITE);
			fromRobot = robot.Write(new Message((MessageID)cpMove));

			cout << YELLOW << "[#Monitor] : " << RESET << "Periodic move : " << cpMove << endl << flush;

			/*if (isMsgFromRobotNOk(fromRobot)){
				rt_mutex_acquire(&mutex_robotMsgLost,TM_INFINITE);
				robotMsgLost += 1;
				if (not(robotMsgLost < robotMaxMsgLost)){
					cout << RED << "[#Robot] " << RESET << "Error msg robot" << RESET << endl << flush;
					robot.Close();
				}
			}else{
				rt_mutex_acquire(&mutex_robotMsgLost,TM_INFINITE);
				robotMsgLost = 0;
				rt_mutex_release(&mutex_robotMsgLost);
			}*/
			delete(fromRobot);
/*
			batteryAsk += 1;
			if (batteryAsk == 5){	 
				msg = (MessageBattery*)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
			*	if (isMsgFromRobotNOk(msg)){
					rt_mutex_acquire(&mutex_robotMsgLost,TM_INFINITE);
					robotMsgLost += 1;
					if (not(robotMsgLost < robotMaxMsgLost)){
						cout << RED << "[#Robot] " << RESET << "Error msg robot" << RESET << endl << flush;
						robot.Close();
					}
				}else{
					rt_mutex_acquire(&mutex_robotMsgLost,TM_INFINITE);
					robotMsgLost = 0;
					rt_mutex_release(&mutex_robotMsgLost);
				}

				WriteInQueue(&q_messageToMon, msg);  // msgSend will be deleted by sendToMon
				batteryAsk = 0;
			}*/
			rt_mutex_release(&mutex_robot);
		}
	}
}

void Tasks::RefreshWDTask(void *arg){
	int sem_status;
	int rs;
	Message * wd;
	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

	rt_task_set_periodic(NULL, TM_NOW, TIME_REFRESH_WD);
	while (1){
		rt_task_wait_period(NULL);
		rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
		rs = robotStarted;
		rt_mutex_release(&mutex_robotStarted);
		if (rs == 1){
			robot.Write(robot.ReloadWD());	
		}
/*			cout << BLUE << "[#Robot] :" << RESET << "Watchdog : " << wd->ToString() << RESET << endl << flush;
			//delete(wd);
		}*/
	}
}
/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
	int err;
	if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
		cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
		throw std::runtime_error{"Error in write in queue"};
	}
}


void Tasks::CameraRequest(void * args){
	int rs = 1;
	int cpMsgCamera;
	Message * cachedMessage ;

	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
	// Synchronization barrier (waiting that all tasks are starting)
	rt_sem_p(&sem_barrier, TM_INFINITE);

	while (1){
		cachedMessage = ReadInQueue(&q_msgCamera);
		
		rt_mutex_acquire(&mutex_camera, TM_INFINITE);
		if (cachedMessage->CompareID(MESSAGE_CAM_OPEN)){
			rt_mutex_acquire(&mutex_cameraActive, TM_INFINITE);
			cameraActive = camera.Open();
			rt_mutex_release(&mutex_cameraActive);
		}else if (cachedMessage->CompareID(MESSAGE_CAM_CLOSE)){
			rt_mutex_acquire(&mutex_cameraActive, TM_INFINITE);
			cameraActive = false;
			camera.Close();
			rt_mutex_release(&mutex_cameraActive);
		}else if (cachedMessage->CompareID(MESSAGE_CAM_ASK_ARENA)){
			rt_mutex_acquire(&mutex_dessinArene, TM_INFINITE);
			dessinArene = true;
			rt_mutex_release(&mutex_dessinArene);
			cout << RED << "[#Arena] dessins : " RESET << dessinArene << endl << flush;
		}else if (cachedMessage->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){
			rt_mutex_acquire(&mutex_areneOk, TM_INFINITE);
			areneOk = true;
			rt_mutex_release(&mutex_areneOk);
		}else if (cachedMessage->CompareID(MESSAGE_CAM_ARENA_INFIRM)){
			rt_mutex_acquire(&mutex_areneOk, TM_INFINITE);
			areneOk = false;
			rt_mutex_release(&mutex_areneOk);
		}else if (cachedMessage->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)){
			rt_mutex_acquire(&mutex_calculPosition , TM_INFINITE);
			calculPosition = true;
			rt_mutex_release(&mutex_calculPosition);
		}else if(cachedMessage->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){
			rt_mutex_acquire(&mutex_calculPosition , TM_INFINITE);
			calculPosition = false;
			rt_mutex_release(&mutex_calculPosition);
		}else {
			cout << BLUE << "[#Camera] " << RESET << "Request received not handle " << cachedMessage->ToString() << endl << flush;
		}
		
	rt_mutex_release(&mutex_camera);
	}
}


void Tasks::CameraPeriodic(void * args){
	bool cpCameraActive = false;
	bool cpCalculPosition = false;
	bool cpDessinArene = false;

	enum State { askArena, awaitConfirm };
	State actualState = askArena;

	Arena arene;
	cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
	// Synchronization barrier (waiting that all tasks are starting)
	rt_sem_p(&sem_barrier, TM_INFINITE);
	
			
	rt_task_set_periodic(NULL, TM_NOW, TIME_PERIOD_CAMERA);
	
	while(1){
		rt_task_wait_period(NULL);
		rt_mutex_acquire(&mutex_camera, TM_INFINITE);	
		
		rt_mutex_acquire(&mutex_cameraActive, TM_INFINITE);
		cpCameraActive = cameraActive;
		rt_mutex_release(&mutex_cameraActive);

		rt_mutex_acquire(&mutex_calculPosition, TM_INFINITE);
		cpCalculPosition = calculPosition;
		rt_mutex_release(&mutex_calculPosition);
		
		rt_mutex_acquire(&mutex_dessinArene, TM_INFINITE);
		cpDessinArene = dessinArene;
		rt_mutex_release(&mutex_dessinArene);

		if (camera.IsOpen()){
			
			if (cpCameraActive && not(cpDessinArene) && not(cpCalculPosition) ){
				Message * tosend= new MessageImg(MESSAGE_CAM_IMAGE,camera.Grab().Copy());
				WriteInQueue(&q_messageToMon, tosend);  // toSend will be deleted by sendToMon
			}

			if (cpDessinArene){
				// Dessiner arène
				if (actualState == askArena){
					Img * img = camera.Grab().Copy();
					arene = img->SearchArena();
						
					if (not(arene.IsEmpty())){
						img->DrawArena(arene);
						Message * tosend= new MessageImg(MESSAGE_CAM_IMAGE,img);
						WriteInQueue(&q_messageToMon, tosend);  // toSend will be deleted by sendToMon
					}
					actualState = awaitConfirm;
				}
				if (actualState == awaitConfirm){
					rt_mutex_acquire(&mutex_areneOk , TM_INFINITE);
					if (areneOk){
						rt_mutex_acquire(&mutex_arena, TM_INFINITE);
						arena = arene ;
						rt_mutex_release(&mutex_arena);
						
						rt_mutex_acquire(&mutex_dessinArene, TM_INFINITE);
						dessinArene = false;
						rt_mutex_release(&mutex_dessinArene);
					}else{
						actualState = askArena;
					}
					rt_mutex_release(&mutex_areneOk);
				}
			}

			if (cpCalculPosition){
				Img * img = camera.Grab().Copy();
				rt_mutex_acquire(&mutex_arena, TM_INFINITE);
				arene = arena;
				rt_mutex_release(&mutex_arena);
				list<Position> listPos = img->SearchRobot(arene);
				img->DrawAllRobots(listPos);
				WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE,img));
				
				if (!listPos.empty()){
					for (Position pos : listPos ){
						WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION,pos));
					}
				}
			}
		}
	
		rt_mutex_release(&mutex_camera);
	}
}


/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
	int err;
	Message *msg;

	if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
		cout << "Read in queue failed: " << strerror(-err) << endl << flush;
		throw std::runtime_error{"Error in read in queue"};
	}/** else {
		 cout << "@msg :" << msg << endl << flush;
		 } /**/

	return msg;
}


