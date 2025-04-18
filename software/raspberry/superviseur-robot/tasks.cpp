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
#define PRIORITY_TCAMERA_CHANGEACTIVITY 21
#define PRIORITY_TCAMERA_SEND 28
#define PRIORITY_TBATTERY 40
#define PRIORITY_TARENA 35

#define CLOCKTICKS_TO_MS 1000000

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
void Tasks::Init() {
    int status;
    int err;

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
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cameraActions, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_imageStreamActive, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_RobotPos, NULL)) {
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
    if (err = rt_sem_create(&sem_cameraActivity, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_askArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arenaConfirm, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }



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
    if (err = rt_task_create(&th_periodicGetBatteryStatus, "th_getBatteryStatus", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_cameraSend, "th_camera", 0, PRIORITY_TCAMERA_SEND, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_cameraChangeActivity, "th_cameraChangeActivity", 0, PRIORITY_TCAMERA_CHANGEACTIVITY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_findArena, "th_findArena", 0, PRIORITY_TARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
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
    if (err = rt_task_start(&th_periodicGetBatteryStatus, (void(*)(void*)) & Tasks::periodic_GetBatteryStatusTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_cameraSend, (void(*)(void*)) & Tasks::periodic_cameraSendTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_cameraChangeActivity, (void(*)(void*)) & Tasks::cameraChangeActivityTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_findArena, (void(*)(void*)) & Tasks::findArenaTask, this)) {
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
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
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
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
            //Opening and closing of camera:
        } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN) ||
                (msgRcv->CompareID(MESSAGE_CAM_CLOSE))) {

            rt_mutex_acquire(&mutex_cameraActions, TM_INFINITE);
            //Set correct cameraActivity
            if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
                cameraAction = openCamera;
            } else if ((msgRcv->CompareID(MESSAGE_CAM_CLOSE))) {
                cameraAction = closeCamera;
            }
            rt_mutex_release(&mutex_cameraActions);


            //Release camera change activity semaphore, triggering the thread
            //that handles changing camera activity.
            rt_sem_v(&sem_cameraActivity);
            
            //Starting finding of arena:
        } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            rt_sem_v(&sem_askArena); //trigger arena finding task
            
            //Handle confirmation/rejection of found arena
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM) or
                msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {

            rt_mutex_acquire(&mutex_arena, TM_INFINITE);
            arenaConfirmed = (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM));
            rt_mutex_release(&mutex_arena);

            rt_sem_v(&sem_arenaConfirm);
            
            //Detect starting and stopping of position finding
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
            rt_mutex_acquire(&mutex_RobotPos, TM_INFINITE);
            calculateRobotPosition = true;
            rt_mutex_release(&mutex_RobotPos);
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
            rt_mutex_acquire(&mutex_RobotPos, TM_INFINITE);
            calculateRobotPosition = false;
            rt_mutex_release(&mutex_RobotPos);
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
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

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
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon

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

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);

            cout << " move: " << cpMove;

            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID) cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
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

/**
 * @brief Thread that periodically sends robot's battery status to monitor
 */

void Tasks::periodic_GetBatteryStatusTask(void) {

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    MessageBattery * batteryMsg;
    bool hasRobotStarted; // Used for checking if robot has started

    // Task starts here
    rt_task_set_periodic(NULL, TM_NOW, 500 * CLOCKTICKS_TO_MS); //500ms    


    while (1) {
        rt_task_wait_period(NULL);


        //Check if robot is started:
        //Storing in temp variable to avoid holding mutex too long.
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        hasRobotStarted = robotStarted;
        rt_mutex_release(&mutex_robotStarted);


        if (hasRobotStarted) {

            cout << "Sending battery status" << endl << flush;

            //Get battery status from robot
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            batteryMsg = (MessageBattery*) robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
            rt_mutex_release(&mutex_robot);

            //Send battery status to monitor
            WriteInQueue(&q_messageToMon, batteryMsg);
        }
    }
}

/**
 * @brief Thread handling opening and closing of camera, as well as turning
 * the image stream on/off.
 */
void Tasks::cameraChangeActivityTask(void) {

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    // For ack/nack message
    Message *msgResponse;

    //Temporary variable for storing the wanted camera action
    //Using a temp variable to avoid holding a mutex too long.
    enum cameraActions_t cameraAction_temp = closeCamera;

    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);



    while (1) {

        // Wait for activation of camera from monitor
        rt_sem_p(&sem_cameraActivity, TM_INFINITE);

        rt_mutex_acquire(&mutex_cameraActions, TM_INFINITE);
        cameraAction_temp = cameraAction;
        rt_mutex_release(&mutex_cameraActions);

        switch (cameraAction_temp) {
            case openCamera:
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);

                //Open camera and check if success or not
                if (!Cam.Open()) {
                    rt_mutex_release(&mutex_camera);
                    //Send error msg
                    msgResponse = new Message(MESSAGE_ANSWER_NACK);
                    cout << "Camera failed to open" << endl << flush;

                    WriteInQueue(&q_messageToMon, msgResponse);
                } else rt_mutex_release(&mutex_camera);

                //Start image stream:
                rt_mutex_acquire(&mutex_imageStreamActive, TM_INFINITE);
                imageStreamActive = true;
                rt_mutex_release(&mutex_imageStreamActive);

                break;
            case closeCamera:
                //Stop sending images
                rt_mutex_acquire(&mutex_imageStreamActive, TM_INFINITE);
                imageStreamActive = false;
                rt_mutex_release(&mutex_imageStreamActive);


                rt_mutex_acquire(&mutex_camera, TM_INFINITE);

                //Avoid attempting to close camera if it is already closed
                if (Cam.IsOpen()) {
                    Cam.Close();
                }

                rt_mutex_release(&mutex_camera);
                
                msgResponse = new Message(MESSAGE_ANSWER_ACK);
                WriteInQueue(&q_messageToMon, msgResponse);

                break;
            case stopImageStream:
                //Stop sending images
                rt_mutex_acquire(&mutex_imageStreamActive, TM_INFINITE);
                imageStreamActive = false;
                rt_mutex_release(&mutex_imageStreamActive);

                break;
            case startImageStream:

                //Start image stream:
                rt_mutex_acquire(&mutex_imageStreamActive, TM_INFINITE);
                imageStreamActive = true;
                rt_mutex_release(&mutex_imageStreamActive);
                break;
            default:
                //Do nothing.
                ;
        }
    }
}

/**
 * @brief Thread handling periodic sending of images from camera.
 * if an arena has been found, also adds arena to image.
 * if position calculation is enabled, also sends position and adds it to 
 * the image
 */
void Tasks::periodic_cameraSendTask(void) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    //Temporary variable to avoid holding a mutex too long
    bool image_sending_active_temp = false;
    
    //Used for storing the calculated robot position
    std::list<Position> robotPositionList;
    
    //Used for sending the calculated robot position
    MessagePosition *msgPos;
    
    bool everyOther = false; //Used to run position finding only every other
    //loop, if not the task takes too much CPU time.

    // Task starts here
    rt_task_set_periodic(NULL, TM_NOW, 200 * CLOCKTICKS_TO_MS); //200 ms
    //Slower than required, if not the task takes too much processor time

    while (1) {
        rt_task_wait_period(NULL);

        rt_mutex_acquire(&mutex_imageStreamActive, TM_INFINITE);
        image_sending_active_temp = imageStreamActive;
        rt_mutex_release(&mutex_imageStreamActive);


        if (image_sending_active_temp) {
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);

            if (Cam.IsOpen()) {
                Img * img = new Img(Cam.Grab());
                rt_mutex_release(&mutex_camera);

                //Add arena to image, if an arena has been found:
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                if (arenaConfirmed) {
                    img->DrawArena(foundArena);

                    //check if we should be calculating robot position
                    //Note that this code runs only every other time the thread
                    //runs, this is to avoid saturating the CPU.
                    rt_mutex_acquire(&mutex_RobotPos, TM_INFINITE);
                    if (everyOther and calculateRobotPosition) {
                        robotPositionList = img->SearchRobot(foundArena);

                        //Check if position list is empty (i.e. robot not found)
                        if (robotPositionList.empty()) {//if yes: construct (-1,-1) message                            
                            Position EmptyPos;
                            EmptyPos.center = cv::Point2f(-1.0, -1.0);
                            msgPos = new MessagePosition(MESSAGE_CAM_POSITION, EmptyPos);
                        } else {//if no: construct real position message, then draw pos on image

                            msgPos = new MessagePosition(MESSAGE_CAM_POSITION, robotPositionList.front());
                            img->DrawRobot(robotPositionList.front());
                        }
                        
                        
                        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                        monitor.Write(msgPos); // The message is deleted with the Write
                        rt_mutex_release(&mutex_monitor);
                    }
                    rt_mutex_release(&mutex_RobotPos);
                    everyOther = !everyOther;
                }
                rt_mutex_release(&mutex_arena);

                //Construct message with the found image
                MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
                cout << "Sending image" << endl << flush;

                //Send message
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msgImg); // The message is deleted with the Write
                rt_mutex_release(&mutex_monitor);
                delete img;
            } else {
                rt_mutex_release(&mutex_camera);
            }
        }
    }
}

/**
 * @brief Task that, when demanded by the monitor, finds the arena. If an
 * arena is accepted, adds this arena to a shared variable s.t. the camera 
 * thread can add the arena to the images it sends.
 */
void Tasks::findArenaTask(void) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    Arena potentialArena;
    
    bool cameraWasClosed = false;

    // For ack/nack message
    Message *msgResponse;


    while (1) {
        rt_sem_p(&sem_askArena, TM_INFINITE);
        cout << "finding arena" << endl << flush;

        //Check if camera is open
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        if (Cam.IsOpen()) { //Yes-> stop image stream
            rt_mutex_release(&mutex_camera);

            rt_mutex_acquire(&mutex_cameraActions, TM_INFINITE);
            cameraAction = stopImageStream;
            rt_mutex_release(&mutex_cameraActions);

            rt_sem_v(&sem_cameraActivity);
        } else { //No -> open camera locally
            Cam.Open();
            rt_mutex_release(&mutex_camera);
            cameraWasClosed = true;
        }

        //Grab image from camera
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        Img * img = new Img(Cam.Grab());

        //(Close camera if camera was not already open)
        if (cameraWasClosed) {
            Cam.Close();
        }
        rt_mutex_release(&mutex_camera);

        //Extract arena
        potentialArena = img->SearchArena();

        //Check if arena was not found
        if (potentialArena.IsEmpty()) {
            cout << "Arena not found" << endl << flush;

            msgResponse = new Message(MESSAGE_ANSWER_NACK);
            WriteInQueue(&q_messageToMon, msgResponse);
        }


        //Send arena to monitor
        img->DrawArena(potentialArena);

        MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
        cout << "Sending image" << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msgImg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
        delete img;

        //Wait for response
        rt_sem_p(&sem_arenaConfirm, TM_INFINITE);

        //Check response
        rt_mutex_acquire(&mutex_arena, TM_INFINITE);
        if (arenaConfirmed) {
            foundArena = potentialArena;
        }
        rt_mutex_release(&mutex_arena);

        //Restart camera 
        rt_mutex_acquire(&mutex_cameraActions, TM_INFINITE);
        cameraAction = startImageStream;
        rt_mutex_release(&mutex_cameraActions);

        rt_sem_v(&sem_cameraActivity);
    }

}
