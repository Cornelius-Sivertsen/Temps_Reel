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

#ifndef __TASKS_H__
#define __TASKS_H__

#include <unistd.h>
#include <iostream>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "messages.h"
#include "commonitor.h"
#include "comrobot.h"
#include "camera.h"
#include "img.h"

using namespace std;

class Tasks {
public:
    /**
     * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
     */
    void Init();

    /**
     * @brief Starts tasks
     */
    void Run();

    /**
     * @brief Stops tasks
     */
    void Stop();
    
    /**
     * @brief Suspends main thread
     */
    void Join();
    
private:
    /**********************************************************************/
    /* Shared data                                                        */
    /**********************************************************************/
    ComMonitor monitor;
    ComRobot robot;
    int robotStarted = 0;
    int move = MESSAGE_ROBOT_STOP;
    
    // The action asked for by the monitor in the latest received cam message
    enum cameraActions_t{
        openCamera,
        closeCamera,
        stopImageStream,
        startImageStream
    } cameraAction = closeCamera; 
    
    
    bool imageStreamActive = false; //True if supervisor is supposed to be
                                     //currently sending images to monitor
    
    Camera Cam = Camera(sm, 10); //Shared object used to access the camera.
    
    Arena foundArena;
    bool arenaConfirmed = false;
    
    bool calculateRobotPosition = false; //True if monitor has activated 
					 //position calculation.
    
    

    /**********************************************************************/
    /* Tasks                                                              */
    /**********************************************************************/
    RT_TASK th_server;
    RT_TASK th_sendToMon;
    RT_TASK th_receiveFromMon;
    RT_TASK th_openComRobot;
    RT_TASK th_startRobot;
    RT_TASK th_move;
    RT_TASK th_periodicGetBatteryStatus; // Periodically checks battery status of robot
    RT_TASK th_cameraSend; // Thread handling camera
    RT_TASK th_cameraChangeActivity; //Turn camera on and off
    RT_TASK th_findArena;
    
    /**********************************************************************/
    /* Mutex                                                              */
    /**********************************************************************/
    RT_MUTEX mutex_monitor;
    RT_MUTEX mutex_robot;
    RT_MUTEX mutex_robotStarted;
    RT_MUTEX mutex_move;
    RT_MUTEX mutex_readMsg;
    RT_MUTEX mutex_camera;
    RT_MUTEX mutex_cameraActions; //Protects cameraActions enum
    RT_MUTEX mutex_imageStreamActive; //Protects imageStreamActive bool
    RT_MUTEX mutex_arena; //Protects all shared data regarding to the arena
    RT_MUTEX mutex_RobotPos; //Protects calculateRobotPosition bool

    /**********************************************************************/
    /* Semaphores                                                         */
    /**********************************************************************/
    RT_SEM sem_barrier;
    RT_SEM sem_openComRobot;
    RT_SEM sem_serverOk;
    RT_SEM sem_startRobot;
    RT_SEM sem_cameraActivity; //Used to signal changing of camera activity
    RT_SEM sem_askArena; //Trigger finding of arena
    RT_SEM sem_arenaConfirm;

    /**********************************************************************/
    /* Message queues                                                     */
    /**********************************************************************/
    int MSG_QUEUE_SIZE;
    RT_QUEUE q_messageToMon;
    
    /**********************************************************************/
    /* Tasks' functions                                                   */
    /**********************************************************************/
    /**
     * @brief Thread handling server communication with the monitor.
     */
    void ServerTask(void *arg);
     
    /**
     * @brief Thread sending data to monitor.
     */
    void SendToMonTask(void *arg);
        
    /**
     * @brief Thread receiving data from monitor.
     */
    void ReceiveFromMonTask(void *arg);
    
    /**
     * @brief Thread opening communication with the robot.
     */
    void OpenComRobot(void *arg);

    /**
     * @brief Thread starting the communication with the robot.
     */
    void StartRobotTask(void *arg);
    
    /**
     * @brief Thread handling control of the robot.
     */
    void MoveTask(void *arg);
    
    /**
    * @brief Thread that periodically sends robot's battery status to monitor
    */
    void periodic_GetBatteryStatusTask(void);

    /**
     * @brief Thread handling periodic sending of images from camera.
     * if an arena has been found, also adds arena to image.
     * if position calculation is enabled, also sends position and adds it to 
     * the image
     */
    void periodic_cameraSendTask(void);

    /**
     * @brief Thread handling opening and closing of camera, as well as turning
     * the image stream on/off.
     */
    void cameraChangeActivityTask(void);

    
    /**********************************************************************/
    /* Queue services                                                     */
    /**********************************************************************/
    /**
     * Write a message in a given queue
     * @param queue Queue identifier
     * @param msg Message to be stored
     */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);
    
    /**
     * Read a message from a given queue, block if empty
     * @param queue Queue identifier
     * @return Message read
     */
    Message *ReadInQueue(RT_QUEUE *queue);


    /**
     * @brief Task that, when demanded by the monitor, finds the arena. If an
     * arena is accepted, adds this arena to a shared variable s.t. the camera 
     * thread can add the arena to the images it sends.
     */
    void findArenaTask(void);
    
};


#endif // __TASKS_H__ 

