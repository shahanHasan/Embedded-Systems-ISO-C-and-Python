//======================================================================
//
// Test program to test the infrared sensors (and motors) of the
// 4tronix initio robot car. One can run this program within an
// ssh session.
//
// author: Raimund Kirner, University of Hertfordshire
//         initial version: Dec.2016
//
// license: GNU LESSER GENERAL PUBLIC LICENSE
//          Version 2.1, February 1999
//          (for details see LICENSE file)
//
// Compilation: 
// gcc -o camcar -Wall -Werror -lcurses -lwiringPi -lpthread -linitio camcar.c
//
//======================================================================

#include <assert.h>
#include <stdlib.h>
#include <pthread.h>
#include <initio.h>
#include <curses.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include "detect_blob.h"

//======================================================================
// Coursework ESD, general instructions
// This file (camcar.c) is the major file to be modified in order to 
// complete the coursework.  There are a few locations marked with "TODO",
// which indicate places where the code might need changes or completion.
// This directory also contains two other source files:
// quickblob.c ... this is a library for searching blobs in images
// detect_blob.c ... this is a wrapper for quickblob.c, providing a 
//                   direct interface to the RaspberryPI camera.
// Normally, quickblob.c and detect_blob.c don't need changes. However,
// studying detect_blob.c a bit is still advisable.
//
// The implementation of the nested state machine in camcar() follows
// the implementation proposal given in the Lecture slides. You may
// want to change the FSM implementation to add extra or refined
// behaviour.
//======================================================================

struct thread_dat {
  TBlobSearch blob;
  int blobnr;
  int bExit; // flag to indicate termination of thread
};

pthread_mutex_t count_mutex; // mutex to protect thread communication


//======================================================================
// camcar():
// Skeleton code for the ESD coursework.
// The implementation uses hierarchical finite state machines (FSM), in order
// to reduce the size of the state transition graph.
// To be done: Fill the actions of the individual states of the FSMs
// with meaningful behaviour.
//======================================================================
void camcar(int argc, char *argv[], struct thread_dat *pdat) 
{
    int ch = 0;

    // main control loop:  
    while (ch != 'q') {
        int obstacle_L, obstacle_R, obstacle; // FSM-OA
        TBlobSearch blob;
        unsigned int blob_count = 0;  // simple counter of performed blob searches
        int time, time_blob;	
        int blobSufficient; // FSM-SB
        int carBlobAligned; // FSM-AB
        int distance;
        distance = initio_UsGetDistance ();
        int lastblobn08r = 0;
        enum { distok, tooclose, toofar} distanceState; // FSM-MB

        mvprintw(1, 1,"%s: Press 'q' to end program", argv[0]);
        blob.size = 0;

        obstacle_L = ( initio_IrLeft() !=0 );
        obstacle_R = ( initio_IrRight()!=0 );
        obstacle = obstacle_L || obstacle_R;

        // FSM-OA (Obstacle Avoidance)
        if (obstacle || distance < 5) {
            mvprintw(3, 1,"State OA (stop to avoid obstacle), o-left=%d, o-right=%d", obstacle_L, obstacle_R);
            clrtoeol(); // curses library
            initio_DriveForward (0); // Stop
        }
        else {
            // blob search is time consuming, keep some intervals between updates
            time = millis();
            if (blob_count==0 || time_blob < (time - 1000)) {
                refresh(); // curses lib: update display
                //blob = cameraSearchBlob( blobColor ); // search for sign with RED colored blob
                pthread_mutex_lock(&count_mutex);
                blob = pdat->blob;
                pthread_mutex_unlock(&count_mutex);
                // writeImageWithBlobAsJPEG() seems to have a bug, do not use right now:
                //writeImageWithBlobAsJPEG(blob, "test_blob.jpg", 70);  // TODO: this function is for testing (deactivate if not needed)
                time_blob = time;
                blob_count++;
            }
            blobSufficient = (blob.size > 60);  // TODO: experiment with that threshold value

            // FSM-SB (Search Blob)
            if ( ! blobSufficient ) {
                mvprintw(3, 1,"State SB (search blob), blob.size=%d (count: %u)", blob.size, blob_count);
                clrtoeol(); // curses library
                // TODO: potential actions: turn car or camera platform a few steps around and see if a blob is to be found
                if (lastblobnr < pdat->blobnr) {
                   initio_SpinRight(70);
				   delay(400);
                   lastblobnr = pdat->blobnr;
                }
            }
            else {
                carBlobAligned = (blob.halign >= -0.3 && blob.halign <= 0.3);  // TODO: adjust values to useful ones

                // FSM-AB (Align to Blob)
                if ( ! carBlobAligned) {
                    clrtoeol(); // curses library
                    if (lastblobnr < pdat->blobnr) {
                       if (blob.halign <= 0)//do
						{
							mvprintw(3, 1,"State AB (align right towards blob), blob.size=%d, halign=%f", blob.size, blob.halign);
							initio_SpinRight(70);
							delay(400);
						}
						else//otherwise do
						{
							mvprintw(3, 1,"State AB (align left towards blob), blob.size=%d, halign=%f", blob.size, blob.halign);
							initio_SpinLeft(70);
							delay(400);
						}
						
						delay(400);
						initio_DriveForward (0); 
						lastblobnr = pdat->blobnr;
                    }
                    
                   // TODO: slightly turn car to align with blob (find a useful turn duration)
                    //       if blob.halign is negative, then turn right, otherwise left
                }
                else {
                    
                    if (distance < 40)
                    { distanceState = tooclose; }
                    else if (distance > 50)
                    { distanceState = toofar; }
                    else 
                    { distanceState = distok; }
                    
                    // FSM-MB (cat at middle of blob, keep distance)
                    switch (distanceState) {
                    case toofar:
                        mvprintw(3, 1,"State FB (drive forward), dist=%d", distance);
                        clrtoeol(); // curses library
                        initio_DriveForward(30); // TODO: move car forward to come closer
                        break;
                    case tooclose:
                        mvprintw(3, 1,"State RB (drive backwards), dist=%d", distance);
                        clrtoeol(); // curses library
                        initio_DriveReverse(30);// TODO: move car backwards to get more distance
                        break;
                    case distok:
                        mvprintw(3, 1,"State KD (keep distance), dist=%d", distance);
                        clrtoeol(); // curses library
                        initio_DriveForward (0); // Stop
                    } // switch (FSM-MB)
                } // if (FSM-AB)
            } // if (FSM-SB)
        } // if (FSM-OA)

        ch = getch();
        if (ch != ERR) mvprintw(2, 1,"Key code: '%c' (%d)  ", ch, ch);
        refresh(); // curses lib: update display
        //delay (100); // pause 100ms
  } // while

  return;
}


// Thread function to measure distance with ultrasonic sensor
void *worker(void *p_thread_dat)
{
  struct thread_dat *ptdat = (struct thread_dat *) p_thread_dat;
  // TODO: fill in code: loop with termination check and distance measurement
  //                     the measured distance is to be copied into ptdat->distance
  //                     the exit condition comes from ptdat->bExit
  const char blobColor[3] = { 255, 0, 0 };  // TODO: maybe adjust depending on room's light condition
  TBlobSearch blob;
  while (ptdat->bExit == 0) {
    blob = cameraSearchBlob( blobColor ); // search for sign with RED colored blob
    pthread_mutex_lock(&count_mutex);
    ptdat->blob = blob;
    pthread_mutex_unlock(&count_mutex);
    ptdat->blobnr++;
  }
  return NULL;
}


//======================================================================
// main(): initialisation of libraries, etc
//======================================================================
int main (int argc, char *argv[])
{
  WINDOW *mainwin = initscr();  // curses: init screen
  noecho ();                    // curses: prevent the key being echoed
  cbreak();                     // curses: disable line buffering 
  nodelay(mainwin, TRUE);       // curses: set getch() as non-blocking 
  keypad (mainwin, TRUE);       // curses: enable detection of cursor and other keys

  initio_Init (); // initio: init the library

  pthread_t us_thread;          // pthread: thread handle
  pthread_attr_t pt_attr;       // pthread: thread attributes
  struct thread_dat tdat;       // data structure to communicate with thread
  tdat.blobnr = 0;
  tdat.bExit = 0;
  pthread_attr_init(&pt_attr);  // pthread: create and init thread attribute
  // TODO: create thread via pthread_create()
  // thread handle ... &us_thread
  // thread attributes ... &pt_attr
  // thread routine ... worker
  // parameter shared with thread routine: &tdat
  assert	(pthread_create(&us_thread,	&pt_attr,	worker,	&tdat)==0	);

  camcar(argc, argv, &tdat);

  tdat.bExit = 1;               // signal thread to terminate
  // TODO: wait for thread to finish via pthread_join():
  //       thread handle ... us_thread
  //       pointer return value ... NULL
  assert	(	pthread_join(us_thread,	NULL)	==	0	);
  // TODO: distroy thread attribute pt_attr via pthread_attr_destroy()
  //       thread attributes: &pt_attr
  pthread_attr_destroy(&pt_attr);
  
  initio_Cleanup ();  // initio: cleanup the library (reset robot car)
  endwin();           // curses: cleanup the library
  return EXIT_SUCCESS;
}

