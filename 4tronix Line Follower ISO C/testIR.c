//======================================================================
//
// Test program to test the infrared sensors (and motors) of the
// 4tronix initio robot car. One can run this program within an
// ssh session.
//
// author: Raimund Kirner, University of Hertfordshire
//         initial version: Oct.2016
//
// license: GNU LESSER GENERAL PUBLIC LICENSE
//          Version 2.1, February 1999
//          (for details see LICENSE file)
//
// Compilation: 
// gcc -o testIR -Wall -Werror -lcurses -lwiringPi -lpthread -linitio testIR.c
//
//======================================================================

#include <stdlib.h>
#include <initio.h>
#include <curses.h>



//======================================================================
// testIR():
// Simple program to test infrared obstacle sensors:
// Drive forward and stop whenever an obstacle is detected by either
// the left or right infrared (IR) sensor.
//======================================================================
void testIR(int argc, char *argv[]) 
{
  int ch = 0;
  int irL, irR ;  
  int lfL, lfR ;  // white .. 1, black ... 0
 

  while (ch != 'q') {
    mvprintw(1, 1,"%s: Press 'q' to end program", argv[0]);

    irL = initio_IrLeft();
    irR = initio_IrRight();
    lfL = initio_IrLineLeft(); // white .. 1, black ... 0
    lfR = initio_IrLineRight(); //

    if (irL != 0 || irR != 0) {
      mvprintw(3, 1,"Action: Stop (IR sensors: %d, %d)     ", irL, irR);
      initio_DriveForward (0); // Stop
    }
    // no obstacle ahead, so focus on line following
    else if ((lfL == 0) && (lfR == 0 )) { 
      mvprintw(3, 1,"Action: Straight (Line sensors: %d, %d)    ", lfL, lfR);
      // todo: move straight forward
      //initio_DriveForward (100);
      //delay(1000);
      initio_DriveForward (80);
    }
    else if ((lfL == 1) && (lfR ==0 )) {
      // car is too much on the left
      mvprintw(3, 1,"Action: Spin left (Line sensors: %d, %d)    ", lfL, lfR);
      // todo: turn left
      initio_SpinRight(90);
      //delay(10);
      

    }
    else if ((lfR == 1) && (lfL == 0 )) {
	/* todo: change */
      // car is too much on the right
      mvprintw(3, 1,"Action: Spin right (Line sensors: %d, %d)    ", lfL, lfR);
      // todo: turn right
      initio_SpinLeft(90);
      //delay(10);
      //initio_DriveForward (0); // Stop
      
    }
    else
    {
      mvprintw(3, 1,"Lost my line (Line sensors: %d, %d)        ", lfL, lfR);
      // todo: Stop
      //initio_DriveForward (0); // Stop
	initio_DriveReverse (100);
      //delay(10);
      
    }

    ch = getch();
    if (ch != ERR)
      mvprintw(2, 1,"Key code: '%c' (%d)", ch, ch);
    refresh();
    //delay(20);
  } // while

  return;
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

  testIR(argc, argv);

  initio_Cleanup ();  // initio: cleanup the library (reset robot car)
  endwin();           // curses: cleanup the library
  return EXIT_SUCCESS;
}

