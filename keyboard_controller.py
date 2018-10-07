import rospy
import time
import locale
import curses
import sys
from pySpacebrew.spacebrew import Spacebrew

if __name__ == "__main__":
  name = "pyBoolean Example"
  server = "sandbox.spacebrew.cc"

  brew = Spacebrew(name=name, server=server)
  brew.addPublisher("cmd", "string")
  try:
    # start-up spacebrew
    brew.start()
    locale.setlocale(locale.LC_ALL, '')
    code = locale.getpreferredencoding()
    rate = rospy.Rate(1.0)
      # initialize the terminal display
    stdscr = curses.initscr()
    stdscr.keypad(1)
    curses.noecho()			# turn off echo
    curses.curs_set(0)		# turn off cursor
    while 1:
      c = stdscr.getch()

      if (c == 119): 
        brew.publish('cmd', 'w')
      elif (c == 115):
        brew.publish('cmd', 's')
      elif (c == 97):
        brew.publish('cmd', 'a')
      elif (c == 100):
        brew.publish('cmd', 'd')
      elif (c == 120):
        break 
      else:
        brew.publish('cmd', 'nan')
      rate.sleep()
    curses.nocbreak()
    stdscr.keypad(0)
    curses.echo()
    curses.endwin()  
  finally:
    brew.stop()
    
