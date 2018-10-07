import rospy
import time
import locale
import curses
import sys
from pySpacebrew.spacebrew import Spacebrew

def check_pwd(val):
  global pswd, check
  if val==pswd:
    check = True
  else:
    check = False

if __name__ == "__main__":
  pswd = None
  check = False
  global check, pswd
  name = "Keyboard Controller"
  server = "sandbox.spacebrew.cc"

  brew = Spacebrew(name=name, server=server)
  brew.addSubscriber("pwd_check", "string")
  brew.addPublisher("pwd", "boolean")
  brew.addPublisher("fwd", "boolean")
  brew.addPublisher("rev", "boolean")
  brew.addPublisher("left", "boolean")
  brew.addPublisher("right", "boolean")
  brew.subscribe("pwd_check", check_pwd)
  cont = 0
  try:
    # start-up spacebrew
    brew.start()
    pswd = str(raw_input("Enter the password: "))
    brew.publish('pwd', True)
    i=0
    while 1:
      if check==True:
        cont = 1
        break
      else:
        i+=1
        time.sleep(1)
        if i>=60:
          cont = 0
          break
      continue
    if cont == 1:
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
          brew.publish('fwd', True)
        elif (c == 115):
          brew.publish('rev', True)
        elif (c == 97):
          brew.publish('left', True)
        elif (c == 100):
          brew.publish('right', True)
        elif (c == 120):
          break 
        else:
          brew.publish('fwd', False)
          brew.publish('rev', False)
          brew.publish('left', False)
          brew.publish('right', False)
        rate.sleep()
      curses.nocbreak()
      stdscr.keypad(0)
      curses.echo()
      curses.endwin()  
    else:
      print "Incorrect password. Try again later"
  finally:
    brew.stop()
    
