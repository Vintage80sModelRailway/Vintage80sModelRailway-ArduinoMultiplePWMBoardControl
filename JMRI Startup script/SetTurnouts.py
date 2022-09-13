# Sample script showing how to multiple turnouts to 
# specific positions, with a time delay between them,
# separately from other things that the program is doing.
#
# By putting the turnout commands in a separate class, they'll
# run independently after the "start" operation
#
# Part of the JMRI distribution

import jmri
from org.apache.log4j import Logger

class setStartup(jmri.jmrit.automat.AbstractAutomaton) :   
	log = Logger.getLogger("jmri.jmrit.automat.AbstractAutomaton.setStartup")   
	def init(self):
		return
	def handle(self):
  
		state = sensors.provideSensor("CS1001").knownState	
		self.log.info("CS1001 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT1001").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT1001 Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT1001").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C11001 Closed")
		
		state = sensors.provideSensor("CS1002").knownState	
		self.log.info("CS1002 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT1002").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT1002 Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT1002").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C11002 Closed")
		
		state = sensors.provideSensor("CS1013").knownState	
		self.log.info("CS1003 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT1003").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT1003 Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT1003").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C11003 Closed")
		
		state = sensors.provideSensor("CS1015").knownState	
		self.log.info("CS1015 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT1004").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT1004 Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT1004").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C11004 Closed")
		
		code = "1005"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		state = sensors.provideSensor("CS3012").knownState	
		self.log.info("CS1006 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT1006").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT1006 Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT1006").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C11006 Closed")

		code = "1009"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "1010"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "1011"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "1012"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "1013"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "1014"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "1015"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "1016"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "2001"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "2002"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "2003"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "2004"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "2005"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "2006"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "2007"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		code = "2008"
		state = sensors.provideSensor("CS"+code).knownState	
		self.log.info("CS"+code+" state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT"+code).setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT"+code+" Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT"+code).setState(2)
			self.waitMsec(50) 
			self.log.info("Set state C1"+code+" Closed")

		state = sensors.provideSensor("CS3001").knownState	
		self.log.info("CS3001 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT3001").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT3001 Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT3001").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state CT3001 Closed")
			
		state = sensors.provideSensor("CS3002").knownState
		self.log.info("CS3002 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT3002").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT3002 Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT3002").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state CT3002 Closed") 
			
		state = sensors.provideSensor("CS3003").knownState
		self.log.info("CS3003 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT3003").setState(4)
			self.log.info("Set state CT3003 Thrown")
			self.waitMsec(50) 
		elif state == 4:
			turnouts.provideTurnout("CT3003").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state CT3003 Closed") 
			
		state = sensors.provideSensor("CS3004").knownState
		self.log.info("CS3004 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT3004").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT3004 Thrown") 
		elif state == 4:
			turnouts.provideTurnout("CT3004").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state CT3004 Closed")
			
		state = sensors.provideSensor("CS3005").knownState
		self.log.info("CS3005 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT3005").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT3005 Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT3005").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state CT3005 Closed")
			
		state = sensors.provideSensor("CS3006").knownState
		self.log.info("CS3006 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT3006").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT3006 Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT3006").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state CT3006 Closed")
			
		state = sensors.provideSensor("CS3007").knownState
		self.log.info("CS3007 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT3007").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT3007 Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT3007").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state CT3007 Closed")
			
		state = sensors.provideSensor("CS3008").knownState
		self.log.info("CS3008 state: " + str(state))
		if state == 2:
			turnouts.provideTurnout("CT3008").setState(4)
			self.waitMsec(50) 
			self.log.info("Set state CT3008 Thrown")
		elif state == 4:
			turnouts.provideTurnout("CT3008").setState(2)
			self.waitMsec(50) 
			self.log.info("Set state CT3008 Closed")
		return False              # all done, don't repeat again

setStartup().start()          # create one of these, and start it running
