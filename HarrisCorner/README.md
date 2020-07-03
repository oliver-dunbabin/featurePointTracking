# Harris corner algorithm with Jevois camera

Compile the Module for host using 
	./rebuild-host.sh

Compile the module for plaform using
	./rebuild-platform.sh

To add module to microsd card
	./rebuild-platform.sh --microsd

To add module to connected Jevois
	./rebuild-platform.sh --live

Make sure the Module has been added to videomappings.cfg file in order to select it using the daemon/ on the Jevois (see http://jevois.org/doc/ModuleTutorial.html#overview)
