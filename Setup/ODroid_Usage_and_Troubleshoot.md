# ODroid Basic Instructions and Troubleshooting Guide

1. Always set date after booting to ODroid (sudo date DDMMhhmmYYYY.ss). 
This will prevent Internet certificate expiry errors.
2. Whenever you connect to a new Network, "always allow it for access to all users":

	GUI> Go To Edit Connections > Edit the Current Wi-Fi > Allow all users to connect to this network
This is necessary to allow network access to ODroid without needing any user to log in through the GUI.
3. Whenever you change the wireless module on an ODroid, Wi-Fi credentials have to be added separately for the new module. 
Again do not forget to "allow all users to connect to the network".
4. VIMP: When using a cloned image, remember to delete **.ssh** direcctory before starting to use SSH to avoid identity conflicts. 

FOR FUTURE:
1. A boot-time script to start roscore and other applications at the boot time itself. 
