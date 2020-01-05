Documentation:

Name of topic : /listener 

Messagetype is std_msgs/Float64MultiArray.msg

See: http://docs.ros.org/melodic/api/std_msgs/html/index-msg.html for details.
Pass coordinates as x,y in the Message.

To run, install the following dependencies - 

ros-melodic-ros-base (basic ROS installation)

ros-melodic-rosbridge-server (rosbridge_server)

to install these dependcies run -

     sudo apt install <package-name>
  
  NOTE: replace "melodic" with your ros version.
  
  After all dependencies are installed, to test setup,
  
  1.We will launch ROS
    
    roscore
    
  2.We can now launch the rosbridge v2.0 server with the following:
    
     
     roslaunch rosbridge_server rosbridge_websocket.launch
 3.Now we can start publishing a message from the server to test our JavaScript subscriber:
      
      rostopic pub /listener std_msgs/Float64MultiArray {} [87.30903, 22.315826] 
      
   (this is the current centre of the map.)
  
  NOTE: multi array msgs have two parameters: layout and data {} parses an empty layout to msg.
  We can edit it as required.
  
  Note: edit Source files and rebuild to make changes. (NPM to be used)
  See OpenLayers docs:
    https://openlayers.org/en/latest/doc/tutorials/bundle.html
.
