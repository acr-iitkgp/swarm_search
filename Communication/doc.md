## Communication setup and working

Communication can be broken into three layers:
  1. Physical setup (Infra/Ad-Hoc) modes   
  2. Routing(Only for Ad-Hoc): (None/OLSR/AODV)     
  3. Application: (ROS Multimaster-fkie/Socket Communication)   
 
 *Ad-Hoc Mode provides the benefit of mobility and independence from infrastructure*  
 *OLSR/Dynamic routing allows us to provide bridge network between distant nodes.*  
 *ROS Multimaster-fkie proves multimaster communication between each master on the respective devices.*  
  
On the basis of Physical setup our solutions can be divided into two parts:
  1. Ad-Hoc [AdHoc_Setup](AdHoc_Setup)  
  2. Router-based infrastructure  

### Ad-Hoc Possibilities  
- - - -

#### 1. No Dynmic Routing + ROS Multimaster-fkie *(Currently working solution)* 
[Multimaster Setup](Multimaster_Setup)
#### 2. OLSR Routing + ROS Multimaster-fkie (Compatibility of OLSR with fkie is under investigation)  
[OLSRD_Setup](OLSR_Setup) and [Multimaster Setup](Multimaster_Setup)
#### 3. AODV Routing + ROS Multimaster-fkie (AODV Implementation feasibility not investigated yet)  
#### 4. OLSR Routing + Socket Communication (Very tedious)  


### Infrastructure Possibilities
- - - -
#### 1. ROS Multimaster-fkie (To be Tested)   


### General Topics  
- - - - 
#### ROS multimaster-fkie
[Package Summary](http://wiki.ros.org/multimaster_fkie)   
[GitHub](https://github.com/fkie/multimaster_fkie)  
[Multimaster Setup](Multimaster_Setup) To be Added: Concise documentation.  
[Multi-Master Report](https://l.messenger.com/l.php?u=http%3A%2F%2Fwww.iri.upc.edu%2Ffiles%2Fscidoc%2F1607-Multi-master-ROS-systems.pdf&h=AT3TktPG6r55eKY0PLOgQ0Ucyuys47o4Tsw296HVCCHYiGhbS0120_djqPju6cHIvOTlBP3Z7fJ4VeiBxg5K0xN8WQLcOKXm0BkDfcHons3hNL6QkfPg2WX9bHAMPA)

#### OLSR Working
[OLSRD_Setup](OLSR_Setup)   
[OLSR_AdHoc Working](https://github.com/Aryan-jaiswal/UAV_Fleet_Challenge/wiki/OLSR_AdHoc-Working)

#### Tweaking OLSRD (olsrd.conf)
Content to be added.
