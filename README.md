# call_cartographer_service

this project is used to call cartographer_ros's service '/read_metrics' 
and process its responds to judge weather the SLAM is work will.
If the SLAM dose not localize well, a topic should send messages to ugv
to stop the ugv and run global localization. 
