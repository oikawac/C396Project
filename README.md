# C396Project
ad hoc WAVE network implementing gossip protocol to share information between vehicles

Usage: 

./waf --run "project --traceFile=scratch/mobility.tcl --nodeFile=scratch/mobility.nodedata --tileFile=scratch/mobility.tiledata --nodeCount=154 --duration=1150.0 
--ifLocFile=scratch/mobility.if_locations_500 --ifCount=11 --param_r=1000 --param_p=5"

param_r - integer radius of request in meters
param_p - integer period between requests in seconds

ifLocFile - 500m between access points
