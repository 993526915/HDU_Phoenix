#! /bin/bash

while true 
do
	monitor=`ps -ef | grep RM_CV  | grep -v grep | wc -l ` 
	if [ $monitor -eq 0 ] 
	then
		echo "Program start"
		sudo ./RM_CV
	else
		echo "Program resatrt"
		fi
	sleep 5
done

