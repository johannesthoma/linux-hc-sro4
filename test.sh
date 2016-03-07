while true
do
	dist=`cat /sys/class/distance-sensor/distance_23_24/measure`
 	if [ "$dist" ] ; then
	        cm=$[ $dist * 17150 ]
		printf "%d.%06d cm\n" $[ $cm/1000000 ] $[ $cm%1000000 ]
	fi
#	sleep 1
done
