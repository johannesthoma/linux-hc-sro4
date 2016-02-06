dist1=`cat /sys/class/distance-sensor/distance_23_24/measure`
while true
do
	dist=`cat /sys/class/distance-sensor/distance_23_24/measure`
	rel=$[ ($dist-$dist1)*10000 / $dist1 ]
	echo $[ $rel/100 ].$[ $rel%100 ]
	sleep 1
done
