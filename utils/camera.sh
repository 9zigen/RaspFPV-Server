#!/bin/sh
PIDFile="raspivid.pid"
StreamingStatusFile="raspivid.streaming"
RecordStatusFile="raspivid.recording"

video_width=1280
video_height=720
photo_width=2592
photo_height=1944

[ -e $PIDFile ] && read PID <$PIDFile || PID=""
PID="$(pidof  raspivid)"
[ -e $StreamingStatusFile ] && read SS <$StreamingStatusFile || SS=""
[ -e $RecordStatusFile ] && read RS <$RecordStatusFile || RS=""

pauseStreaming() {
	if [ -e "$StreamingStatusFile" ]; then
		if [ ! -e "$RecordStatusFile" ]; then
			echo "Pause STREAMING"
			kill $PID
			rm $PIDFile
			PID=""
			sleep 1
		fi
	fi
}
resumeStreaming() {
	if [ -e "$StreamingStatusFile" ]; then
		echo "Resume STREAMING"
		$0 stream start $SS
	fi
}
killRaspivid() {
	kill $PID
	rm $PIDFile
	PID=""
}

case "$1" in
	stream)

		case "$2" in
			start)
				w=$5
				h=$6
				if [ -z $w ]; then
						w=640
				fi
				if [ -z $h ]; then
						h=480
				fi

				if [ ! -e "$StreamingStatusFile" ]; then
					echo "Start STREAMING"

					/usr/bin/raspivid -vf -w $w -h $h -fps 49 -b 2500000 -t 0 -n -pf base -o - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink port=$4 host=$3 &
					PID=$!
					echo $PID > $PIDFile
					echo $3 $4 $w $h > $StreamingStatusFile
				fi
				;;
			stop)
				echo "Stop STREAMING"
				if [ -e "$StreamingStatusFile" ]; then
					rm $StreamingStatusFile
				fi
				if [ -z "$PID" ]; then
					killRaspivid
				fi
				;;
			*)
				echo "Usage $0 $1 {start|stop} host port [width] [height]"
				;;
			esac
		;;

	video)
		case "$2" in
			record)
				ts=`date +%s`
				w=$5
				h=$6
				if [ -z $w ]; then
						w=640
				fi
				if [ -z $h ]; then
						h=480
				fi

				pauseStreaming

				if [ -z "$PID" ]; then
					echo "Start RECORDING"
				
					/usr/bin/raspivid -vf -w $w -h $h -fps 49 -b 2500000 -t 0 -n -pf base -o - | tee /rpicopter/cam/video-$ts.h264 | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink port=$4 host=$3 &
					PID=$!
					echo $PID > $PIDFile
					echo $3 $4 $w $h > $RecordStatusFile
				fi
				;;
			stop)
				echo "Stop RECORDING"
				killRaspivid
				resumeStreaming
				rm $RecordStatusFile
				;;
			*)
				echo "Usage $0 $1 {record|stop} host port [width] [height]"
				;;
		esac
	;;


	takepicture)
		pauseStreaming

		echo "TAKEPICTURE"
		/usr/bin/raspistill -o /rpicopter/cam/image-$2.jpg  -hf -vf -w $photo_width -h $photo_height -ex sports
		#sleep 2

		resumeStreaming

		;;

	*)
		echo "Usage $0 {stream|video|takepicture} [parameters]"
		;;
esac
exit 0