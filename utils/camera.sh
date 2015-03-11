#!/bin/sh
PIDFile="raspivid.pid"
StreamingStatusFile="raspivid.streaming"
RecordStatusFile="raspivid.recording"

video_width=1280
video_height=720
photo_width=2592
photo_height=1944

#/usr/bin/raspivid  -vf -hf -w $video_width -h $video_height -fps 25 -b 5000000 -t 0 -g 20 -n -ex auto -o - | \
#gst-launch-1.0 -v fdsrc ! tee name=tp \
#tp. ! queue ! h264parse ! videorate ! 'video/x-raw-yuv,framerate=10/1' ! videoscale ! 'video/x-raw-yuv,width=640,height=480' ! x264enc pass=qual quantizer=20 tune=zerolatency ! rtph264pay config-interval=1 pt=96 ! udpsink port=$4 host=$3 \
#tp. ! queue ! filesink location=/rpicopter/cam/video-$3.h264

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

toggleRecordPause() {
	kill -USR1 $PID
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
						w=320
				fi
				if [ -z $h ]; then
						h=240
				fi

				echo "Start STREAMING"

				/usr/bin/raspivid -vf -w $w -h $h -fps 10 -b 400000 -t 0 -g 20 -n -ex auto -o - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink port=$4 host=$3 &
				PID=$!
				echo $PID > $PIDFile
				echo $3 $4 $w $h > $StreamingStatusFile
				;;
			stop)
				echo "Stop STREAMING"
				rm $StreamingStatusFile
				killRaspivid
				;;
			*)
				echo "Usage $0 $1 {start|stop} host port [width] [height]"
				;;
			esac
		;;

	video)
		case "$2" in
			record)
				pauseStreaming
				if [ -z "$PID" ]; then
					# TODO use tee in gstreamer to save to file and stream at same time
					echo "Start RECORDING"
					/usr/bin/raspivid -o /rpicopter/cam/video-$3.h264 -vf -hf -w $video_width -h $video_height -fps 25 -b 5000000 -t 0 --signal &
					PID=$!
					echo $PID > $PIDFile
				else
					# resume from pause
					echo "Resume RECORDING"
					toggleRecordPause
				fi
				echo $2 > $RecordStatusFile
				;;

			stop)
				echo "Stop RECORDING"
				killRaspivid
				resumeStreaming
				rm $RecordStatusFile
				;;

			pause)
				echo "Pause RECORDING"
				toggleRecordPause
				echo $2 > $RecordStatusFile
				;;
			*)
				echo "Usage $0 $1 {record|stop|pause} [file-suffix]"
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