ffmpeg -f video4linux2 -qscale 10 -r 12 -s 640x480 -i /dev/video0 -f alsa -i hw:1 -ab 16 -ar 22050 -ac 1 -f mp3 -f flv rtmp://192.168.3.4:5000/rtmpsvr/rtmp1

ffmpeg -r 30 -i /dev/video0 -vcodec h264 -max_delay 100 -f flv -g 5 -b 700000 rtmp://127.0.0.1:5000/rtmpsvr/rtmp1

ffmpeg -r -i /dev/video0 -f rtsp rtsp://192.168.3.18:8554

ffmpeg -f v4l2 -i /dev/video0  -s 640x480 -r 24 -vcodec libx264 -an http:///192.168.3.18:8554

cvlc -vvv v4l2:///dev/video0 --sout '#transcode{vcodec=h264,vb=800,acodec=mp4a}:rtp{sdp=rtsp://:8554/live.ts}' -I dummy