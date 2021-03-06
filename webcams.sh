#!/bin/bash

v4l2-ctl --device=/dev/video0 --set-fmt-video=width=800,height=600,pixelformat=1
v4l2-ctl --device=/dev/video1 --set-fmt-video=width=800,height=600,pixelformat=1

~/workspace/v4l2rtspserver/v4l2rtspserver /dev/video0 /dev/video1

# cvlc --sout=#rtp{sdp=rtsp://192.168.0.100:8080} 'v4l2:///dev/video0:chroma=H264:width=800:height=600:fps=30' &
# cvlc --sout=#rtp{sdp=rtsp://192.168.0.100:8081} 'v4l2:///dev/video1:chroma=H264:width=800:height=600:fps=30' &

# cvlc v4l2:///dev/video0:chroma=h264:width=800:height=600 --sout '#standard{access=http,mux=ts,dst=192.168.0.100:8080,name=stream,mime=video/ts}' &
# cvlc v4l2:///dev/video1:chroma=h264:width=800:height=600 --sout '#standard{access=http,mux=ts,dst=192.168.0.100:8081,name=stream,mime=video/ts}' &