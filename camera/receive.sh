export DISPLAY=:0

gst-launch-1.0 udpsrc port=5000 buffer-size=8388608 ! \
    application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! \
    rtpjitterbuffer latency=150 drop-on-latency=false ! \
    rtph264depay ! \
    h264parse ! \
    mppvideodec fast-mode=1 ! \
    xvimagesink sync=false -v