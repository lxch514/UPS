gst-launch-1.0 v4l2src device=/dev/video41 ! \
    image/jpeg,width=640,height=480,framerate=30/1 ! \
    mppjpegdec ! \
    mpph264enc bps=2500000 rc-mode=cbr gop=15 qp-init=28 ! \
    h264parse ! \
    rtph264pay config-interval=1 pt=96 mtu=1200 ! \
    udpsink host=192.168.100.1 port=5000 sync=false
