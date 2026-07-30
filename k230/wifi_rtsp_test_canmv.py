import gc
import network
import os
import sys
import time
import uctypes
import _thread

import multimedia as mm
from media.media import *
from media.sensor import *
from media.vencoder import *


# The K230 creates this local 2.4 GHz access point. Internet is not needed.
AP_SSID = "H_BALL_CAR"
AP_PASSWORD = "hball2026"

RTSP_SESSION = "ball"
RTSP_PORT = 8554
STREAM_WIDTH = 640
STREAM_HEIGHT = 480


def start_wifi_ap():
    ap = network.WLAN(network.AP_IF)
    if not ap.active():
        ap.active(True)

    ap.config(ssid=AP_SSID, key=AP_PASSWORD)
    time.sleep(3)

    ip_info = ap.ifconfig()
    ip_address = ip_info[0]
    print("WiFi AP ready")
    print("SSID:", AP_SSID)
    print("Password:", AP_PASSWORD)
    print("IP:", ip_address)
    return ap, ip_address


class RtspVideoServer:
    def __init__(self, ip_address):
        self.ip_address = ip_address
        self.sensor = None
        self.encoder = None
        self.link = None
        self.rtsp = None
        self.running = False
        self.thread_done = False
        self.sensor_started = False
        self.encoder_created = False
        self.encoder_started = False
        self.rtsp_initialized = False
        self.rtsp_started = False

    def url(self):
        return "rtsp://%s:%d/%s" % (
            self.ip_address,
            RTSP_PORT,
            RTSP_SESSION,
        )

    def start(self):
        width = ALIGN_UP(STREAM_WIDTH, 16)
        height = STREAM_HEIGHT

        self.sensor = Sensor()
        self.sensor.reset()
        self.sensor.set_framesize(width=width, height=height, alignment=12)
        self.sensor.set_pixformat(Sensor.YUV420SP)

        self.encoder = Encoder()
        self.encoder.SetOutBufs(8, width, height)

        channel_attr = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_MAIN,
            width,
            height,
        )
        self.encoder.Create(channel_attr)
        self.encoder_created = True

        self.link = MediaManager.link(
            self.sensor.bind_info()["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.encoder.chn),
        )

        self.rtsp = mm.rtsp_server()
        self.rtsp.rtspserver_init(RTSP_PORT)
        self.rtsp_initialized = True
        self.rtsp.rtspserver_createsession(
            RTSP_SESSION,
            mm.multi_media_type.media_h264,
            False,
        )
        self.rtsp.rtspserver_start()
        self.rtsp_started = True

        self.encoder.Start()
        self.encoder_started = True
        self.sensor.run()
        self.sensor_started = True

        self.running = True
        self.thread_done = False
        _thread.start_new_thread(self._stream_loop, ())
        print("RTSP stream ready:", self.url())

    def _stream_loop(self):
        stream_data = StreamData()
        frame_count = 0

        try:
            while self.running:
                os.exitpoint()
                self.encoder.GetStream(stream_data)
                timestamp = time.ticks_ms()

                for pack_index in range(stream_data.pack_cnt):
                    size = stream_data.data_size[pack_index]
                    payload = bytes(
                        uctypes.bytearray_at(
                            stream_data.data[pack_index],
                            size,
                        )
                    )
                    self.rtsp.rtspserver_sendvideodata(
                        RTSP_SESSION,
                        payload,
                        size,
                        timestamp,
                    )

                self.encoder.ReleaseStream(stream_data)
                frame_count += 1
                if frame_count % 150 == 0:
                    print("RTSP frames:", frame_count)
        except BaseException as error:
            print("RTSP stream stopped:", error)
        finally:
            self.thread_done = True

    def stop(self):
        if self.running:
            self.running = False
            while not self.thread_done:
                time.sleep_ms(20)

        if self.sensor_started:
            try:
                self.sensor.stop()
            except BaseException as error:
                print("Sensor stop warning:", error)
            self.sensor_started = False
        if self.link is not None:
            try:
                self.link.destroy()
            except BaseException as error:
                print("VENC link destroy warning:", error)
            self.link = None
        if self.encoder_started:
            try:
                self.encoder.Stop()
            except BaseException as error:
                print("Encoder stop warning:", error)
            self.encoder_started = False

        if self.encoder_created:
            try:
                self.encoder.Destroy()
            except BaseException as error:
                print("Encoder destroy warning:", error)
            self.encoder_created = False

        if self.rtsp_started:
            try:
                self.rtsp.rtspserver_stop()
            except BaseException as error:
                print("RTSP stop warning:", error)
            self.rtsp_started = False

        if self.rtsp_initialized:
            try:
                self.rtsp.rtspserver_deinit()
            except BaseException as error:
                print("RTSP deinit warning:", error)
            self.rtsp_initialized = False

        gc.collect()


def main():
    ap = None
    server = None

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        ap, ip_address = start_wifi_ap()
        server = RtspVideoServer(ip_address)
        server.start()

        print("Connect the iPad to", AP_SSID)
        print("Open this URL in VLC:", server.url())

        while True:
            os.exitpoint()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped by user")
    except BaseException as error:
        sys.print_exception(error)
    finally:
        if server is not None:
            server.stop()
        if ap is not None:
            ap.active(False)
        gc.collect()


if __name__ == "__main__":
    main()
