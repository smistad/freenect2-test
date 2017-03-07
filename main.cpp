#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <iostream>

int main(int argc, char **argv) {

    bool enable_rgb = true;
    bool enable_depth = true;

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    std::string serial = "";

    if (freenect2.enumerateDevices() == 0) {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    if (serial == "") {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
    pipeline = new libfreenect2::OpenGLPacketPipeline();
    dev = freenect2.openDevice(serial, pipeline);

    int types = 0;
    if (enable_rgb)
        types |= libfreenect2::Frame::Color;
    if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    if (enable_rgb && enable_depth) {
        if (!dev->start())
            return -1;
    } else {
        if (!dev->startStreams(enable_rgb, enable_depth))
            return -1;
    }
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    libfreenect2::Registration *registration = new libfreenect2::Registration(dev->getIrCameraParams(),
                                                                              dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    while (true) {
        if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 sconds
        {
            std::cout << "timeout!" << std::endl;
            return -1;
        }
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];


        registration->apply(rgb, depth, &undistorted, &registered);

        std::cout << "Status: " << registered.status << std::endl;
        std::cout << rgb->width << " " << rgb->bytes_per_pixel << " " << rgb->format << std::endl;
        std::cout << depth->width << " " << depth->bytes_per_pixel << " " << depth->format << std::endl;
        std::cout << registered.bytes_per_pixel << " " << registered.format << std::endl;
        std::cout << undistorted.bytes_per_pixel << " " << undistorted.format << std::endl;

        float* depth_data = (float*)undistorted.data;
        unsigned int* rgb_data = (unsigned int*)registered.data;

        for(int x = 0; x < 512; ++x) {
            for(int y = 0; y < 424; ++y) {
                std::cout << "Depth: " << depth_data[x + y*512] << std::endl;
                const uint8_t* pixel = reinterpret_cast<uint8_t*>(&(rgb_data[x + y*512]));
                uint8_t b = pixel[0];
                uint8_t g = pixel[1];
                uint8_t r = pixel[2];
                std::cout << "Color: " << (int)r << " " << (int)g << " " << (int)b << std::endl;
            }
        }

        listener.release(frames);
    }

    dev->stop();
    dev->close();
}