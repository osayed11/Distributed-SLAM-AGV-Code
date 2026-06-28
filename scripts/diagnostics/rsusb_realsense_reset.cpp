#include <librealsense2/rs.hpp>

#include <chrono>
#include <iostream>
#include <thread>

int main()
{
    try
    {
        rs2::context context;
        auto devices = context.query_devices();
        if (devices.size() == 0)
        {
            std::cerr << "No RealSense device found\n";
            return 1;
        }

        auto device = devices.front();
        std::cout << "Resetting " << device.get_info(RS2_CAMERA_INFO_NAME)
                  << " serial " << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "\n";
        device.hardware_reset();
        std::this_thread::sleep_for(std::chrono::seconds(8));
        return 0;
    }
    catch (const rs2::error& e)
    {
        std::cerr << "rs2_error: " << e.what() << "\n";
        return 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "std_error: " << e.what() << "\n";
        return 1;
    }
}
