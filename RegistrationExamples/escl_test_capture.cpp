/*
Capture point clouds, with color, from the Zivid camera, and visualize it.
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <iostream>

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: escl_test_capture <output_file.ply>" << std::endl;
        return EXIT_FAILURE;
    }
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Creating settings" << std::endl;
        const auto settings = Zivid::Settings{
            Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{ Zivid::Settings::Acquisition{} } },
            Zivid::Settings::Color{
                Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } }
        };

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture2D3D(settings);

        std::cout << "Setting up visualization" << std::endl;
        Zivid::Visualization::Visualizer visualizer;

        std::cout << "Visualizing point cloud" << std::endl;
        visualizer.showMaximized();
        visualizer.show(frame);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes." << std::endl;
        visualizer.run();

        // make sure this gets assigned as a string
        std::string dataFilePLY = argv[1];

        std::cout << "Saving point cloud to " << dataFilePLY << std::endl;

        // dataFilePLY needs to be a std::string for the save function
        frame.save(dataFilePLY);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
